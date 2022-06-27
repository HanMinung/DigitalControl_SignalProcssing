#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

/*===========================================================================================================================*/
/*                                                 �ȵǸ� �ǰ� �϶� �ݵ��.                                                  */
/*===========================================================================================================================*/

/*===========================================================================================================================*/
/*                                                 GIMBALL MOTOR CONTROL                                                     */
/*===========================================================================================================================*/

#include "Run_func.h"

double      Buf_Time[N_DATA] = { 0,0 };
double      Buf_Input_Vout[N_DATA] = { 0,0 };
double      Buf_Input_Fmtr[N_DATA] = { 0,0 };
double      Buf_Output_Vss[N_DATA] = { 0,0 };
double      Buf_Output_Vcmd[N_DATA] = { 0,0 };

double      Alpha_L = 0.0;
double      Alpha_R = 0.0;
double      Beta_L = 0.0;
double      Beta_R = 0.0;

double      a_L = 0.0;
double      a_R = 0.0;
double      b_L = 0.0;
double      b_R = 0.0;

float64     Voltage_Gyro = 0.0;
double      W_gyro = 0.0;
double      Out_Voltage_mean = 0.0;


/* forloop buffer */

double      Buf_SumData[N_DATA] = { 0.0, };
double      Buf_SumAngVel[N_DATA] = { 0.0, };
double      TaskTime[N_DATA] = { 0.0, };
double      Buf_SumOffset[N_DATA] = { 0,0 };

/* while loop buffer */

double      Buf_Outdata[N_DATA] = { 0.0, };
double      Buf_Inputdata[N_DATA] = { 0.0, };

double      Buf_Wgyro[N] = { 0,0 };

TaskHandle   taskAI2 = 0.0;

/*===========================================================================================================================*/
/*                                                  START MAIN CODE                                                          */
/*===========================================================================================================================*/

/****************************************************************************************/
/*                                  N VALUE    :  101                                   */
/*                                  FINAL TIME :  8 sec                                 */
/****************************************************************************************/

void main(void) {

    FILE* pFile;
    //FILE* fFile;

    DAQmxCreateTask("", &taskAI2);

    DAQmxCreateAIVoltageChan(taskAI2, "Dev2/ai2", "", DAQmx_Val_Diff, 0, 10.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI2);

    /* Channel Creation , Channel Configuration , Starting the channel */
    DAQ_channel_creation();

    DAQ_channel_config();

    DAQ_channel_start();

    // MOTOR FLAG ON & Initialization
    DAQmxWriteAnalogScalarF64(taskAO0, 0.0, 5.0, 5.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);

    Start_presskey();

    /*===========================================================================================================================*/

    Alpha_L = 0.0819001 ;
    Alpha_R = 0.07859 ;
    Beta_L = 1.01384 ;
    Beta_R = 1.018007 ;

    a_L = 0.42149 ;
    b_L = 0.24909 ;
    a_R = 0.42586 ;
    b_R = 0.067538 ;

    /*===========================================================================================================================*/

    EMERGENCY_STOP;

    FOR_LOOP(i, N) {    // Essential Option : Setting Proper Value of N

        /*Time = SAMPLING_TIME * count;*/

        // N�� ���� �ʿ� : 101

        Ref_Voltage = 0.05 * i;

        if (0 <= Ref_Voltage && Ref_Voltage <= 2.3775) {

            Linearized_Vcmd = (Alpha_L * Ref_Voltage + Beta_L - b_L) / a_L;

        }

        if (Ref_Voltage >= 2.3776 && Ref_Voltage <= 2.5776) Linearized_Vcmd = Ref_Voltage;

        if (2.5775 <= Ref_Voltage && Ref_Voltage <= 5.00) {

            Linearized_Vcmd = (Alpha_R * Ref_Voltage + Beta_R - b_R) / a_R;

        }

        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, Linearized_Vcmd, NULL);

        count = 0; // do while ���� ���� count �ʱ�ȭ


        do {

            // PIN ���� �ʿ� AI1 OR AI2
            DAQmxReadAnalogScalarF64(taskAI2, -1, &Voltage_Gyro, NULL);

            Buf_SumData[count] = Voltage_Gyro;

            EMERGENCY_STOP;

            Idle_time();

            Time_prev = Time_curr;

            Time = SAMPLING_TIME * count;

            count++;

        } while (Time < FINAL_TIME);

        // 1000���� ������ �� 400 ~ 900���� �����͸� ����

        EMERGENCY_STOP;

        // DATA SIZE : 1600��

        for (int i = 500; i < (N_DATA - 100); i++) {

            Out_Voltage_mean += Buf_SumData[i];

        }

        //// �̰� ������ �� ���� ���� �ؾ���
        Buf_Inputdata[i] = Ref_Voltage;
        Buf_Outdata[i] = Out_Voltage_mean / 1000;
        // Calculation of W_gyro
        Buf_Wgyro[i] = ( Buf_Outdata[i] - 1.21418) * 1000/0.67; //  [deg/sec]

        /*   Update Voltage Level   */

        //  DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);
        Out_Voltage_mean = 0.0;

        DAQmxWriteAnalogScalarF64(taskAO1, 0.0, 5.0, 2.5, NULL);

        Sleep(2000);

        printf("Vc : %lf , Vcmd : %lf , OutVol : %lf , Out_W_Gyro : %lf \n", Ref_Voltage, Linearized_Vcmd, Buf_Outdata[i], Buf_Wgyro[i]);
   

    }

    /*===========================================================================================================================*/
    /*                                             Please FINISH with 2.5V AND FLAG OFF                                          */
    /*===========================================================================================================================*/

    void DAQ_write_set(void);

    // DAQ stop task 
    void DAQ_stop_task(void);

    void DAQ_clear_task(void);


    sprintf(OutFileName_1, "Gimbal_Characteristic_Linearization(5.25)");

    pFile = fopen(strcat(OutFileName_1, "_data.txt"), "w+t");

    for (int idx = 0; idx < N; idx++)
    {
        fprintf(pFile, "%20.10f\t%20.10f\t%20.10f \n", Buf_Inputdata[idx], Buf_Outdata[idx], Buf_Wgyro[idx]);
    }

}
