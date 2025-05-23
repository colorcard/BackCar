#include "pid.h"

//================== ȫ�ֱ������� ==================
PID SE_PID;

//================== PID���������� ==================
/*==========================PID��������ʼ��==========================*/
void Servo_PID_Init(float Integrate_max,
                    float Integrate_min,
                    float Output_max,
                    float Output_min,
                    float Filter_value,
                    float kp,
                    float ki,
                    float kd)
{
    // PID��������
    SE_PID.Kp = kp;
    SE_PID.Ki = ki;
    SE_PID.Kd = kd;

    // ���ֵ��ʼ��
    SE_PID.current_error = 0.0f;
    SE_PID.last_error = 0.0f;
    SE_PID.before_last_error = 0.0f;

    // ������ز���
    SE_PID.integrate_val = 0.0f;
    SE_PID.integrate_max = Integrate_max;
    SE_PID.integrate_min = Integrate_min;

    // �����ز���
    SE_PID.output_value = 0.0f;
    SE_PID.output_max = Output_max;
    SE_PID.output_min = Output_min;

    // �˲���������
    SE_PID.filter_value = Filter_value;
}

/*==========================PID����������==========================*/
float Servo_PID_Control(float current_input)
{
    // 1. ���ݸ���
    SE_PID.before_last_error = SE_PID.last_error;
    SE_PID.last_error = SE_PID.current_error;
    SE_PID.current_error = current_input;

    // 2. �����ۼ�
    SE_PID.integrate_val += SE_PID.current_error;

    // 3. �������ã�����С���𵴣�
    if (SE_PID.current_error > -SE_PID.filter_value &&
        SE_PID.current_error < SE_PID.filter_value) {
        SE_PID.integrate_val = 0.0f;
    }

    // 4. �����޷�����ֹ���ֱ��ͣ�
    if (SE_PID.integrate_val >= SE_PID.integrate_max) {
        SE_PID.integrate_val = SE_PID.integrate_max;
    } else if (SE_PID.integrate_val <= SE_PID.integrate_min) {
        SE_PID.integrate_val = SE_PID.integrate_min;
    }

    // 5. PID�������
    // P���������
    const float p_term = SE_PID.Kp * SE_PID.current_error;

    // I����ֿ���
    const float i_term = SE_PID.Ki * SE_PID.integrate_val;

    // D�΢�ֿ��ƣ�ʹ�ö��ײ�֣�
    const float d_term = SE_PID.Kd * (SE_PID.before_last_error +
                                      SE_PID.current_error -
                                      2.0f * SE_PID.last_error);

    // �����
    SE_PID.output_value = p_term + i_term + d_term;

    // 6. ����޷�
    if (SE_PID.output_value >= SE_PID.output_max) {
        SE_PID.output_value = SE_PID.output_max;
    } else if (SE_PID.output_value <= SE_PID.output_min) {
        SE_PID.output_value = SE_PID.output_min;
    }

    return SE_PID.output_value;
}

/*==========================��ȡPID����==========================*/
float Servo_PID_Get_Output(void)
{
    return SE_PID.output_value;
}

float Servo_PID_Get_Error(void)
{
    return SE_PID.current_error;
}

float Servo_PID_Get_Integral(void)
{
    return SE_PID.integrate_val;
}

/*==========================PID��������==========================*/
void Servo_PID_Set_Params(float kp, float ki, float kd)
{
    SE_PID.Kp = kp;
    SE_PID.Ki = ki;
    SE_PID.Kd = kd;
}

void Servo_PID_Reset(void)
{
    SE_PID.current_error = 0.0f;
    SE_PID.last_error = 0.0f;
    SE_PID.before_last_error = 0.0f;
    SE_PID.integrate_val = 0.0f;
    SE_PID.output_value = 0.0f;
}
