#include "Servo.h"

//================== ������ƺ��� ==================
/*==========================�����ʼ��==========================*/
/* ���ܣ���ʼ�����PWM��PID������
 * ������PID�������ĸ������
 * ˵����������Ʒ�Χ520~720����ֵ620
 */
void Servo_Init(float Integrate_max,
                float Integrate_min,
                float Output_max,
                float Output_min,
                float Filter_value,
                float kp,
                float ki,
                float kd)
{
    // ��ʼ�����PWM��Ƶ��50Hz����ʼռ�ձ�Ϊ��ֵ
    pwm_init(PWM_SERVO, 50, MID_VALUE);

    // ��ʼ�����PID������
    Servo_PID_Init(Integrate_max,
                   Integrate_min,
                   Output_max,
                   Output_min,
                   Filter_value,
                   kp,
                   ki,
                   kd);
}

/*==========================���ƫ�ƿ���==========================*/
/* ���ܣ����ݵ�ǰƫ�������ƶ��ת��
 * ������current_onto - ��ǰƫ��������ֵ��ƫ����ֵ��ƫ��
 * ˵����ͨ��PID�������������������������Ч��Χ��
 */
void Servo_Onto_Control(float current_onto)
{
    int16 servo_pwm_value;

    // 1. ͨ��PID�������������ֵ
    float pid_output = Servo_PID_Control(current_onto);

    // 2. ת��Ϊ���PWMֵ��ȡ������Ϊƫ�Ʒ�������ת���෴��
    servo_pwm_value = -(int16)pid_output + MID_VALUE;

    // 3. ����PWMֵ����Ч��Χ�ڣ�520~720��
    if (servo_pwm_value > 720) {
        servo_pwm_value = 720;
    } else if (servo_pwm_value < 520) {
        servo_pwm_value = 520;
    }

    // 4. ���ö��PWM���
    pwm_set_duty(PWM_SERVO, servo_pwm_value);

    // 5. ��ʾ������Ϣ��ƫ��������Χ-100~+100��
    ips200_show_float(0 * 16, MT9V03X_H + 16 * 11, servo_pwm_value - MID_VALUE, 3, 2);
}
