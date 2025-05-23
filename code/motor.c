#include "motor.h"

//================== ������ƺ��� ==================
/*==========================�����ʼ��==========================*/
/* ���ܣ���ʼ�����PWM�ͷ����������
 * ��������
 * ˵��������PWMƵ�ʺͳ�ʼ״̬
 */
void Motor_Init(void)
{
    // ��ʼ�������������
    gpio_init(DIR_L, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // ��ʼ��PWM�����Ƶ��20kHz����ʼռ�ձ�Ϊ0
    pwm_init(PWM_L, 20000, 0);
    pwm_init(PWM_R, 20000, 0);
}

/*==========================����ٶ�����==========================*/
/* ���ܣ����Ƶ���ٶ�����Ч��Χ��
 * ������speed - ָ���ٶ�ֵ��ָ��
 * ˵���������ٶ���-MOTOR_PWM_MAX��+MOTOR_PWM_MAX֮��
 */
void Motor_Speed_Limit(float *speed)
{
    if (*speed > MOTOR_PWM_MAX) {
        *speed = MOTOR_PWM_MAX;
    } else if (*speed < -MOTOR_PWM_MAX) {
        *speed = -MOTOR_PWM_MAX;
    }
}

/*==========================�����������==========================*/
/* ���ܣ����Ƶ��������ת�ٺͷ���
 * ������pwm_pin - PWM����, dir_pin - ��������, speed - �ٶ�ֵ
 * ˵������ֵ��ת����ֵ��ת���ٶȷ�Χ0~10000
 */
static void Motor_Single_Control(uint32 pwm_pin, uint32 dir_pin, float speed)
{
    // �����ٶȷ�Χ
    Motor_Speed_Limit(&speed);

    if (speed >= 0) {
        // ��ת
        gpio_set_level(dir_pin, MOTOR_FORWARD);
        pwm_set_duty(pwm_pin, (uint32)speed);
    } else {
        // ��ת
        gpio_set_level(dir_pin, MOTOR_BACKWARD);
        pwm_set_duty(pwm_pin, (uint32)(-speed));
    }
}

/*==========================���PWM����==========================*/
/* ���ܣ��������ҵ����ת�ٺͷ���
 * ������motor_left  - �����ٶȣ�-10000~+10000��
 *       motor_right - �ҵ���ٶȣ�-10000~+10000��
 * ˵������ֵΪǰ�����򣬸�ֵΪ���˷���
 */
void Motor_PWM_Control(float motor_left, float motor_right)
{
    // ��������
    Motor_Single_Control(PWM_L, DIR_L, motor_left);

    // �����ҵ��
    Motor_Single_Control(PWM_R, DIR_R, motor_right);
}

/*==========================���ֹͣ==========================*/
/* ���ܣ�ֹͣ���е��
 * ��������
 * ˵������PWMռ�ձ���Ϊ0�����ͣת
 */
void Motor_Stop(void)
{
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
}

/*==========================������Կ��ƣ���ѡ��==========================*/
/* ���ܣ���������Ϣ�ĵ������
 * ������motor_left  - �����ٶ�
 *       motor_right - �ҵ���ٶ�
 * ˵��������Ļ����ʾ���������Ϣ
 */
void Motor_PWM_Control_Debug(float motor_left, float motor_right)
{
    // ִ�е������
    Motor_PWM_Control(motor_left, motor_right);

    // ��ʾ������Ϣ
    ips200_show_float(0, MT9V03X_H + 16 * 14, motor_left, 5, 1);   // ��ʾ�����ٶ�
    ips200_show_float(0, MT9V03X_H + 16 * 15, motor_right, 5, 1);  // ��ʾ�ҵ���ٶ�

    // ��ʾ�������
    ips200_show_string(120, MT9V03X_H + 16 * 14, motor_left >= 0 ? "FWD" : "BWD");
    ips200_show_string(120, MT9V03X_H + 16 * 15, motor_right >= 0 ? "FWD" : "BWD");
}
