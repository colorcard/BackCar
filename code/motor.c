#include "motor.h"

//����ٶ����ã�0~10000��
void PWM_motor(float motor_1,float motor_2)
{
    if(motor_1<=0)             //���1��ת
        {
           motor_1 = -motor_1;
           gpio_set_level(DIR_L, 0);                    // DIR����
           pwm_set_duty(PWM_L, motor_1);                // PWMռ�ձ�
        }
        else
        {

           gpio_set_level(DIR_L, 1 );    //���1����                            // DIR����͵�ƽ
           pwm_set_duty(PWM_L, motor_1);                // ����ռ�ձ�=motor_1/1000000
        }

        if(motor_2<=0)       //���2��ת
           {
              motor_2 = -motor_2;
              gpio_set_level(DIR_R, 0);                    // DIR����͵�ƽ
              pwm_set_duty(PWM_R, motor_2);                // ����ռ�ձ�
           }
           else             //���2��ת
           {

              gpio_set_level(DIR_R,1);                    // DIR����͵�ƽ
              pwm_set_duty(PWM_R, motor_2);                // ����ռ�ձ�
           }
}

void ws_motor_init(void)
{
    pwm_init(PWM_R, 5000,0);                        // PWM ͨ����ʼ��Ƶ��50Hz ռ�ձȳ�ʼΪ 0
    pwm_init(PWM_L, 5000,0);
}
