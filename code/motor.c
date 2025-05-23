#include "motor.h"

//电机速度设置（0~10000）
void PWM_motor(float motor_1,float motor_2)
{
    if(motor_1<=0)             //电机1反转
        {
           motor_1 = -motor_1;
           gpio_set_level(DIR_L, 0);                    // DIR方向
           pwm_set_duty(PWM_L, motor_1);                // PWM占空比
        }
        else
        {

           gpio_set_level(DIR_L, 1 );    //电机1正传                            // DIR输出低电平
           pwm_set_duty(PWM_L, motor_1);                // 计算占空比=motor_1/1000000
        }

        if(motor_2<=0)       //电机2反转
           {
              motor_2 = -motor_2;
              gpio_set_level(DIR_R, 0);                    // DIR输出低电平
              pwm_set_duty(PWM_R, motor_2);                // 计算占空比
           }
           else             //电机2正转
           {

              gpio_set_level(DIR_R,1);                    // DIR输出低电平
              pwm_set_duty(PWM_R, motor_2);                // 计算占空比
           }
}

void ws_motor_init(void)
{
    pwm_init(PWM_R, 5000,0);                        // PWM 通道初始化频率50Hz 占空比初始为 0
    pwm_init(PWM_L, 5000,0);
}
