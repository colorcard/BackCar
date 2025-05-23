#include "motor.h"

//================== 电机控制函数 ==================
/*==========================电机初始化==========================*/
/* 功能：初始化电机PWM和方向控制引脚
 * 参数：无
 * 说明：设置PWM频率和初始状态
 */
void Motor_Init(void)
{
    // 初始化方向控制引脚
    gpio_init(DIR_L, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 初始化PWM输出，频率20kHz，初始占空比为0
    pwm_init(PWM_L, 20000, 0);
    pwm_init(PWM_R, 20000, 0);
}

/*==========================电机速度限制==========================*/
/* 功能：限制电机速度在有效范围内
 * 参数：speed - 指向速度值的指针
 * 说明：限制速度在-MOTOR_PWM_MAX到+MOTOR_PWM_MAX之间
 */
void Motor_Speed_Limit(float *speed)
{
    if (*speed > MOTOR_PWM_MAX) {
        *speed = MOTOR_PWM_MAX;
    } else if (*speed < -MOTOR_PWM_MAX) {
        *speed = -MOTOR_PWM_MAX;
    }
}

/*==========================单个电机控制==========================*/
/* 功能：控制单个电机的转速和方向
 * 参数：pwm_pin - PWM引脚, dir_pin - 方向引脚, speed - 速度值
 * 说明：正值正转，负值反转，速度范围0~10000
 */
static void Motor_Single_Control(uint32 pwm_pin, uint32 dir_pin, float speed)
{
    // 限制速度范围
    Motor_Speed_Limit(&speed);

    if (speed >= 0) {
        // 正转
        gpio_set_level(dir_pin, MOTOR_FORWARD);
        pwm_set_duty(pwm_pin, (uint32)speed);
    } else {
        // 反转
        gpio_set_level(dir_pin, MOTOR_BACKWARD);
        pwm_set_duty(pwm_pin, (uint32)(-speed));
    }
}

/*==========================电机PWM控制==========================*/
/* 功能：控制左右电机的转速和方向
 * 参数：motor_left  - 左电机速度（-10000~+10000）
 *       motor_right - 右电机速度（-10000~+10000）
 * 说明：正值为前进方向，负值为后退方向
 */
void Motor_PWM_Control(float motor_left, float motor_right)
{
    // 控制左电机
    Motor_Single_Control(PWM_L, DIR_L, motor_left);

    // 控制右电机
    Motor_Single_Control(PWM_R, DIR_R, motor_right);
}

/*==========================电机停止==========================*/
/* 功能：停止所有电机
 * 参数：无
 * 说明：将PWM占空比设为0，电机停转
 */
void Motor_Stop(void)
{
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
}

/*==========================电机调试控制（可选）==========================*/
/* 功能：带调试信息的电机控制
 * 参数：motor_left  - 左电机速度
 *       motor_right - 右电机速度
 * 说明：在屏幕上显示电机控制信息
 */
void Motor_PWM_Control_Debug(float motor_left, float motor_right)
{
    // 执行电机控制
    Motor_PWM_Control(motor_left, motor_right);

    // 显示调试信息
    ips200_show_float(0, MT9V03X_H + 16 * 14, motor_left, 5, 1);   // 显示左电机速度
    ips200_show_float(0, MT9V03X_H + 16 * 15, motor_right, 5, 1);  // 显示右电机速度

    // 显示电机方向
    ips200_show_string(120, MT9V03X_H + 16 * 14, motor_left >= 0 ? "FWD" : "BWD");
    ips200_show_string(120, MT9V03X_H + 16 * 15, motor_right >= 0 ? "FWD" : "BWD");
}
