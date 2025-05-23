#include "Servo.h"

//================== 舵机控制函数 ==================
/*==========================舵机初始化==========================*/
/* 功能：初始化舵机PWM和PID控制器
 * 参数：PID控制器的各项参数
 * 说明：舵机控制范围520~720，中值620
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
    // 初始化舵机PWM，频率50Hz，初始占空比为中值
    pwm_init(PWM_SERVO, 50, MID_VALUE);

    // 初始化舵机PID控制器
    Servo_PID_Init(Integrate_max,
                   Integrate_min,
                   Output_max,
                   Output_min,
                   Filter_value,
                   kp,
                   ki,
                   kd);
}

/*==========================舵机偏移控制==========================*/
/* 功能：根据当前偏移量控制舵机转向
 * 参数：current_onto - 当前偏移量（正值右偏，负值左偏）
 * 说明：通过PID控制器计算输出，并限制在有效范围内
 */
void Servo_Onto_Control(float current_onto)
{
    int16 servo_pwm_value;

    // 1. 通过PID控制器计算输出值
    float pid_output = Servo_PID_Control(current_onto);

    // 2. 转换为舵机PWM值（取反是因为偏移方向与舵机转向相反）
    servo_pwm_value = -(int16)pid_output + MID_VALUE;

    // 3. 限制PWM值在有效范围内（520~720）
    if (servo_pwm_value > 720) {
        servo_pwm_value = 720;
    } else if (servo_pwm_value < 520) {
        servo_pwm_value = 520;
    }

    // 4. 设置舵机PWM输出
    pwm_set_duty(PWM_SERVO, servo_pwm_value);

    // 5. 显示调试信息（偏移量，范围-100~+100）
    ips200_show_float(0 * 16, MT9V03X_H + 16 * 11, servo_pwm_value - MID_VALUE, 3, 2);
}
