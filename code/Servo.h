#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_

#include "zf_common_headfile.h"
#include "pid.h"

//================== 宏定义 ==================
#define PWM_SERVO           (ATOM0_CH1_P33_9)    // 舵机PWM输出引脚
#define MID_VALUE           620                   // 舵机中值（范围520~720）

//================== 函数声明 ==================
void Servo_Init(float Integrate_max,
                float Integrate_min,
                float Output_max,
                float Output_min,
                float Filter_value,
                float kp,
                float ki,
                float kd);

void Servo_Onto_Control(float current_onto);

#endif /* CODE_SERVO_H_ */
