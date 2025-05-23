#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

//================== 引脚定义 ==================
// 右电机引脚定义
#define DIR_R              (P21_4)          // 右电机方向控制引脚
#define PWM_R              (ATOM1_CH0_P21_2) // 右电机PWM输出引脚

// 左电机引脚定义
#define DIR_L              (P21_5)          // 左电机方向控制引脚
#define PWM_L              (ATOM1_CH1_P21_3) // 左电机PWM输出引脚

//================== 电机控制参数 ==================
#define MOTOR_PWM_MAX      10000            // 电机PWM最大值
#define MOTOR_PWM_MIN      0                // 电机PWM最小值
#define MOTOR_FORWARD      1                // 电机正转方向
#define MOTOR_BACKWARD     0                // 电机反转方向

//================== 函数声明 ==================
void Motor_Init(void);
void Motor_PWM_Control(float motor_left, float motor_right);
void Motor_Stop(void);
void Motor_Speed_Limit(float *speed);

#endif /* CODE_MOTOR_H_ */
