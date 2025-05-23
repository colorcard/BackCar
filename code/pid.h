#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

//================== 结构体定义 ==================
// PID控制器结构体
typedef struct {
    float Kp;                    // 比例系数
    float Ki;                    // 积分系数
    float Kd;                    // 微分系数

    // 误差
    float current_error;         // 当前误差
    float last_error;            // 上一次误差
    float before_last_error;     // 上上次误差

    // 积分限幅及其限幅
    float integrate_val;         // 积分值
    float integrate_max;         // 积分上限
    float integrate_min;         // 积分下限

    // 积分死区
    float filter_value;          // 死区范围

    // 位置PID累加变量
    float output_value;          // 输出值

    // 输出限幅
    float output_max;            // 输出上限
    float output_min;            // 输出下限
} PID;

//================== 外部变量声明 ==================
extern PID SE_PID;

//================== 函数声明 ==================
void Servo_PID_Init(float Integrate_max,
                    float Integrate_min,
                    float Output_max,
                    float Output_min,
                    float Filter_value,
                    float kp,
                    float ki,
                    float kd);

float Servo_PID_Control(float current_onto);

#endif /* CODE_PID_H_ */
