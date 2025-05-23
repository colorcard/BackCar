#include "pid.h"

//================== 全局变量定义 ==================
PID SE_PID;

//================== PID控制器函数 ==================
/*==========================PID控制器初始化==========================*/
void Servo_PID_Init(float Integrate_max,
                    float Integrate_min,
                    float Output_max,
                    float Output_min,
                    float Filter_value,
                    float kp,
                    float ki,
                    float kd)
{
    // PID参数设置
    SE_PID.Kp = kp;
    SE_PID.Ki = ki;
    SE_PID.Kd = kd;

    // 误差值初始化
    SE_PID.current_error = 0.0f;
    SE_PID.last_error = 0.0f;
    SE_PID.before_last_error = 0.0f;

    // 积分相关参数
    SE_PID.integrate_val = 0.0f;
    SE_PID.integrate_max = Integrate_max;
    SE_PID.integrate_min = Integrate_min;

    // 输出相关参数
    SE_PID.output_value = 0.0f;
    SE_PID.output_max = Output_max;
    SE_PID.output_min = Output_min;

    // 滤波死区参数
    SE_PID.filter_value = Filter_value;
}

/*==========================PID控制器运算==========================*/
float Servo_PID_Control(float current_input)
{
    // 1. 数据更新
    SE_PID.before_last_error = SE_PID.last_error;
    SE_PID.last_error = SE_PID.current_error;
    SE_PID.current_error = current_input;

    // 2. 积分累计
    SE_PID.integrate_val += SE_PID.current_error;

    // 3. 死区设置（消除小幅震荡）
    if (SE_PID.current_error > -SE_PID.filter_value &&
        SE_PID.current_error < SE_PID.filter_value) {
        SE_PID.integrate_val = 0.0f;
    }

    // 4. 积分限幅（防止积分饱和）
    if (SE_PID.integrate_val >= SE_PID.integrate_max) {
        SE_PID.integrate_val = SE_PID.integrate_max;
    } else if (SE_PID.integrate_val <= SE_PID.integrate_min) {
        SE_PID.integrate_val = SE_PID.integrate_min;
    }

    // 5. PID输出运算
    // P项：比例控制
    const float p_term = SE_PID.Kp * SE_PID.current_error;

    // I项：积分控制
    const float i_term = SE_PID.Ki * SE_PID.integrate_val;

    // D项：微分控制（使用二阶差分）
    const float d_term = SE_PID.Kd * (SE_PID.before_last_error +
                                      SE_PID.current_error -
                                      2.0f * SE_PID.last_error);

    // 总输出
    SE_PID.output_value = p_term + i_term + d_term;

    // 6. 输出限幅
    if (SE_PID.output_value >= SE_PID.output_max) {
        SE_PID.output_value = SE_PID.output_max;
    } else if (SE_PID.output_value <= SE_PID.output_min) {
        SE_PID.output_value = SE_PID.output_min;
    }

    return SE_PID.output_value;
}

/*==========================获取PID参数==========================*/
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

/*==========================PID参数调整==========================*/
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
