#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

//================== 编码器引脚定义 ==================
// 左编码器配置
#define ENCODER_LEFT              (TIM2_ENCODER)                // 左编码器定时器接口
#define ENCODER_LEFT_A            (TIM2_ENCODER_CH2_P33_6)      // 左编码器A相引脚
#define ENCODER_LEFT_B            (TIM2_ENCODER_CH1_P33_7)      // 左编码器B相引脚

// 右编码器配置
#define ENCODER_RIGHT             (TIM5_ENCODER)                // 右编码器定时器接口
#define ENCODER_RIGHT_A           (TIM5_ENCODER_CH1_P10_3)      // 右编码器A相引脚
#define ENCODER_RIGHT_B           (TIM5_ENCODER_CH2_P10_1)      // 右编码器B相引脚

//================== 编码器参数配置 ==================
#define ENCODER_SAMPLE_TIME       10                           // 编码器采样周期(ms)
#define ENCODER_PPR               1000                         // 编码器每转脉冲数
#define ENCODER_GEAR_RATIO        30                           // 减速比
#define ENCODER_WHEEL_DIAMETER    65                           // 车轮直径(mm)
#define ENCODER_RESOLUTION        (ENCODER_PPR * ENCODER_GEAR_RATIO * 4) // 编码器分辨率(四倍频)

//================== 滤波参数 ==================
#define ENCODER_FILTER_DEPTH      5                            // 滤波深度
#define ENCODER_MAX_SPEED         20000                        // 最大速度限制
#define ENCODER_DEAD_ZONE         5                            // 死区阈值

//================== 全局变量声明 ==================
extern int16 encoder_speed_left;                              // 左编码器速度
extern int16 encoder_speed_right;                             // 右编码器速度
extern int16 encoder_speed_left_filtered;                     // 左编码器滤波后速度
extern int16 encoder_speed_right_filtered;                    // 右编码器滤波后速度
extern float encoder_distance_left;                           // 左轮行驶距离
extern float encoder_distance_right;                          // 右轮行驶距离

//================== 函数声明 ==================
void Encoder_Init(void);
void Encoder_Read_Speed(void);
void Encoder_Filter_Update(void);
void Encoder_Clear_Distance(void);
int16 Encoder_Get_Left_Speed(void);
int16 Encoder_Get_Right_Speed(void);
float Encoder_Get_Left_Distance(void);
float Encoder_Get_Right_Distance(void);
float Encoder_Speed_To_MMS(int16 encoder_count);
void Encoder_Debug_Display(void);

// 兼容原有接口
#define encoder_filter()          Encoder_Read_Speed()
#define encoder_init()            Encoder_Init()
#define Encoder_speed_l           encoder_speed_left
#define Encoder_speed_r           encoder_speed_right

#endif /* CODE_ENCODER_H_ */
