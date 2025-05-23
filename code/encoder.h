/*
作者：Charon and 快乐牌小刀片
       未经授权禁止转售
 */


#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

#define ENCODER_l                 (TIM2_ENCODER)                         // 正交编码器对应使用的编码器接口
#define ENCODER_l_A               (TIM2_ENCODER_CH2_P33_6)               // A 相对应的引脚
#define ENCODER_l_B               (TIM2_ENCODER_CH1_P33_7)               // B 相对应的引脚
#define ENCODER_r                 (TIM5_ENCODER)                         // 正交编码器对应使用的编码器接口
#define ENCODER_r_A               (TIM5_ENCODER_CH1_P10_3)               // A 相对应的引脚
#define ENCODER_r_B               (TIM5_ENCODER_CH2_P10_1)               // B 相对应的引脚




void encoder_init(void);
void encoder_filter(void);


extern int16 Encoder_speed_l;
extern int16 Encoder_speed_r;

#endif /* CODE_ENCODER_H_ */
