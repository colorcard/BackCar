/*
���ߣ�Charon and ������С��Ƭ
       δ����Ȩ��ֹת��
 */


#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

#define ENCODER_l                 (TIM2_ENCODER)                         // ������������Ӧʹ�õı������ӿ�
#define ENCODER_l_A               (TIM2_ENCODER_CH2_P33_6)               // A ���Ӧ������
#define ENCODER_l_B               (TIM2_ENCODER_CH1_P33_7)               // B ���Ӧ������
#define ENCODER_r                 (TIM5_ENCODER)                         // ������������Ӧʹ�õı������ӿ�
#define ENCODER_r_A               (TIM5_ENCODER_CH1_P10_3)               // A ���Ӧ������
#define ENCODER_r_B               (TIM5_ENCODER_CH2_P10_1)               // B ���Ӧ������




void encoder_init(void);
void encoder_filter(void);


extern int16 Encoder_speed_l;
extern int16 Encoder_speed_r;

#endif /* CODE_ENCODER_H_ */
