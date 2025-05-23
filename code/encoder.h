#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

//================== ���������Ŷ��� ==================
// �����������
#define ENCODER_LEFT              (TIM2_ENCODER)                // ���������ʱ���ӿ�
#define ENCODER_LEFT_A            (TIM2_ENCODER_CH2_P33_6)      // �������A������
#define ENCODER_LEFT_B            (TIM2_ENCODER_CH1_P33_7)      // �������B������

// �ұ���������
#define ENCODER_RIGHT             (TIM5_ENCODER)                // �ұ�������ʱ���ӿ�
#define ENCODER_RIGHT_A           (TIM5_ENCODER_CH1_P10_3)      // �ұ�����A������
#define ENCODER_RIGHT_B           (TIM5_ENCODER_CH2_P10_1)      // �ұ�����B������

//================== �������������� ==================
#define ENCODER_SAMPLE_TIME       10                           // ��������������(ms)
#define ENCODER_PPR               1000                         // ������ÿת������
#define ENCODER_GEAR_RATIO        30                           // ���ٱ�
#define ENCODER_WHEEL_DIAMETER    65                           // ����ֱ��(mm)
#define ENCODER_RESOLUTION        (ENCODER_PPR * ENCODER_GEAR_RATIO * 4) // �������ֱ���(�ı�Ƶ)

//================== �˲����� ==================
#define ENCODER_FILTER_DEPTH      5                            // �˲����
#define ENCODER_MAX_SPEED         20000                        // ����ٶ�����
#define ENCODER_DEAD_ZONE         5                            // ������ֵ

//================== ȫ�ֱ������� ==================
extern int16 encoder_speed_left;                              // ��������ٶ�
extern int16 encoder_speed_right;                             // �ұ������ٶ�
extern int16 encoder_speed_left_filtered;                     // ��������˲����ٶ�
extern int16 encoder_speed_right_filtered;                    // �ұ������˲����ٶ�
extern float encoder_distance_left;                           // ������ʻ����
extern float encoder_distance_right;                          // ������ʻ����

//================== �������� ==================
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

// ����ԭ�нӿ�
#define encoder_filter()          Encoder_Read_Speed()
#define encoder_init()            Encoder_Init()
#define Encoder_speed_l           encoder_speed_left
#define Encoder_speed_r           encoder_speed_right

#endif /* CODE_ENCODER_H_ */
