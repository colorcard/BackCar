#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

//================== ͼ��������� ==================
#define IMAGE_WID      188
#define IMAGE_HIGH     120

// �ü����ұ�ͼ���ų���Ե��Ȧ����
#define LEFT_BOUNDARY  20
#define RIGHT_BOUNDARY 168

// ����ֵ����
#define WHITE_PIXEL 255    // ͼ���
#define BLACK_PIXEL 0      // ͼ���

//================== �ṹ�嶨�� ==================
// �����߽�״̬�ṹ��
typedef struct {
    float slope;          // б�� (m)
    float intercept;      // �ؾ� (b)
    float angle;          // �Ƕ� (��)
    float level_distance; // ƽ������
} LineFitResult;

//================== �ⲿ�������� ==================
extern Parking_struct parking;
extern float distance_to_side;
extern float angle_by_image;
extern uint8 threshold_value;

//================== �������� ==================
// ͼ������
void OtsuThreshold(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8_t img_out[MT9V03X_H][MT9V03X_W]);

// �ǶȺ;�����㺯��
LineFitResult calculate_angle_and_intercept(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_distance(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_image_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]);

// ͣ����غ���
void parking_struct_init(void);
uint8 parking_condition_scan(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8 start_line, uint8 range);
uint8 grape_broom_monitor_parking(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8 start_line, uint8 range);

// �ڲ����������������Ҫ�������ļ���ʹ�ã�
//static void display_calculation_results(const LineFitResult* result);
//static void draw_parking_assist_lines(uint8 start_line, uint8 range);

#endif /* CODE_IMAGE_H_ */
