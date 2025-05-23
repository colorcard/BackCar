#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"


//����ͼ����Ϣ
#define IMAGE_WID      188
#define IMAGE_HIGH     120

//�ü����ұ�ͼ���ų���Ե��Ȧ����
#define LEFT_BOUNDARY  20
#define RIGHT_BOUNDARY 168

//�����߽�״̬�ṹ��
typedef struct {
    float slope;          // б�� (m)
    float intercept;      // �ؾ� (b)
    float angle;          //�Ƕ�  (��)
    float level_distance; //ƽ������
} LineFitResult;

extern Parking_struct parking;


#define white_pixel 255    //ͼ���
#define black_pixel 0      //ͼ���


void OtsuThreshold(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8_t img_out[MT9V03X_H][MT9V03X_W]);
//��ǰ�ǶȻ�ȡ
LineFitResult calculate_angle_and_intercept(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
void parking_struct_init(void);
float calculate_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_distance(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_image_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
uint8 grape_broom_monitor_parking(uint8_t img_in[MT9V03X_H][MT9V03X_W],uint8 start_line,uint8 range);
//void draw_subline(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
//��ǰ���߽�λ�û�ȡ

//λ���ж�
#endif /* CODE_IMAGE_H_ */
