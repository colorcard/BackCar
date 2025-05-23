#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"


//定义图像信息
#define IMAGE_WID      188
#define IMAGE_HIGH     120
//裁剪左右边图像，排除边缘黑圈干扰
#define Left_boundary  20
#define Right_boundary 168

//赛道边界状态结构体
typedef struct {
    float slope;          // 斜率 (m)
    float intercept;      // 截距 (b)
    float angle;          //角度  (。)
    float level_distance; //平均距离
} LineFitResult;

extern Parking_struct parking;

//uint8 road_boundary[Right_boundary-Left_boundary] = {0};

#define white_pixel 255    //图像黑
#define black_pixel 0      //图像白

//uint8 threshold_value;       //储存阈值信息

void OtsuThreshold(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8_t img_out[MT9V03X_H][MT9V03X_W]);
//当前角度获取
LineFitResult calculate_angle_and_intercept(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
void parking_struct_init(void);
float calculate_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_distance(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_image_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
uint8 grape_broom_monitor_parking(uint8_t img_in[MT9V03X_H][MT9V03X_W],uint8 start_line,uint8 range);
//void draw_subline(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
//当前靠边界位置获取

//位置判定
#endif /* CODE_IMAGE_H_ */
