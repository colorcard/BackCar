#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_

#include "zf_common_headfile.h"

//================== 图像参数定义 ==================
#define IMAGE_WID      188
#define IMAGE_HIGH     120

// 裁剪左右边图像，排除边缘黑圈干扰
#define LEFT_BOUNDARY  20
#define RIGHT_BOUNDARY 168

// 像素值定义
#define WHITE_PIXEL 255    // 图像白
#define BLACK_PIXEL 0      // 图像黑

//================== 结构体定义 ==================
// 赛道边界状态结构体
typedef struct {
    float slope;          // 斜率 (m)
    float intercept;      // 截距 (b)
    float angle;          // 角度 (度)
    float level_distance; // 平均距离
} LineFitResult;

//================== 外部变量声明 ==================
extern Parking_struct parking;
extern float distance_to_side;
extern float angle_by_image;
extern uint8 threshold_value;

//================== 函数声明 ==================
// 图像处理函数
void OtsuThreshold(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8_t img_out[MT9V03X_H][MT9V03X_W]);

// 角度和距离计算函数
LineFitResult calculate_angle_and_intercept(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_distance(uint8_t img_in[MT9V03X_H][MT9V03X_W]);
float get_image_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]);

// 停车相关函数
void parking_struct_init(void);
uint8 parking_condition_scan(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8 start_line, uint8 range);
uint8 grape_broom_monitor_parking(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8 start_line, uint8 range);

// 内部辅助函数（如果需要在其他文件中使用）
//static void display_calculation_results(const LineFitResult* result);
//static void draw_parking_assist_lines(uint8 start_line, uint8 range);

#endif /* CODE_IMAGE_H_ */
