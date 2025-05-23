#include "encoder.h"

//================== 全局变量定义 ==================
int16 encoder_speed_left = 0;                    // 左编码器当前速度
int16 encoder_speed_right = 0;                   // 右编码器当前速度
int16 encoder_speed_left_filtered = 0;           // 左编码器滤波后速度
int16 encoder_speed_right_filtered = 0;          // 右编码器滤波后速度

// 距离累计变量
float encoder_distance_left = 0.0f;              // 左轮累计距离(mm)
float encoder_distance_right = 0.0f;             // 右轮累计距离(mm)

// 滤波缓存数组
static int16 left_speed_buffer[ENCODER_FILTER_DEPTH] = {0};
static int16 right_speed_buffer[ENCODER_FILTER_DEPTH] = {0};
static uint8 filter_index = 0;

//================== 编码器初始化 ==================
/*==========================编码器初始化==========================*/
/* 功能：初始化左右编码器
 * 参数：无
 * 返回：无
 * 说明：配置编码器定时器和引脚
 */
void Encoder_Init(void)
{
    // 初始化左编码器
    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_A, ENCODER_LEFT_B);

    // 初始化右编码器
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_A, ENCODER_RIGHT_B);

    // 清空编码器计数
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);

    // 初始化滤波缓存
    for (uint8 i = 0; i < ENCODER_FILTER_DEPTH; i++) {
        left_speed_buffer[i] = 0;
        right_speed_buffer[i] = 0;
    }

    filter_index = 0;
}

//================== 编码器数据处理 ==================
/*==========================编码器速度限制==========================*/
/* 功能：限制编码器速度在合理范围内
 * 参数：speed - 输入速度值
 * 返回：限制后的速度值
 * 说明：防止异常大值干扰系统
 */
static int16 Encoder_Speed_Limit(int16 speed)
{
    if (speed > ENCODER_MAX_SPEED) {
        speed = ENCODER_MAX_SPEED;
    } else if (speed < -ENCODER_MAX_SPEED) {
        speed = -ENCODER_MAX_SPEED;
    }

    // 死区处理
    if (abs(speed) < ENCODER_DEAD_ZONE) {
        speed = 0;
    }

    return speed;
}

/*==========================读取编码器速度==========================*/
/* 功能：读取并处理编码器速度数据
 * 参数：无
 * 返回：无
 * 说明：获取编码器计数值，转换为速度，并清空计数器
 */
void Encoder_Read_Speed(void)
{
    int16 temp_left, temp_right;

    // 读取编码器计数值
    temp_left = -encoder_get_count(ENCODER_LEFT);    // 左编码器取反（根据安装方向调整）
    temp_right = encoder_get_count(ENCODER_RIGHT);   // 右编码器

    // 速度限制处理
    encoder_speed_left = Encoder_Speed_Limit(temp_left);
    encoder_speed_right = Encoder_Speed_Limit(temp_right);

    // 累计距离计算
    encoder_distance_left += Encoder_Speed_To_MMS(encoder_speed_left) * ENCODER_SAMPLE_TIME / 1000.0f;
    encoder_distance_right += Encoder_Speed_To_MMS(encoder_speed_right) * ENCODER_SAMPLE_TIME / 1000.0f;

    // 清空编码器计数器
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
}

/*==========================编码器滤波更新==========================*/
/* 功能：对编码器速度进行滑动平均滤波
 * 参数：无
 * 返回：无
 * 说明：使用滑动窗口平均滤波算法平滑速度数据
 */
void Encoder_Filter_Update(void)
{
    int32 left_sum = 0, right_sum = 0;

    // 更新滤波缓存
    left_speed_buffer[filter_index] = encoder_speed_left;
    right_speed_buffer[filter_index] = encoder_speed_right;

    // 计算滑动平均值
    for (uint8 i = 0; i < ENCODER_FILTER_DEPTH; i++) {
        left_sum += left_speed_buffer[i];
        right_sum += right_speed_buffer[i];
    }

    encoder_speed_left_filtered = left_sum / ENCODER_FILTER_DEPTH;
    encoder_speed_right_filtered = right_sum / ENCODER_FILTER_DEPTH;

    // 更新滤波索引
    filter_index = (filter_index + 1) % ENCODER_FILTER_DEPTH;
}

//================== 编码器数据获取 ==================
/*==========================获取左编码器速度==========================*/
int16 Encoder_Get_Left_Speed(void)
{
    return encoder_speed_left_filtered;
}

/*==========================获取右编码器速度==========================*/
int16 Encoder_Get_Right_Speed(void)
{
    return encoder_speed_right_filtered;
}

/*==========================获取左轮行驶距离==========================*/
float Encoder_Get_Left_Distance(void)
{
    return encoder_distance_left;
}

/*==========================获取右轮行驶距离==========================*/
float Encoder_Get_Right_Distance(void)
{
    return encoder_distance_right;
}

/*==========================清空距离累计==========================*/
void Encoder_Clear_Distance(void)
{
    encoder_distance_left = 0.0f;
    encoder_distance_right = 0.0f;
}

//================== 编码器数据转换 ==================
/*==========================编码器计数转换为mm/s==========================*/
/* 功能：将编码器计数值转换为线速度(mm/s)
 * 参数：encoder_count - 编码器计数值
 * 返回：对应的线速度(mm/s)
 * 说明：根据编码器分辨率和车轮直径计算实际速度
 */
float Encoder_Speed_To_MMS(int16 encoder_count)
{
    float circumference = 3.14159f * ENCODER_WHEEL_DIAMETER;  // 车轮周长(mm)
    float speed_mms = (float)encoder_count * circumference / ENCODER_RESOLUTION * (1000.0f / ENCODER_SAMPLE_TIME);
    return speed_mms;
}

//================== 调试显示 ==================
/*==========================编码器调试显示==========================*/
/* 功能：在屏幕上显示编码器相关信息
 * 参数：无
 * 返回：无
 * 说明：显示原始速度、滤波速度、累计距离等信息
 */
void Encoder_Debug_Display(void)
{
    // 显示原始编码器速度
    ips200_show_string(0, MT9V03X_H + 16 * 5, "Encoder Raw:");
    ips200_show_int(120, MT9V03X_H + 16 * 5, encoder_speed_left, 5);
    ips200_show_int(170, MT9V03X_H + 16 * 5, encoder_speed_right, 5);

    // 显示滤波后速度
    ips200_show_string(0, MT9V03X_H + 16 * 6, "Encoder Filter:");
    ips200_show_int(120, MT9V03X_H + 16 * 6, encoder_speed_left_filtered, 5);
    ips200_show_int(170, MT9V03X_H + 16 * 6, encoder_speed_right_filtered, 5);

    // 显示实际速度(mm/s)
    ips200_show_string(0, MT9V03X_H + 16 * 7, "Speed(mm/s):");
    ips200_show_float(120, MT9V03X_H + 16 * 7, Encoder_Speed_To_MMS(encoder_speed_left_filtered), 3, 1);
    ips200_show_float(170, MT9V03X_H + 16 * 7, Encoder_Speed_To_MMS(encoder_speed_right_filtered), 3, 1);

    // 显示累计距离
    ips200_show_string(0, MT9V03X_H + 16 * 8, "Distance(mm):");
    ips200_show_float(120, MT9V03X_H + 16 * 8, encoder_distance_left, 4, 1);
    ips200_show_float(170, MT9V03X_H + 16 * 8, encoder_distance_right, 4, 1);
}

//================== 兼容性函数 ==================
/*==========================兼容原有接口==========================*/
/* 以下函数保持与原代码的兼容性 */
//void encoder_filter(void)
//{
//    Encoder_Read_Speed();
//    Encoder_Filter_Update();
//}
//
//void encoder_init(void)
//{
//    Encoder_Init();
//}
