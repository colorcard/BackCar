#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

//================== 宏定义区 ==================
#define IPS200_TYPE             (IPS200_TYPE_SPI)      // IPS200使用SPI驱动
#define PID_TIMER_PERIOD        (10)                   // PID控制周期 10ms
#define SERVO_DEAD_ZONE         (5)                    // 舵机死区范围
#define DISTANCE_REFERENCE      (29)                   // 距离基准值
#define PARKING_DETECT_OFFSET   (20)                   // 停车检测偏移量
#define PARKING_DETECT_THRESHOLD (3)                   // 停车检测阈值

//================== 图像处理参数 ==================
#define GYRO_WEIGHT            (0.5f)                  // 陀螺仪权重
#define DISTANCE_WEIGHT        (1.3f)                  // 距离权重
#define IMAGE_ANGLE_WEIGHT     (0.5f)                  // 图像角度权重

//================== 全局变量区 ==================
// 图像数组
uint8 original_image[MT9V03X_H][MT9V03X_W];          // 原始图像数组
uint8 processed_image[MT9V03X_H][MT9V03X_W];         // 大津法处理后的图像数组

// 系统状态变量
uint64 system_millis = 0;                            // 系统运行时间(ms)
uint64 millis = 0; // 定时器计数器
//float distance_to_line = 0.0f;                       // 图像与赛道的距离
float current_steering_angle = 0.0f;                 // 当前舵机输出角度
//float distance_to_side = 0.0f;                       // 侧向距离
//float angle_by_image = 0.0f;                         // 图像计算得到的角度

// 停车系统
Parking_struct parking;                       // 停车控制结构体

// 外部变量声明 - 陀螺仪角度
extern float X_now_angle_to_path;                    // 陀螺仪角度（pitch）
extern float Y_now_angle_to_path;                    // 陀螺仪角度（roll）
extern float Z_now_angle_to_path;                    // 陀螺仪角度（yaw）

//================== 函数声明区 ==================
static void Parking_System_Init(void);
static void Image_Process(void);
static float Calculate_Steering_Angle(void);
static uint8 Parking_Detection(void);
static void System_Info_Display(uint8 seed_connect_number);
static void Gyroscope_Display(void);
static void Encoder_Display(void);

//================== 主函数 ==================
/*==========================核心0主函数==========================*/
int core0_main(void)
{
    //================== 初始化 ==================
    // 基础系统初始化
    clock_init();
    debug_init();

    // 显示器初始化
    ips200_init(IPS200_TYPE);
    ips200_clear();

    // 传感器初始化
    mt9v03x_init();                                   // 摄像头初始化
    mpu6050_init();                                   // 陀螺仪初始化
    Encoder_Init();                                   // 编码器初始化

    // 执行器初始化
    Servo_Init(80, -80, 100, -100, 5, 2, 0.5, 0.5); // 舵机初始化

//    // 应用模块初始化
//    parking_struct_init();                           // 停车系统初始化

    // 定时器初始化
    pit_ms_init(CCU60_CH0, PID_TIMER_PERIOD);       // PID控制器中断
    Parking_System_Init();

    // 等待系统就绪
    cpu_wait_event_ready();

    //================== 主循环 ==================
    while (TRUE)
    {
        //================== 传感器数据处理 ==================
        // 图像采集与处理
        Image_Process();

        // 停车位检测
        uint8 seed_connect_number = Parking_Detection();

        //================== 控制算法执行 ==================
        // 计算融合控制角度
        current_steering_angle = Calculate_Steering_Angle();

        // 舵机角度控制
        Servo_Onto_Control(current_steering_angle);

        //================== 显示信息更新 ==================
        // 显示系统主要信息
        System_Info_Display(seed_connect_number);

        // 显示陀螺仪数据
        Gyroscope_Display();

        // 显示编码器数据
        Encoder_Display();

        //================== 系统状态更新 ==================
        system_millis++;  // 系统时间递增（如果需要精确计时，建议使用定时器）
    }
}

//================== 中断服务函数 ==================
/*==========================定时器中断服务函数==========================*/
/* 功能：PID控制器定时中断处理
 * 参数：无
 * 返回：无
 * 说明：每10ms执行一次，用于编码器数据采集和控制更新
 */
IFX_INTERRUPT(cc60_pit_ch0_isr, 1, CCU6_0_CH0_ISR_PRIORITY)
{
    // 开启中断嵌套
    interrupt_global_enable(0);

    // 清除中断标志
    pit_clear_flag(CCU60_CH0);

    //================== 编码器数据处理 ==================
    // 读取编码器速度
    Encoder_Read_Speed();

    // 更新滤波数据
    Encoder_Filter_Update();

    //================== 控制算法更新 ==================
    // 这里可以添加其他需要定时执行的控制算法
    // 例如：速度PID控制、位置控制等

    //================== 系统状态更新 ==================
    system_millis++;  // 精确的系统时间计数
}

/*==========================停车系统初始化==========================*/
/* 功能：初始化停车系统参数
 * 参数：无
 * 返回：无
 */
static void Parking_System_Init(void)
{
    parking.car_length = 3120;                // 车长对应的编码器脉冲数
    parking.parking_flag = 0;                 // 停车标志位
    parking.parking_station = 0;              // 停车阶段标志位
    parking.monitor_parking_opportunity = 0;   // 检测停车时机标志位
}

//================== 图像处理函数 ==================
/*==========================图像采集与处理==========================*/
/* 功能：采集并处理摄像头图像
 * 参数：无
 * 返回：无
 */
static void Image_Process(void)
{
    // 复制原始图像
    memcpy(original_image, mt9v03x_image, MT9V03X_IMAGE_SIZE * sizeof(uint8));

    // 大津法二值化处理
    OtsuThreshold(original_image, processed_image);

    // 计算图像角度和截距
    calculate_angle_and_intercept(processed_image);
}

//================== 控制算法函数 ==================
/*==========================角度融合计算==========================*/
/* 功能：融合多传感器数据计算最终控制角度
 * 参数：无
 * 返回：融合后的控制角度
 */
static float Calculate_Steering_Angle(void)
{
    float steering_angle;

    // 多传感器融合：陀螺仪 + 距离偏差 + 图像角度
    steering_angle = GYRO_WEIGHT * Z_now_angle_to_path +
                     DISTANCE_WEIGHT * (distance_to_side - DISTANCE_REFERENCE) +
                     IMAGE_ANGLE_WEIGHT * angle_by_image;

    // 死区处理
    if (steering_angle >= -SERVO_DEAD_ZONE && steering_angle <= SERVO_DEAD_ZONE) {
        steering_angle = 0.0f;
    }

    return steering_angle;
}

/*==========================停车检测==========================*/
/* 功能：检测停车位并返回连通域数量
 * 参数：无
 * 返回：种子连通域数量
 */
static uint8 Parking_Detection(void)
{
    return grape_broom_monitor_parking(processed_image,
                                       distance_to_side + PARKING_DETECT_OFFSET,
                                       PARKING_DETECT_THRESHOLD);
}

//================== 显示相关函数 ==================
/*==========================系统信息显示==========================*/
/* 功能：在屏幕上显示系统运行信息
 * 参数：seed_connect_number - 连通域数量
 * 返回：无
 */
static void System_Info_Display(uint8 seed_connect_number)
{
    // 显示处理后的图像
    ips200_displayimage03x(processed_image[0], MT9V03X_W, MT9V03X_H);

    // 显示系统运行时间
    ips200_show_string(0, MT9V03X_H + 16 * 0, "System Time:");
    ips200_show_uint(96, MT9V03X_H + 16 * 0, (uint32)system_millis, 7);

    // 显示连通域数量
    ips200_show_string(0, MT9V03X_H + 16 * 1, "Connect Num:");
    ips200_show_int(96, MT9V03X_H + 16 * 1, seed_connect_number, 2);

    // 显示当前控制角度
    ips200_show_string(0, MT9V03X_H + 16 * 2, "Steer Angle:");
    ips200_show_float(96, MT9V03X_H + 16 * 2, current_steering_angle, 5, 2);

    // 显示距离信息
    ips200_show_string(0, MT9V03X_H + 16 * 3, "Side Dist:");
    ips200_show_float(80, MT9V03X_H + 16 * 3, distance_to_side, 4, 1);
}

/*==========================陀螺仪数据显示==========================*/
/* 功能：显示陀螺仪姿态角信息
 * 参数：无
 * 返回：无
 */
static void Gyroscope_Display(void)
{
    // 显示三轴姿态角
    ips200_show_string(0, MT9V03X_H + 16 * 5, "Pitch:");
    ips200_show_float(48, MT9V03X_H + 16 * 5, X_now_angle_to_path, 6, 2);

    ips200_show_string(0, MT9V03X_H + 16 * 6, "Roll :");
    ips200_show_float(48, MT9V03X_H + 16 * 6, Y_now_angle_to_path, 6, 2);

    ips200_show_string(0, MT9V03X_H + 16 * 7, "Yaw  :");
    ips200_show_float(48, MT9V03X_H + 16 * 7, Z_now_angle_to_path, 6, 2);
}

/*==========================编码器数据显示==========================*/
/* 功能：显示编码器速度信息
 * 参数：无
 * 返回：无
 */
static void Encoder_Display(void)
{
    ips200_show_string(0, MT9V03X_H + 16 * 8, "L_Speed:");
    ips200_show_int(64, MT9V03X_H + 16 * 8, Encoder_Get_Left_Speed(), 5);

    ips200_show_string(0, MT9V03X_H + 16 * 9, "R_Speed:");
    ips200_show_int(64, MT9V03X_H + 16 * 9, Encoder_Get_Right_Speed(), 5);
}

#pragma section all restore
