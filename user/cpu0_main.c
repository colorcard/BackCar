#include "zf_common_headfile.h"


#pragma section all "cpu0_dsram"


#define IPS200_TYPE     (IPS200_TYPE_SPI)
uint8 original_image[MT9V03X_H][MT9V03X_W];     //转载图像数组
uint8 image[MT9V03X_H][MT9V03X_W];
/***********变量区**************/
uint64 millis = 0;
float distance_to_line;
float current_angle = 0;
float distance_to_side;
float angle_by_image;

Parking_struct parking;


extern float X_now_angle_to_path;
extern float Y_now_angle_to_path;
extern float Z_now_angle_to_path;


int core0_main(void)
{
    clock_init();
    debug_init();
    // 此处编写用户代码 例如外设初始化代码等
    //初始化部分
    ips200_init(IPS200_TYPE);
    ips200_clear();
    mt9v03x_init();
    mpu6050_init();
    encoder_init();

    //初始化舵机与PID控制器
    Servo_Init(80,-80,100,-100,5,2,0.5,0.5);
    //设置倒车结构体
    parking.car_length = 3120;    //3120次编码器脉冲
    parking.parking_flag = 0;
    parking.parking_station = 0;
    parking.monitor_parking_opportunity = 0;


//    ws_motor_init();
//    PWM_motor(1000,-1000);

    pit_ms_init(CCU60_CH0, 100); //PID控制器中断，时间周期为10Hz

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();
    while (TRUE)
    {
        //工程结构没建立好，前期能用就行
        //二值化图像
        memcpy(original_image,mt9v03x_image,MT9V03X_IMAGE_SIZE*sizeof(uint8));
        OtsuThreshold(original_image,image);
        calculate_angle_and_intercept(image);
//        //获取的角度 = 陀螺仪角度+图像计算角度+（距离-基准距离）*k构成，后期可加入惩罚机制
        current_angle = 0.5*Z_now_angle_to_path+1.3*(distance_to_side-29)+0.5*angle_by_image;

//        current_angle = 1*Z_now_angle_to_path;
        Servo_Onto_Control(current_angle);
        //检查种子连通个数
        uint8 seed_connect_number = grape_broom_monitor_parking(image,distance_to_side+20,3);


        //设置死区
        if(current_angle>=-5&&current_angle<=5){current_angle = 0;}

        //显示区域
        ips200_displayimage03x(image[0], MT9V03X_W, MT9V03X_H);

        ips200_show_string(0*16,MT9V03X_H + 16 * 5,"connect_number:");
        ips200_show_int(9 * 16, MT9V03X_H + 16 * 5, seed_connect_number,2);


        //显示陀螺仪解算结果
        ips200_show_string(0*16,MT9V03X_H + 16 * 7,"Pitch:");
        ips200_show_string(0*16,MT9V03X_H + 16 * 8,"Roll:");
        ips200_show_string(0*16,MT9V03X_H + 16 * 9,"Yaw:");

        ips200_show_float(3 * 16, MT9V03X_H + 16 * 7, X_now_angle_to_path, 3, 2);
        ips200_show_float(3 * 16, MT9V03X_H + 16 * 8, Y_now_angle_to_path, 3, 2);
        ips200_show_float(3 * 16, MT9V03X_H + 16 * 9, Z_now_angle_to_path, 3, 2);

        //显示滴答计时器数值
        ips200_show_string(0*16,MT9V03X_H + 16 * 0,"time:");
        ips200_show_uint(3*16,MT9V03X_H + 16 * 0,(uint32)millis,7);

        ips200_show_string(0*16,MT9V03X_H + 16 * 10,"Current_Angle:");
        ips200_show_float(8 * 16, MT9V03X_H + 16 * 10, current_angle, 5, 2);



    }

}



////滴答定时器
//IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
//{
//    interrupt_global_enable(0);                     // 开启中断嵌套
//    pit_clear_flag(CCU60_CH1);
//    millis++;
//
//}

IFX_INTERRUPT(cc60_pit_ch0_isr, 1, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    //获取编码器数值
    encoder_filter();
    //控制小车定距离巡线
//    Servo_Onto_Control(current_angle);
}



#pragma section all restore

