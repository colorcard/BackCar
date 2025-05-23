#include "zf_common_headfile.h"


#pragma section all "cpu0_dsram"

/*=============== 宏定义区 ===============*/
#define IPS200_TYPE     (IPS200_TYPE_SPI)   //  IPS200使用SPI驱动

/*=============== 变量区 ================*/
uint8 original_image[MT9V03X_H][MT9V03X_W];     //转载图像数组
uint8 image[MT9V03X_H][MT9V03X_W];  //  大津法处理后的图像数组

uint64 millis = 0;
float distance_to_line;     //图像和赛道的距离
float current_angle = 0;    //图像和赛道的角度（舵机输出角度）
float distance_to_side;
float angle_by_image;

Parking_struct parking;

//设置倒车结构体
parking.car_length = 3120;    //3120次编码器脉冲
parking.parking_flag = 0;   //开启倒车标志位
parking.parking_station = 0;    //倒车阶段标志位
parking.monitor_parking_opportunity = 0;    //检测倒车时机标志位


extern float X_now_angle_to_path;   //  陀螺仪角度（pitch）
extern float Y_now_angle_to_path;   //  陀螺仪角度（roll）
extern float Z_now_angle_to_path;   //  陀螺仪角度（yaw）


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

    parking_struct_init(); //初始化车库结构体



    pit_ms_init(CCU60_CH0, 100); //PID控制器中断，时间周期为10Hz

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();
    while (TRUE)
    {
        /*  工程结构没建立好，前期能用就行 */

        //  转载图像
        memcpy(original_image,mt9v03x_image,MT9V03X_IMAGE_SIZE*sizeof(uint8));

        //  大津法处理后的图像
        OtsuThreshold(original_image,image);

        //获取的角度 = 陀螺仪角度+图像计算角度+（距离-基准距离）*k构成，后期可加入惩罚机制
        calculate_angle_and_intercept(image);

        //检查种子连通个数
        uint8 seed_connect_number = grape_broom_monitor_parking(image,distance_to_side+20,3);

        //获取当前图像和赛道的角度
        current_angle = 0.5*Z_now_angle_to_path+1.3*(distance_to_side-29)+0.5*angle_by_image;
        if(current_angle>=-5&&current_angle<=5){current_angle = 0;} //设置死区

        //舵机控制角度
        Servo_Onto_Control(current_angle);





        /*=============显示区域============*/
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

