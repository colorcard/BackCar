/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************
//串口屏显示函数一律在此核中处理


// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

/********陀螺仪解算相关结构体函数定义(start)*******/
typedef struct SensorMsg {
    float A[3];
    float G[3];
} SensorMsg;

typedef struct MPU6050Params {
    uint16_t MPU6050dt;
    uint64_t preMillis;
    float MPU6050ERROE[6];


} MPU6050Params;

MPU6050Params mpu6050 = {
        .MPU6050dt = 10,
        .preMillis = 0,
        .MPU6050ERROE = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

SensorMsg msg = {
        .A = {0.0f, 0.0f, 0.0f},
        .G = {0.0f, 0.0f, 0.0f}
};

extern uint64 millis;

float X_now_angle = 0;
float Y_now_angle = 0;
float Z_now_angle = 0;

//备份数据，避免两核之间资源访问冲突
float X_now_angle_to_path = 0;
float Y_now_angle_to_path = 0;
float Z_now_angle_to_path = 0;

//相关函数定义
void dataGetERROR(void);
void getMPU6050Data(void);
void dataGetAndFilter(void);
/*******陀螺仪解算相关结构体函数定义(end)*******/


void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    begin(1000.0f / (float)mpu6050.MPU6050dt);
    system_delay_ms(100);
    dataGetERROR();
    system_delay_ms(100);

    pit_ms_init(CCU60_CH1, 1); //滴答计时器


    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        mpu6050_get_acc();
        mpu6050_get_gyro();
        system_delay_ms(10);

        if(millis - mpu6050.preMillis >= mpu6050.MPU6050dt) {
              mpu6050.preMillis = millis;
              dataGetAndFilter();
              updateIMU(msg.G[0], msg.G[1], msg.G[2], msg.A[0], msg.A[1], msg.A[2]);
          }

        X_now_angle = getPitch();
        Y_now_angle = getRoll();
        Z_now_angle = getYaw();

        //拷贝数据
        X_now_angle_to_path = X_now_angle;
        Y_now_angle_to_path = Y_now_angle;
        Z_now_angle_to_path = Z_now_angle;

        // 此处编写需要循环执行的代码
    }
}


//MPU6050获取稳态误差
void dataGetERROR(void) {
    for(uint8_t i = 0; i < 50; ++i) {
        getMPU6050Data();
        mpu6050.MPU6050ERROE[0] += msg.A[0];
        mpu6050.MPU6050ERROE[1] += msg.A[1];
        mpu6050.MPU6050ERROE[2] += msg.A[2] - 9.7915;
        mpu6050.MPU6050ERROE[3] += msg.G[0];
        mpu6050.MPU6050ERROE[4] += msg.G[1];
        mpu6050.MPU6050ERROE[5] += msg.G[2];
        system_delay_ms(20);
    }
    for(uint8_t i = 0; i < 6; ++i) {
        mpu6050.MPU6050ERROE[i] /= 50.0f;
    }
}

void getMPU6050Data(void) {
    msg.A[0] = (float)((float)mpu6050_acc_x / (float)32768) * 16 * 9.80*0.2*(90/56)*(180.0/119.0);
    msg.A[1] = (float)((float)mpu6050_acc_y / (float)32768) * 16 * 9.8*0.2*(90/56)*(180.0/119.0);
    msg.A[2] = (float)((float)mpu6050_acc_z / (float)32768) * 16 * 9.8*0.2*(90/56)*(180.0/119.0);
    msg.G[0] = (float)((float)mpu6050_gyro_x / (float)32768) * 2000 * 3.5*0.2*(90/56)*(180.0/119.0);
    msg.G[1] = (float)((float)mpu6050_gyro_y / (float)32768) * 2000 * 3.5*0.2*(90/56)*(180.0/119.0);
    msg.G[2] = (float)((float)mpu6050_gyro_z / (float)32768) * 2000 * 3.5*0.2*(90/56)*(180.0/119.0);

}

void dataGetAndFilter(void) {
    getMPU6050Data();
    msg.A[0] -= mpu6050.MPU6050ERROE[0];
    msg.A[1] -= mpu6050.MPU6050ERROE[1];
    msg.A[2] -= mpu6050.MPU6050ERROE[2];
    msg.G[0] -= mpu6050.MPU6050ERROE[3];
    msg.G[1] -= mpu6050.MPU6050ERROE[4];
    msg.G[2] -= mpu6050.MPU6050ERROE[5];
}

//滴答定时器
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);
    millis++;


}







#pragma section all restore
// **************************** 代码区域 ****************************
