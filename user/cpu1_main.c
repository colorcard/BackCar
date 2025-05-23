/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu1_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.10.2
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

// **************************** �������� ****************************
//��������ʾ����һ���ڴ˺��д���


// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

/********�����ǽ�����ؽṹ�庯������(start)*******/
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

//�������ݣ���������֮����Դ���ʳ�ͻ
float X_now_angle_to_path = 0;
float Y_now_angle_to_path = 0;
float Z_now_angle_to_path = 0;

//��غ�������
void dataGetERROR(void);
void getMPU6050Data(void);
void dataGetAndFilter(void);
/*******�����ǽ�����ؽṹ�庯������(end)*******/


void core1_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�
    // �˴���д�û����� ���������ʼ�������

    begin(1000.0f / (float)mpu6050.MPU6050dt);
    system_delay_ms(100);
    dataGetERROR();
    system_delay_ms(100);

    pit_ms_init(CCU60_CH1, 1); //�δ��ʱ��


    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
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

        //��������
        X_now_angle_to_path = X_now_angle;
        Y_now_angle_to_path = Y_now_angle;
        Z_now_angle_to_path = Z_now_angle;

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}


//MPU6050��ȡ��̬���
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

//�δ�ʱ��
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH1);
    millis++;


}







#pragma section all restore
// **************************** �������� ****************************
