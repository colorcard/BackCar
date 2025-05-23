//#include "stdio.h"
//#include "stdint.h"
//#include "stdbool.h"
//#include "string.h"
#include "pid.h"
//#include "math.h"
//#include "zf_common_font.h"


PID SE_PID;

//PID��������ʼ��
void Servo_PID_Init(float Integrate_max
        ,float Integrate_min
        ,float Output_max
        ,float Output_min
        ,float Filter_value
        ,float kp
        ,float ki
        ,float kd){
    SE_PID.Kp = kp;
    SE_PID.Ki = ki;
    SE_PID.Kd = kd;
    SE_PID.before_last_error = 0;
    SE_PID.current_error = 0;
    SE_PID.filter_value = Filter_value;
    SE_PID.integrate_max = Integrate_max;
    SE_PID.integrate_min = Integrate_min;
    SE_PID.integrate_val = 0;
    SE_PID.last_error = 0;
    SE_PID.output_max = Output_max;
    SE_PID.output_min = Output_min;
    SE_PID.output_value = 0;


}

float Servo_PID_Control(float current_onto){
    //���ݸ���
    SE_PID.before_last_error = SE_PID.last_error;
    SE_PID.last_error = SE_PID.current_error;
    SE_PID.current_error = current_onto;
    SE_PID.integrate_val+= SE_PID.current_error;

    //��������
    if(SE_PID.current_error>-SE_PID.filter_value&&SE_PID.current_error<SE_PID.filter_value){
        SE_PID.integrate_val = 0;
    }

    //�����޷�
    if(SE_PID.integrate_val>=SE_PID.integrate_max){
        SE_PID.integrate_val = SE_PID.integrate_max;
    }
    if(SE_PID.integrate_val<=SE_PID.integrate_min){
        SE_PID.integrate_val = SE_PID.integrate_min;
    }
    //�������
    SE_PID.output_value = SE_PID.Kp*SE_PID.current_error
            +SE_PID.Ki*SE_PID.integrate_val
            +SE_PID.Kd*(SE_PID.before_last_error+SE_PID.current_error-2*SE_PID.last_error);
    //����޷�
    if(SE_PID.output_value>=SE_PID.output_max){
        SE_PID.output_value = SE_PID.output_max;
    }
    if(SE_PID.output_value<=SE_PID.output_min){
        SE_PID.output_value = SE_PID.output_min;
    }
    return SE_PID.output_value;
}
