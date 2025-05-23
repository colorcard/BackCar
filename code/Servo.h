#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"


#define  pwm_servo                 (ATOM0_CH1_P33_9) //宏定义舵机信号线

#define  mid_value  620

void Servo_Init(float Integrate_max
        ,float Integrate_min
        ,float Output_max
        ,float Output_min
        ,float Filter_value
        ,float kp
        ,float ki
        ,float kd);

void Servo_Onto_Control(float current_onto);


#endif /* CODE_SERVO_H_ */
