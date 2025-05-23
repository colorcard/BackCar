#include "Servo.h"
//#include "pid.h"

//¶æ»ú·¶Î§520~720£¬ÖÐÖµ£º620

void Servo_Init(float Integrate_max
        ,float Integrate_min
        ,float Output_max
        ,float Output_min
        ,float Filter_value
        ,float kp
        ,float ki
        ,float kd){
    pwm_init(pwm_servo,50,mid_value);
    Servo_PID_Init(Integrate_max
            ,Integrate_min
            ,Output_max
            ,Output_min
            ,Filter_value
            ,kp
            ,ki
            ,kd);
}

void Servo_Onto_Control(float current_onto){
    int16 onto;
    onto = -(int)(Servo_PID_Control(current_onto))+620;
    pwm_set_duty(pwm_servo,onto);
    ips200_show_float(0 * 16, MT9V03X_H + 16 * 11, onto-620, 3, 2);

}




