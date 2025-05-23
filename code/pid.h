#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "zf_common_headfile.h"

void Servo_PID_Init(float Integrate_max,float Integrate_min,float Output_max,float Output_min,float Filter_value,float kp,float ki,float kd);
float Servo_PID_Control(float current_onto);

#endif /* CODE_MOTOR_H_ */
