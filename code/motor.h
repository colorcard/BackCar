#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

//µç»úÒý½Å
#define DIR_R              (P21_4)
#define PWM_R              (ATOM1_CH0_P21_2)

#define DIR_L              (P21_5)
#define PWM_L              (ATOM1_CH1_P21_3)

void PWM_motor(float motor_1,float motor_2);
void ws_motor_init(void);

#endif /* CODE_MOTOR_H_ */
