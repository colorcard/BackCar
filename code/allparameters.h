#ifndef CODE_ALLPARAMETERS_H_
#define CODE_ALLPARAMETERS_H_

//pid控制器结构体
typedef struct{
        float Kp;
        float Ki;
        float Kd;
        //误差
        float current_error;
        float last_error;
        float before_last_error;
        //积分限幅及其限幅
        float integrate_val;
        float integrate_max;
        float integrate_min;
        //积分死区
        float filter_value;
        //位置pid累加变量
        float output_value;
        //输出限幅
        float output_max;
        float output_min;
}PID;

typedef struct{
        uint8 parking_flag;                     //开启倒车标志位
        uint8 monitor_parking_opportunity;      //检测倒车时机标志位
        uint8 parking_station;                  //倒车阶段标志位
        int16 car_length;                       //小车长度
        int8 init_flage;                        //结构体初始化标志位，1代表已经初始化
}Parking_struct;





#endif /* CODE_IMAGE_H_ */


