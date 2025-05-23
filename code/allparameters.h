#ifndef CODE_ALLPARAMETERS_H_
#define CODE_ALLPARAMETERS_H_


typedef struct{
        uint8 parking_flag;                     //开启倒车标志位
        uint8 monitor_parking_opportunity;      //检测倒车时机标志位
        uint8 parking_station;                  //倒车阶段标志位
        int16 car_length;                       //小车长度
        int8 init_flage;                        //结构体初始化标志位，1代表已经初始化
}Parking_struct;





#endif /* CODE_ALLPARAMETERS_H_ */


