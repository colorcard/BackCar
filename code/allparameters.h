#ifndef CODE_ALLPARAMETERS_H_
#define CODE_ALLPARAMETERS_H_

//pid�������ṹ��
typedef struct{
        float Kp;
        float Ki;
        float Kd;
        //���
        float current_error;
        float last_error;
        float before_last_error;
        //�����޷������޷�
        float integrate_val;
        float integrate_max;
        float integrate_min;
        //��������
        float filter_value;
        //λ��pid�ۼӱ���
        float output_value;
        //����޷�
        float output_max;
        float output_min;
}PID;

typedef struct{
        uint8 parking_flag;                     //����������־λ
        uint8 monitor_parking_opportunity;      //��⵹��ʱ����־λ
        uint8 parking_station;                  //�����׶α�־λ
        int16 car_length;                       //С������
        int8 init_flage;                        //�ṹ���ʼ����־λ��1�����Ѿ���ʼ��
}Parking_struct;





#endif /* CODE_IMAGE_H_ */


