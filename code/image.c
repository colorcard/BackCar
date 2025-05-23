#include "image.h"



extern float distance_to_side;
extern float angle_by_image;

uint8 road_boundary[Right_boundary-Left_boundary] = {0};//赛道边界
uint8 threshold_value;       //储存阈值信息



/*==============================进行图像大津法===============================*/
void OtsuThreshold(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8_t img_out[MT9V03X_H][MT9V03X_W]){
    int histogram[256] = {0};  // 灰度直方图
    float prob[256] = {0};     // 每个灰度级的概率
    float omega[256] = {0};    // 累积概率
    float mu[256] = {0};       // 累积均值
    float sigma[256] = {0};    // 类间方差

    // Step 1: 计算灰度直方图
    for (int y = 0; y < MT9V03X_H; y++) {
        for (int x = 0; x < MT9V03X_W; x++) {
            histogram[img_in[y][x]]++;
        }
    }

    // Step 2: 计算概率分布
    int total_pixels = MT9V03X_W * MT9V03X_H;
    for (int i = 0; i < 256; i++) {
        prob[i] = (float)histogram[i] / total_pixels;
    }

    // Step 3: 计算累积概率和均值
    omega[0] = prob[0];
    mu[0] = 0.0f;
    for (int i = 1; i < 256; i++) {
        omega[i] = omega[i - 1] + prob[i];
        mu[i] = mu[i - 1] + i * prob[i];
    }

    // Step 4: 计算类间方差，并找到最佳阈值
    float mu_total = mu[255];  // 全局均值
    float max_sigma = 0.0f;
    int best_threshold = 0;

    for (int t = 0; t < 256; t++) {
        if (omega[t] == 0 || omega[t] == 1) {
            sigma[t] = 0;
        } else {
            float w0 = omega[t];
            float w1 = 1.0f - w0;
            float mu0 = mu[t] / w0;
            float mu1 = (mu_total - mu[t]) / w1;
            sigma[t] = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);  // 类间方差公式
        }

        if (sigma[t] > max_sigma) {
            max_sigma = sigma[t];
            best_threshold = t;
        }
    }

    // Step 5: 应用阈值进行二值化并画辅助线
    for (int y = 0; y < MT9V03X_H-30; y++) {//剪除赛道外部分，节省算力
        for (int x = Left_boundary; x < Right_boundary; x++) {
            img_out[y][x] = (img_in[y][x] > best_threshold) ? 255 : 0;

        }

    }
}



/*======================线性拟合计算斜率和截距=======================*/
LineFitResult calculate_angle_and_intercept(uint8_t img_in[MT9V03X_H][MT9V03X_W]) {
    LineFitResult result = {0.0f, 0.0f, 0.0f, 0.0f};
    memset(road_boundary, 0, sizeof(road_boundary));

    // 1. 提取边界点（避免越界）
    for (int i = Left_boundary + 1; i < Right_boundary; i++) {
        for (int j = 0; j < IMAGE_HIGH - 2; j++) {
            if (img_in[j][i] != 0 && img_in[j+1][i] == 0 && img_in[j+2][i] == 0) {
                road_boundary[i - Left_boundary - 1] = (uint8_t)j;
                break;
            }
        }
    }

    // 2. 线性拟合计算斜率和截距
    float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
    uint16_t num_points = Right_boundary - Left_boundary;

    for (uint16_t i = 0; i < num_points; i++) {
        sum_x += i;
        sum_y += road_boundary[i];
        sum_xy += i * road_boundary[i];
        sum_xx += i * i;
    }

    float denominator = num_points * sum_xx - sum_x * sum_x;
    if (fabsf(denominator) < 1e-6f) {
        return result; // 避免除零，返回默认值
    }

    // 计算斜率 (m) 和截距 (b)
    result.intercept = (sum_y - result.slope * sum_x) / num_points;
    result.slope = (num_points * sum_xy - sum_x * sum_y) / denominator;


    // 4. 计算角度（弧度转角度）
    float temp = atan2f(1.0f, result.slope); // 注意：atan2(y,x) 返回的是与x轴的夹角
    result.angle = temp * 180.0f / 3.1415-90;

    //5.计算该直线到x轴的平均距离
    float right_pont_distance = 0;
    float left_pont_distance = 0;

    left_pont_distance = result.slope*(float)Left_boundary+result.intercept;
    right_pont_distance = result.slope*(float)Right_boundary+result.intercept;
    result.level_distance = (left_pont_distance+right_pont_distance)/2;

    //工程问题，前期先将信息储存在全局变量中
    distance_to_side =  result.level_distance;
    angle_by_image = result.angle;
    // 3. 显示结果（可选）
    ips200_show_string(0*16,MT9V03X_H + 16 * 1,"Solpe:");
    ips200_show_string(0*16,MT9V03X_H + 16 * 2,"Intercept:");
    ips200_show_string(0*16,MT9V03X_H + 16 * 3,"Angle:");
    ips200_show_string(0*16,MT9V03X_H + 16 * 4,"Diastance:");

    ips200_show_float(5 * 16, MT9V03X_H + 16 * 1, result.slope, 3, 2);
    ips200_show_float(5 * 16, MT9V03X_H + 16 * 2, result.intercept, 3, 2);
    ips200_show_float(5 * 16, MT9V03X_H + 16 * 3, result.angle, 3, 2);
    ips200_show_float(5 * 16, MT9V03X_H + 16 * 4, result.level_distance, 3, 2);

    return result;
}


float get_distance(uint8_t img_in[MT9V03X_H][MT9V03X_W]){
    float temp = 0;
    temp = calculate_angle_and_intercept(img_in).level_distance;

    return temp;
}//获取当前距离

float get_image_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W]){
    float temp = 0;
    temp = calculate_angle_and_intercept(img_in).angle;

    return temp;
}//获取当前角度


//倒车入库时机识别函数
//画辅助线
//void draw_subline(uint8_t img_in[MT9V03X_H][MT9V03X_W]){
//    for(int i = 0;i<MT9V03X_H;i++){
//        img_in[i][(int)(MT9V03X_W-(3.0/7.0)*MT9V03X_W)+10] = 1;
//    }
//
//}

//初始化车库结构体
void parking_struct_init(void){
    parking.car_length = 3120;    //3120次编码器脉冲
    parking.parking_flag = 0;
    parking.parking_station = 0;
    parking.monitor_parking_opportunity = 0;
    parking.parking_station = 0;
}

//扫描突变点个数
uint8 parking_condition_scan(uint8_t img_in[MT9V03X_H][MT9V03X_W],uint8 start_line,uint8 range){
    uint8 number = 0;
    for(int i = Left_boundary+1;i<Right_boundary-2;i++){
        for(int j = start_line; j<start_line+range;i++){
            if(img_in[j][i+1]!=img_in[j][i]
                   &&img_in[j][i] == img_in[j][i-1]
                   &&img_in[j][i+1]==img_in[j][i+2]){
                number++;
            }

        }
    }
    return number;
}

//葡萄串法扫描获取车库状态信息
uint8 grape_seed[5] = {
        Left_boundary+5,
        (uint8)((Left_boundary+Right_boundary)*0.25),
        (uint8)((Left_boundary+Right_boundary)*0.5),
        (uint8)((Left_boundary+Right_boundary)*0.75),
        Right_boundary-5
};

uint8 grape_broom_monitor_parking(uint8_t img_in[MT9V03X_H][MT9V03X_W],uint8 start_line,uint8 range){
    uint8 max_connect_number = 0;
    uint8 connect_number[5] = {0};

    //画辅助线
    for(int i = 0;i<5;i++){
        ips200_draw_line(grape_seed[i],0,grape_seed[i],MT9V03X_H,RGB565_RED);
    }
    ips200_draw_line(Left_boundary,start_line,Right_boundary,start_line,RGB565_RED);
    ips200_draw_line(Left_boundary,start_line+range,Right_boundary,start_line+range,RGB565_RED);

    //如果扫描范围超出图像高度，则返回255，表示错误
    if(start_line+range>MT9V03X_H)return 255;

    for(int i = start_line;i<range+start_line;i++){//从第i+1行出发
        //统计数组初始化
        for(int j = 0;j<5;j++){
            connect_number[j] = 0;
        }

        // 先扫描整行，标记所有种子点的连通情况
        uint8 seed_connected[5] = {0}; // 标记种子点是否被连通

        // 从左到右扫描
        uint8 in_region = 0; // 是否在连通区域内
        uint8 connected_seeds = 0; // 当前连通区域包含的种子点数量
        for(int k = Left_boundary; k < Right_boundary; k++){
            if(img_in[k][i] != 0){ // 如果是有效像素
                if(!in_region){
                    in_region = 1;
                    connected_seeds = 0;
                }
                // 检查是否是种子点
                for(int j = 0; j < 5; j++){
                    if(k == grape_seed[j]){
                        seed_connected[j] = 1;
                        connected_seeds++;
                    }
                }
            }
            else{ // 遇到边界
                if(in_region){
                    in_region = 0;
                    // 更新所有被连通的种子点的计数
                    for(int j = 0; j < 5; j++){
                        if(seed_connected[j]){
                            connect_number[j] = connected_seeds;
                            seed_connected[j] = 0; // 重置标记
                        }
                    }
                }
            }
        }

        // 处理最后一个区域
        if(in_region){
            for(int j = 0; j < 5; j++){
                if(seed_connected[j]){
                    connect_number[j] = connected_seeds;
                }
            }
        }

        //获取该行最大连通数
        for(int j = 0;j<5;j++){
            if(connect_number[j]>max_connect_number){
                max_connect_number = connect_number[j];
            }
        }
    }
    //返回最大连通数
    return max_connect_number;
}
