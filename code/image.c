#include "image.h"

//================== 全局变量定义 ==================
float distance_to_side = 0.0f;
float angle_by_image = 0.0f;
uint8 threshold_value = 0;

// 内部变量
static uint8 road_boundary[RIGHT_BOUNDARY - LEFT_BOUNDARY] = {0};  // 赛道边界

// 葡萄串法种子点位置
static const uint8 grape_seed[5] = {
        LEFT_BOUNDARY + 5,
        (uint8)((LEFT_BOUNDARY + RIGHT_BOUNDARY) * 0.25f),
        (uint8)((LEFT_BOUNDARY + RIGHT_BOUNDARY) * 0.5f),
        (uint8)((LEFT_BOUNDARY + RIGHT_BOUNDARY) * 0.75f),
        RIGHT_BOUNDARY - 5
};

//================== 内部函数声明 ==================
static void display_calculation_results(const LineFitResult* result);
static void draw_parking_assist_lines(uint8 start_line, uint8 range);

//================== 图像处理函数 ==================
/*==============================进行图像大津法===============================*/
void OtsuThreshold(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8_t img_out[MT9V03X_H][MT9V03X_W])
{
    int histogram[256] = {0};    // 灰度直方图
    float prob[256] = {0};       // 每个灰度级的概率
    float omega[256] = {0};      // 累积概率
    float mu[256] = {0};         // 累积均值
    float sigma[256] = {0};      // 类间方差

    // Step 1: 计算灰度直方图
    for (int y = 0; y < MT9V03X_H; y++) {
        for (int x = 0; x < MT9V03X_W; x++) {
            histogram[img_in[y][x]]++;
        }
    }

    // Step 2: 计算概率分布
    const int total_pixels = MT9V03X_W * MT9V03X_H;
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
    const float mu_total = mu[255];  // 全局均值
    float max_sigma = 0.0f;
    int best_threshold = 0;

    for (int t = 0; t < 256; t++) {
        if (omega[t] == 0.0f || omega[t] == 1.0f) {
            sigma[t] = 0.0f;
        } else {
            const float w0 = omega[t];
            const float w1 = 1.0f - w0;
            const float mu0 = mu[t] / w0;
            const float mu1 = (mu_total - mu[t]) / w1;
            const float diff = mu0 - mu1;
            sigma[t] = w0 * w1 * diff * diff;  // 类间方差公式
        }

        if (sigma[t] > max_sigma) {
            max_sigma = sigma[t];
            best_threshold = t;
        }
    }

    // 存储阈值到全局变量
    threshold_value = (uint8)best_threshold;

    // Step 5: 应用阈值进行二值化
    const int process_height = MT9V03X_H - 30;  // 剪除赛道外部分，节省算力
    for (int y = 0; y < process_height; y++) {
        for (int x = LEFT_BOUNDARY; x < RIGHT_BOUNDARY; x++) {
            img_out[y][x] = (img_in[y][x] > best_threshold) ? WHITE_PIXEL : BLACK_PIXEL;
        }
    }
}

//================== 角度和距离计算函数 ==================
/*======================线性拟合计算斜率和截距=======================*/
LineFitResult calculate_angle_and_intercept(uint8_t img_in[MT9V03X_H][MT9V03X_W])
{
    LineFitResult result = {0.0f, 0.0f, 0.0f, 0.0f};
    memset(road_boundary, 0, sizeof(road_boundary));

    // 1. 提取边界点（避免越界）
    const int search_height = IMAGE_HIGH - 2;
    for (int i = LEFT_BOUNDARY + 1; i < RIGHT_BOUNDARY; i++) {
        for (int j = 0; j < search_height; j++) {
            if (img_in[j][i] != BLACK_PIXEL &&
                img_in[j + 1][i] == BLACK_PIXEL &&
                img_in[j + 2][i] == BLACK_PIXEL) {
                road_boundary[i - LEFT_BOUNDARY - 1] = (uint8_t)j;
                break;
            }
        }
    }

    // 2. 线性拟合计算斜率和截距
    float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
    const uint16_t num_points = RIGHT_BOUNDARY - LEFT_BOUNDARY;

    for (uint16_t i = 0; i < num_points; i++) {
        const float x_val = (float)i;
        const float y_val = (float)road_boundary[i];

        sum_x += x_val;
        sum_y += y_val;
        sum_xy += x_val * y_val;
        sum_xx += x_val * x_val;
    }

    const float denominator = num_points * sum_xx - sum_x * sum_x;
    if (fabsf(denominator) < 1e-6f) {
        return result; // 避免除零，返回默认值
    }

    // 计算斜率 (m) 和截距 (b)
    result.slope = (num_points * sum_xy - sum_x * sum_y) / denominator;
    result.intercept = (sum_y - result.slope * sum_x) / num_points;

    // 3. 计算角度（弧度转角度）
    const float angle_rad = atan2f(1.0f, result.slope);
    result.angle = angle_rad * 180.0f / 3.14159265f - 90.0f;  // 使用更精确的PI值

    // 4. 计算该直线到x轴的平均距离
    const float left_point_distance = result.slope * (float)LEFT_BOUNDARY + result.intercept;
    const float right_point_distance = result.slope * (float)RIGHT_BOUNDARY + result.intercept;
    result.level_distance = (left_point_distance + right_point_distance) * 0.5f;

    // 更新全局变量
    distance_to_side = result.level_distance;
    angle_by_image = result.angle;

    // 5. 显示结果
    display_calculation_results(&result);

    return result;
}

float get_distance(uint8_t img_in[MT9V03X_H][MT9V03X_W])
{
    (void)img_in; // 避免未使用参数警告
    return distance_to_side;  // 直接返回全局变量，避免重复计算
}

float get_image_angle(uint8_t img_in[MT9V03X_H][MT9V03X_W])
{
    (void)img_in; // 避免未使用参数警告
    return angle_by_image;  // 直接返回全局变量，避免重复计算
}

//================== 停车相关函数 ==================
void parking_struct_init(void)
{
    parking.car_length = 3120;                      // 3120次编码器脉冲
    parking.parking_flag = 0;
    parking.parking_station = 0;
    parking.monitor_parking_opportunity = 0;
}

uint8 parking_condition_scan(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8 start_line, uint8 range)
{
    // 边界检查
    if (start_line + range >= MT9V03X_H || start_line >= MT9V03X_H) {
        return 0;  // 返回0而不是错误代码
    }

    uint8 number = 0;
    const int end_x = RIGHT_BOUNDARY - 2;
    const int end_y = start_line + range;

    for (int i = LEFT_BOUNDARY + 1; i < end_x; i++) {
        for (int j = start_line; j < end_y && j < MT9V03X_H - 2; j++) {
            // 检测突变点：当前像素与前一像素相同，但与后一像素不同，且后面连续相同
            if (img_in[j][i] == img_in[j][i - 1] &&
                img_in[j][i] != img_in[j][i + 1] &&
                img_in[j][i + 1] == img_in[j][i + 2]) {
                number++;
            }
        }
    }
    return number;
}

uint8 grape_broom_monitor_parking(uint8_t img_in[MT9V03X_H][MT9V03X_W], uint8 start_line, uint8 range)
{
    // 边界检查
    if (start_line + range > MT9V03X_H || start_line >= MT9V03X_H) {
        return 255;  // 错误代码
    }

    uint8 max_connect_number = 0;

    // 画辅助线
    draw_parking_assist_lines(start_line, range);

    const int scan_end = start_line + range;
    for (int i = start_line; i < scan_end; i++) {
        uint8 connect_number[5] = {0};
        uint8 seed_connected[5] = {0};  // 标记种子点是否被连通
        uint8 in_region = 0;            // 是否在连通区域内
        uint8 connected_seeds = 0;      // 当前连通区域包含的种子点数量

        // 从左到右扫描
        for (int k = LEFT_BOUNDARY; k < RIGHT_BOUNDARY; k++) {
            if (img_in[i][k] != BLACK_PIXEL) {  // 使用宏定义
                if (!in_region) {
                    in_region = 1;
                    connected_seeds = 0;
                }
                // 检查是否是种子点
                for (int j = 0; j < 5; j++) {
                    if (k == grape_seed[j] && !seed_connected[j]) {  // 避免重复计数
                        seed_connected[j] = 1;
                        connected_seeds++;
                    }
                }
            } else {  // 遇到边界
                if (in_region) {
                    in_region = 0;
                    // 更新所有被连通的种子点的计数
                    for (int j = 0; j < 5; j++) {
                        if (seed_connected[j]) {
                            connect_number[j] = connected_seeds;
                            seed_connected[j] = 0;  // 重置标记
                        }
                    }
                }
            }
        }

        // 处理最后一个区域
        if (in_region) {
            for (int j = 0; j < 5; j++) {
                if (seed_connected[j]) {
                    connect_number[j] = connected_seeds;
                }
            }
        }

        // 获取该行最大连通数
        for (int j = 0; j < 5; j++) {
            if (connect_number[j] > max_connect_number) {
                max_connect_number = connect_number[j];
            }
        }
    }

    return max_connect_number;
}

//================== 内部辅助函数 ==================
static void display_calculation_results(const LineFitResult* result)
{
    const int base_y = MT9V03X_H;
    const int text_spacing = 16;
    const int value_x_offset = 5 * 16;  // 值的X偏移量

    ips200_show_string(0, base_y + text_spacing * 1, "Slope:");
    ips200_show_string(0, base_y + text_spacing * 2, "Intercept:");
    ips200_show_string(0, base_y + text_spacing * 3, "Angle:");
    ips200_show_string(0, base_y + text_spacing * 4, "Distance:");

    ips200_show_float(value_x_offset, base_y + text_spacing * 1, result->slope, 3, 2);
    ips200_show_float(value_x_offset, base_y + text_spacing * 2, result->intercept, 3, 2);
    ips200_show_float(value_x_offset, base_y + text_spacing * 3, result->angle, 3, 2);
    ips200_show_float(value_x_offset, base_y + text_spacing * 4, result->level_distance, 3, 2);
}

static void draw_parking_assist_lines(uint8 start_line, uint8 range)
{
    // 画垂直辅助线
    for (int i = 0; i < 5; i++) {
        ips200_draw_line(grape_seed[i], 0, grape_seed[i], MT9V03X_H, RGB565_RED);
    }
    // 画水平辅助线
    ips200_draw_line(LEFT_BOUNDARY, start_line, RIGHT_BOUNDARY, start_line, RGB565_RED);
    ips200_draw_line(LEFT_BOUNDARY, start_line + range, RIGHT_BOUNDARY, start_line + range, RGB565_RED);
}
