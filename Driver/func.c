#include "header.h"
//*********************************************************************************************************
extern ADC_HandleTypeDef hadc1;
// !! 必须与您在CubeMX中配置的通道号一致 !!
#define V_IN_ADC_CHANNEL ADC_CHANNEL_1  // PA1 --> ADC_CHANNEL_1 输入端
#define V_OUT_ADC_CHANNEL ADC_CHANNEL_2 // PA2 --> ADC_CHANNEL_2 输出端
//
#define DEVICE_MAX_VPP 6.6f // dds模块在AD9833_AmpSet(255)时可输出的最大幅值（需要实测修改）！！
#define SWEEP_POINT 500     // 扫频点数，设置为500个点
// #define SWEEP_POINT            // 扫频点数，设置为996个点【修改】
#define SWEEP_FREQ_START 200.0f // 扫频起始频率 200Hz
// #define SWEEP_FREQ_START 500.0f   // 扫频起始频率 200Hz【修改】
#define SWEEP_FREQ_STEP 200.0f // 扫频法频率步长 设置为200Hz 扫频范围为200Hz~100KHz
// #define SWEEP_FREQ_STEP 100.0f    // 扫频法频率步长 设置为100Hz 扫频范围为500Hz~100KHz【修改】
#define SWEEP_STAB_DELAY_ms 20    // 波形稳定延时时长
#define SWEEP_AMP_SETTING 85      // 扫频时信号幅值（1~255），85时顶端值是2.2v左右！！
#define ADC_VPP_SAMPLE_COUNT 1024 // 每次测Vpp的ADC采样点数
#define FIXED_VIN_VOLTAGE 2.16f   // 【新增】Vin固定输出值
//*********************************************************************************************************
static float Learning_Gain_Arry[SWEEP_POINT];
static uint16_t g_adc_dma_buffer[ADC_VPP_SAMPLE_COUNT]; // ADC-DMA的数据缓冲区域
static volatile uint8_t g_adc_dma_done = 0;             // 增加DMA完成标志位
/*全局变量：滤波类型和截止频率以及品质因子*/
FilterType G_Learned_Type = FILTER_UNKNOW; // 未知电路滤波类型
float G_Learned_Fc = 1000.0f;              // 截止频率默认值
float G_Learned_Q = 0.707f;                // 品质因子Q默认值
//*********************************************************************************************************
/**
 * @name    Read_Vpp_Blocking
 * @brief   (连续转换模式)测量指定ADC通道的Vpp
 * @note    使用DMA采集N个点, 然后在内存中查找最大/最小值
 * @param   ADC_Channel: 要测量的ADC通道 (例如 V_IN_ADC_CHANNEL)
 * @retval  浮点数的电压值 (V)
 */
static float Read_Vpp_Blocking(uint32_t ADC_Channel)
{
    uint16_t max_val = 0;
    uint16_t min_val = 4095;

    // 1. 切换ADC通道
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_Channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; // 采样时间 (如果波形频率很高, 可能需要更短的采样时间)
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0.0f; // 错误
    }
    // 2. 清除完成标志位
    g_adc_dma_done = 0;

    // 3. 启动ADC DMA (Normal模式)
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer, ADC_VPP_SAMPLE_COUNT) != HAL_OK)
    {
        return 0.0f; // 错误
    }

    // 4. 等待DMA传输完成 (改用标志位等待)
    // 由于使用连续转换形式，不再使用 HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK
    uint32_t timeout = 1000;                   // 1秒超时
    while (g_adc_dma_done == 0 && timeout > 0) // 100ms超时
    {
        HAL_Delay(1);
        timeout--;
    }

    // 5. 停止DMA
    HAL_ADC_Stop_DMA(&hadc1);
    if (timeout == 0) // 超时
    {
        return 0.0f;
    }

    // 6. 在缓冲区中查找最大/最小值
    for (int i = 0; i < ADC_VPP_SAMPLE_COUNT; i++)
    {
        if (g_adc_dma_buffer[i] > max_val)
            max_val = g_adc_dma_buffer[i];
        if (g_adc_dma_buffer[i] < min_val)
            min_val = g_adc_dma_buffer[i];
    }

    // 7. 将ADC计数值的差值 (Vpp) 转换为物理电压
    //    V = (counts / 4095) * 3.3V (假设ADC是12位, Vref=3.3V)
    float vpp_volts = ((float)max_val - (float)min_val) * (3.3f / 4095.0f);

    return vpp_volts;
}

/**
 * @name    FILTER_type_analysis()
 * @brief   滤波类型判断
 * @note    通过接受并分析增益数组数据，判断滤波类型
 * @param   *array：增益数组
 * @param   point_value：扫频数量（数据个数）
 * @retval  返回滤波类型变量
 */
static FilterType FILTER_type_analysis(float *array, int point_value)
{
    float gain_low_sum = 0.0f;
    float gain_high_sum = 0.0f;
    float gain_max = 0.0f;
    float gain_min = 1e9f; // 一个很大的初始值

    int avg_count = 10; // 取10个点计算平均值

    // 1. 遍历数组, 找到最大/最小值, 并累加低频/高频段
    for (int i = 0; i < point_value; i++)
    {
        if (array[i] > gain_max)
            gain_max = array[i];
        if (array[i] < gain_min)
            gain_min = array[i];

        if (i < avg_count)
        {
            gain_low_sum += array[i];
        }
        if (i >= (point_value - avg_count))
        {
            gain_high_sum += array[i];
        }
    }

    // 2. 计算平均增益
    float gain_low = gain_low_sum / avg_count;
    float gain_high = gain_high_sum / avg_count;

    // 3. 定义门限 (基于最大增益的百分比)
    float threshold_high = gain_max * 0.5f; // "通过"门限
    float threshold_low = gain_max * 0.3f;  // "阻断"门限

    // 4. 判断逻辑

    // 低通: 低频通过, 高频阻断
    if (gain_low > threshold_high && gain_high < threshold_low)
    {
        return FILTER_LOW_PASS;
    }
    // 高通: 低频阻断, 高频通过
    else if (gain_low < threshold_low && gain_high > threshold_high)
    {
        return FILTER_HIGH_PASS;
    }
    // 带通: 低频阻断, 高频阻断
    else if (gain_low < threshold_low && gain_high < threshold_low)
    {
        return FILTER_BAND_PASS;
    }
    // 带阻: 低频通过, 高频通过
    else if (gain_low > threshold_high && gain_high > threshold_high)
    {
        return FILTER_BAND_STOP;
    }

    return FILTER_UNKNOW; // 无法识别
}
//*********************************************************************************************************
/**
 * @name   Convert_Vpp_To_AmpValue(float vpp)
 * @brief  将物理电压Vpp转换为数字电位器的0-255设定值
 * @note   这是一个简化的线性标定, 依赖于 DEVICE_MAX_VPP 的准确性
 * @param  vpp: 探究装置需要输出的Vpp
 * @retval 0-255的幅度设定值
 */
static uint8_t Convert_Vpp_To_AmpValue(float vpp)
{
    float amp_value_f;

    // 限制最大值 (如果计算出的Vpp超过模块能力, 则使用最大值)
    if (vpp > DEVICE_MAX_VPP)
    {
        vpp = DEVICE_MAX_VPP;
    }
    // 限制最小值
    if (vpp < 0.0f)
    {
        vpp = 0.0f;
    }

    // 线性映射：(vpp/max)*255
    amp_value_f = (vpp / DEVICE_MAX_VPP) * 255.0f;
    return (uint8_t)(amp_value_f + 0.5f); // 四舍五入转换为整数
}

/**
 * @name    Calculate_KnownModel_Gain()
 * @brief  计算“已知模型电路” H(s) 在指定频率下的增益 |H(jω)|
 * @note   H(s) = (10^-8 * s^2) / (s^2 + 3*10^-4 * s + 1)
 * @param  freq_hz: 输入频率
 * @retval 幅度增益 (一个 0.0 到 1.0 之间的浮点数)
 */
static float Calculate_KnownModel_Gain(float freq_hz)
{
    // 转换公式 H(jω) = 5 / ( (1 - 10^-8 * ω^2) + j(3*10^-4 * ω) )
    // 1.计算ω和ω^2的值
    float w = 2.0f * PI * freq_hz;
    float w2 = w * w;
    // 2. 计算分子幅值
    // |Num| = |5| = 5.0
    float num_mag = 5.0f;

    // 3. 计算分母幅值
    // |Den| = sqrt( (1 - 10^-8 * w^2)^2 + (3*10^-4 * w)^2 )
    float den_real = 1.0f - 1e-8f * w2; // (1 - 10^-8 * ω^2)
    float den_imag = 3e-4f * w;         // (3*10^-4 * ω)
    float den_mag;

    // 使用FPU的快速平方根函数 (来自 arm_math.h)
    arm_sqrt_f32(den_real * den_real + den_imag * den_imag, &den_mag);
    // 4. 计算总增益 |H(jω)| = |Num| / |Den|
    if (den_mag < 1e-9f) // 避免除以零
    {
        return 0.0f;
    }

    return num_mag / den_mag;
}
//*********************************************************************************************************
/*【新增中值滤波算法】*/
/**
 * @brief  用于 qsort 的浮点数比较函数
 */
static int compare_floats(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) ? 1 : (fa < fb) ? -1
                                     : 0;
}

/**
 * @brief  中值滤波器 (去除野值)
 * @note   用于去除 ADC 偶发的跳变干扰
 */
void median_filter(float *input, float *output, int length, int window_size)
{
    if (!input || !output || length <= 0 || window_size < 3)
        return;

    int radius = window_size / 2;
    float window[32]; // 窗口缓存

    for (int i = 0; i < length; i++)
    {
        // 1. 填充窗口 (处理边界)
        int count = 0;
        for (int j = -radius; j <= radius; j++)
        {
            int idx = i + j;
            if (idx >= 0 && idx < length)
            {
                window[count++] = input[idx];
            }
        }

        // 2. 排序
        qsort(window, count, sizeof(float), compare_floats);

        // 3. 取中值
        output[i] = window[count / 2];
    }
}
/*【新增带阻和带通品质因子算法】*/
/**
 * @brief  计算峰值带宽 (用于带通)
 * @note   返回的是索引宽度
 */
uint16_t BW_Calculate_BP_IndexWidth(float *input, int length, float max_val, int max_idx)
{
    int left_idx = 0;
    int right_idx = length - 1;
    float threshold = max_val * 0.707f;

    // 向右搜 (高频边沿)
    for (int i = max_idx; i < length - 1; i++)
    {
        if (input[i] < threshold)
        {
            right_idx = i;
            break;
        }
    }
    // 向左搜 (低频边沿)
    for (int i = max_idx; i > 0; i--)
    {
        if (input[i] < threshold)
        {
            left_idx = i;
            break;
        }
    }

    // 保护：防止宽度为0
    if (right_idx <= left_idx)
        return 10; // 默认给个小宽度

    return (uint16_t)(right_idx - left_idx);
}

/**
 * @brief  计算陷波带宽 (用于带阻)
 * @note   寻找增益回升到 0.707 * MaxGain 的宽度
 */
uint16_t BW_Calculate_BS_IndexWidth(float *input, int length, float passband_gain, int min_idx)
{
    int left_idx = 0;
    int right_idx = length - 1;
    float threshold = passband_gain * 0.707f; // 以通带增益为基准下降3dB

    // 向右搜 (回到通带的点)
    for (int i = min_idx; i < length - 1; i++)
    {
        if (input[i] > threshold)
        {
            right_idx = i;
            break;
        }
    }
    // 向左搜
    for (int i = min_idx; i > 0; i--)
    {
        if (input[i] > threshold)
        {
            left_idx = i;
            break;
        }
    }

    if (right_idx <= left_idx)
        return 50; // 带阻通常较宽

    return (uint16_t)(right_idx - left_idx);
}
//-------------------------------------------------------
/**
 * @name    Extract_Filter_Params()
 * @brief   从增益数组中提取物理参数 Fc 和 Q
 * @note    none
 * @param   type: 已识别的滤波器类型
 * @retval  none
 */
static void Extract_Filter_Params(FilterType type)
{
    // // =================================================================
    // // 【修改 1】 引入中值滤波，提升抗噪能力
    // // =================================================================
    // float smoothed_array[SWEEP_POINT];
    // median_filter(Learning_Gain_Arry, smoothed_array, SWEEP_POINT, 5);

    float max_gain = 0.0f;
    int max_idx = 0;
    float min_gain = 10000.0f;
    int min_idx = 0;

    // // 1. 预处理：找到全局最大值和最小值及其索引
    // // 【修改】 所有的 Learning_Gain_Arry 都要替换为 smoothed_array
    // for (int i = 0; i < SWEEP_POINT; i++)
    // {
    //     if (smoothed_array[i] > max_gain)
    //     {
    //         max_gain = smoothed_array[i];
    //         max_idx = i;
    //     }
    //     if (smoothed_array[i] < min_gain)
    //     {
    //         min_gain = smoothed_array[i];
    //         min_idx = i;
    //     }
    // }

    // // 防止除以零的保护
    // if (max_gain < 0.01f)
    //     max_gain = 0.01f;

    // // 2. 根据类型提取参数
    // if (type == FILTER_LOW_PASS)
    // {
    //     // --- 低通逻辑 ---
    //     // Fc: 从 0 向后找，直到增益 < 0.707 * max_gain
    //     // 【优化】如果存在谐振峰，基准应该是直流增益，这里做一个兼容处理
    //     float dc_gain = smoothed_array[0];
    //     float base_gain = (max_gain > dc_gain * 1.2f) ? max_gain : dc_gain;
    //     float target_gain = base_gain * 0.707f;

    //     int cutoff_idx = max_idx;

    //     for (int i = max_idx; i < SWEEP_POINT; i++)
    //     {
    //         if (smoothed_array[i] < target_gain)
    //         {
    //             cutoff_idx = i;
    //             break;
    //         }
    //     }
    //     G_Learned_Fc = SWEEP_FREQ_START + cutoff_idx * SWEEP_FREQ_STEP;

    //     // Q值: 检查是否有谐振峰 (Max Gain > 1.2 * DC Gain)
    //     if (dc_gain > 0.01f && max_gain > dc_gain * 1.1f) // 稍微放宽到 1.1
    //     {
    //         // 有尖峰，估算 Q
    //         G_Learned_Q = max_gain / dc_gain;
    //     }
    //     else
    //     {
    //         // 平坦，默认 Q
    //         G_Learned_Q = 0.707f;
    //     }
    // }
    // else if (type == FILTER_HIGH_PASS)
    // {
    //     // --- 高通逻辑 ---
    //     float target_gain = max_gain * 0.707f;
    //     int cutoff_idx = max_idx;

    //     for (int i = max_idx; i >= 0; i--)
    //     {
    //         if (smoothed_array[i] < target_gain)
    //         {
    //             cutoff_idx = i;
    //             break;
    //         }
    //     }
    //     G_Learned_Fc = SWEEP_FREQ_START + cutoff_idx * SWEEP_FREQ_STEP;

    //     // 高通 Q 值默认为 0.707
    //     G_Learned_Q = 0.707f;
    // }
    // else if (type == FILTER_BAND_PASS)
    // {
    //     // --- 带通逻辑 ---
    //     G_Learned_Fc = SWEEP_FREQ_START + max_idx * SWEEP_FREQ_STEP;

    //     // Q值: 计算带宽 Q = Fc / BW
    //     // 使用函数BW_Calculate_BP_IndexWidth
    //     uint16_t width_idx = BW_Calculate_BP_IndexWidth(smoothed_array, SWEEP_POINT, max_gain, max_idx);
    //     float bw = width_idx * SWEEP_FREQ_STEP;

    //     if (bw > 100.0f)
    //         G_Learned_Q = G_Learned_Fc / bw;
    //     else
    //         G_Learned_Q = 5.0f; // 带宽极窄
    // }
    // else if (type == FILTER_BAND_STOP)
    // {
    //     // --- 带阻逻辑 ---
    //     // Fc: 最小值所在频率 (陷波点)
    //     G_Learned_Fc = SWEEP_FREQ_START + min_idx * SWEEP_FREQ_STEP;

    //     // 【补全】带阻 Q 值计算 (原代码缺失)
    //     // 使用封装函数寻找 -3dB 回升带宽
    //     // Passband gain 近似为 max_gain
    //     uint16_t width_idx = BW_Calculate_BS_IndexWidth(smoothed_array, SWEEP_POINT, max_gain, min_idx);
    //     float bw = width_idx * SWEEP_FREQ_STEP;

    //     if (bw > 100.0f)
    //         G_Learned_Q = G_Learned_Fc / bw;
    //     else
    //         G_Learned_Q = 1.0f;
    // }

    // // 3. 安全限制 ( MAX262 的极限)
    // if (G_Learned_Q > 10.0f)
    //     G_Learned_Q = 10.0f;
    // if (G_Learned_Q < 0.5f)
    //     G_Learned_Q = 0.5f;

    // // 打印调试信息
    // G_Learned_Type = type;
    // Debug_printf("Result Saved: Type=%d, Fc=%.1f, Q=%.2f\r\n", G_Learned_Type, G_Learned_Fc, G_Learned_Q);

    // float max_gain = 0.0f;
    // int max_idx = 0;
    // float min_gain = 10000.0f;
    // int min_idx = 0;
    //------------------------------------------------------
    // 1. 预处理：找到全局最大值和最小值及其索引
    for (int i = 0; i < SWEEP_POINT; i++)
    {
        if (Learning_Gain_Arry[i] > max_gain)
        {
            max_gain = Learning_Gain_Arry[i];
            max_idx = i;
        }
        if (Learning_Gain_Arry[i] < min_gain)
        {
            min_gain = Learning_Gain_Arry[i];
            min_idx = i;
        }
    }

    // 防止除以零的保护
    if (max_gain < 0.01f)
        max_gain = 0.01f;

    // 2. 根据类型提取参数
    if (type == FILTER_LOW_PASS)
    {
        // --- 低通逻辑 ---
        // Fc: 从 0 向后找，直到增益 < 0.707 * max_gain
        float target_gain = max_gain * 0.707f;
        int cutoff_idx = max_idx; // 从峰值位置开始找(防止有谐振峰导致误判)

        for (int i = max_idx; i < SWEEP_POINT; i++)
        {
            if (Learning_Gain_Arry[i] < target_gain)
            {
                cutoff_idx = i;
                break;
            }
        }
        G_Learned_Fc = SWEEP_FREQ_START + cutoff_idx * SWEEP_FREQ_STEP;

        // Q值: 检查是否有谐振峰 (Max Gain > 1.2 * DC Gain)
        float dc_gain = Learning_Gain_Arry[0]; // 近似直流增益
        if (dc_gain > 0.01f && max_gain > dc_gain * 1.2f)
        {
            // 有尖峰，估算 Q
            G_Learned_Q = max_gain / dc_gain;
        }
        else
        {
            // 平坦，默认 Q
            G_Learned_Q = 0.707f;
        }
    }
    else if (type == FILTER_HIGH_PASS)
    {
        // --- 高通逻辑 ---
        // Fc: 从 max_idx 向前找，直到增益 < 0.707 * max_gain
        float target_gain = max_gain * 0.707f;
        int cutoff_idx = max_idx;

        for (int i = max_idx; i >= 0; i--)
        {
            if (Learning_Gain_Arry[i] < target_gain)
            {
                cutoff_idx = i;
                break;
            }
        }
        G_Learned_Fc = SWEEP_FREQ_START + cutoff_idx * SWEEP_FREQ_STEP;

        // 高通 Q 值较难通过 DC 增益判断，通常设为默认或简单判断峰值
        // 简单策略：如果 max_idx 很靠前且 gain 很高，可能有 Q
        G_Learned_Q = 0.707f;
    }
    else if (type == FILTER_BAND_PASS)
    {
        // --- 带通逻辑 ---
        // Fc: 就是最大值所在频率
        G_Learned_Fc = SWEEP_FREQ_START + max_idx * SWEEP_FREQ_STEP;

        // Q值: 计算带宽 Q = Fc / BW
        float target_gain = max_gain * 0.707f;
        int f_low_idx = 0;
        int f_high_idx = SWEEP_POINT - 1;

        // 向左找下边界
        for (int i = max_idx; i >= 0; i--)
        {
            if (Learning_Gain_Arry[i] < target_gain)
            {
                f_low_idx = i;
                break;
            }
        }
        // 向右找上边界
        for (int i = max_idx; i < SWEEP_POINT; i++)
        {
            if (Learning_Gain_Arry[i] < target_gain)
            {
                f_high_idx = i;
                break;
            }
        }

        // 计算带宽频率
        float f_low = SWEEP_FREQ_START + f_low_idx * SWEEP_FREQ_STEP;
        float f_high = SWEEP_FREQ_START + f_high_idx * SWEEP_FREQ_STEP;
        float bw = f_high - f_low;

        if (bw > 100.0f) // 防止带宽过小导致 Q 无穷大
        {
            G_Learned_Q = G_Learned_Fc / bw;
        }
        else
        {
            G_Learned_Q = 5.0f; // 带宽极窄，给个高 Q 值
        }
    }
    else if (type == FILTER_BAND_STOP)
    {
        // --- 带阻逻辑 ---
        // Fc: 最小值所在频率 (陷波点)
        G_Learned_Fc = SWEEP_FREQ_START + min_idx * SWEEP_FREQ_STEP;

        // 带阻通常 Q 值较高，或者设为默认
        G_Learned_Q = 1.0f; // 默认值
    }

    // 3. 安全限制 (防止参数超出 MAX262 物理极限)
    if (G_Learned_Q > 10.0f)
        G_Learned_Q = 10.0f;
    if (G_Learned_Q < 0.5f)
        G_Learned_Q = 0.5f;

    // 打印调试信息，让你看到算出来的结果
    G_Learned_Type = type;

    Debug_printf("Result Saved: Type=%d, Fc=%.1f, Q=%.2f\r\n", G_Learned_Type, G_Learned_Fc, G_Learned_Q);
}
//*********************************************************************************************************/**

/**
 * @name   Func_Init(void)
 * @brief  初始化AD9833
 * @param  无
 * @retval none
 */
void Func_Init(void)
{
    // 不需要AD9833_Init()，已经在hal库初始化
    AD9833_WaveSeting(0, 0, SIN_WAVE, 0);
    AD9833_AmpSet(0); // 数字电位器归零
}

/**
 * @name   Func_Stop_All_Output()
 * @brief  停止所有输出
 * @note   防止某个任务设置的DDS输出影响其他任务
 * @param  无
 * @retval none
 */
void Func_Stop_All_Output(void)
{
    Debug_printf("clear\r\n");
    AD9833_AmpSet(0);
    AD9833_WaveSeting(0.0, 0, SIN_WAVE, 0);

    // (未来，如果还有其他任务，比如PWM或DAC，也在这里关闭)
}

/**
 * @name    LED_Debug()
 * @brief   控制PC13的LED亮起或者熄灭
 * @note    用于程序现象调试
 * @param   state:=为1则亮，为0则灭
 * @retval  none
 */
void LED_Debug(int state)
{
    if (state)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

/*!!重要改动!!*/
// 添加DMA完成回调函数
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        g_adc_dma_done = 1;
    }
}

//*********************************************************************************************************

/**
 * @name   Func_Basic2_SetSignal
 * @brief  基础部分第二问：生成频率可调（最高不小于1MHZ，步长100Hz）的正弦信号
 * @note   频率距相对误差绝对值不超过5%，各频点输出电压峰峰值的最大值不得小于3V
 * @param  frequence 频率
 * @retval none
 */
void Func_Basic2_SetSignal(float frequency)
{
    AD9833_WaveSeting(frequency, 0, SIN_WAVE, 0);
    AD9833_AmpSet(180); // 数字电位器设置为180，大约4.5V左右
}

/**
 * @name   Fun_Basic3_4_SetSignal
 * @brief  基础部分第三,四问
 * @note   计算已知电路在设定frequency处的增益，结合指定输出Vpp的值反推出输入电压
 * @param  frequence:频率
 * @param  target_model_output_vpp:指定输出幅值（Vpp）
 * @retval none
 */
void Func_Basic3_4_SetSignal(float frequency, float target_model_output_vpp)
{
    // 1. 计算"已知模型"在 f 处的增益
    float model_gain = Calculate_KnownModel_Gain(frequency);

    // 2. 计算"探究装置"需要输出的电压 V_in
    //    V_in = V_out / Gain
    float required_v_in;
    if (model_gain < 1e-9f) // 检查增益是否过小 (接近于0)
    {
        required_v_in = DEVICE_MAX_VPP; // 如果增益为0, 我们无法产生输出, 只能设为最大值
    }
    else
    {
        required_v_in = target_model_output_vpp / model_gain; //
    }

    // 3. 将 V_in 转换为数字电位器的设定值 (0-255)
    uint8_t amp_value = Convert_Vpp_To_AmpValue(required_v_in);

    // 4. 设置硬件
    AD9833_WaveSeting((double)frequency, 0, SIN_WAVE, 0); // 设置频率
    AD9833_AmpSet(amp_value);                             // 设置计算出的幅度
}

/**
 * @name   Fun_Advanced1_StratLearning(void)
 * @brief  发挥部分第一问
 * @note   使用扫频法，构建增益数组。然后再通过数据分析数组，判断类型
 * @param   无
 * @retval  返回类型变量（高通，低通，带通，带阻）
 */
FilterType Func_Advanced1_StratLearning(void)
{
    // 设置扫频时的输出幅值
    AD9833_AmpSet(SWEEP_AMP_SETTING); // 固定为Vpp=3.3V
    // 开始扫频，从200Hz到100KHz，步长为200Hz，一共采集500个点
    for (int i = 0; i < SWEEP_POINT; i++)
    {
        float Sweep_Set_freq = SWEEP_FREQ_START + i * SWEEP_FREQ_STEP; // 设置频率
        AD9833_WaveSeting((double)Sweep_Set_freq, 0, SIN_WAVE, 0);     // 输出信号
        HAL_Delay(SWEEP_STAB_DELAY_ms);                                // 延时5ms，稳定波形方便测量
        /*采集输入电压和输出电压*/
        // float V_in = Read_Vpp_Blocking(V_IN_ADC_CHANNEL);
        float V_in = FIXED_VIN_VOLTAGE; // 既然已经固定Vin幅值，那也可以不需要花费资源重新读取了
        // float V_in = Read_Vpp_Blocking(V_IN_ADC_CHANNEL);
        float V_out = Read_Vpp_Blocking(V_OUT_ADC_CHANNEL);
        /*添加扫频输出调试*/
        if (i % 10 == 0)
        {
            Debug_printf("Point %d: Freq=%.0fHz, Vin=%.3fV, Vout=%.3fV\r\n",
                         i, Sweep_Set_freq, V_in, V_out);
        }

        /*计算增益并放入增益学习数组*/
        if (V_in < 0.01f)
        {
            Learning_Gain_Arry[i] = 0.0f;
        }
        else
        {
            Learning_Gain_Arry[i] = V_out / V_in;
        }
    }
    /*DDS停止发出信号*/
    AD9833_WaveSeting(0, 0, SIN_WAVE, 0);
    AD9833_AmpSet(0);
    FilterType result = FILTER_type_analysis(Learning_Gain_Arry, SWEEP_POINT);
    Extract_Filter_Params(result);
    return result;
}

/**
 * @name    Func2_Advanced2_Imitate(void)
 * @brief   学习滤波类型并通过算法模拟输出
 * @note    通过滤波类型判断使用哪个通道和工作模式，调用MAX262模块使用
 * @param   无
 * @retval  none
 */

void Func2_Advanced2_Imitate(void)
{
    Debug_printf("--- Start Imitating ---\r\n");

    // 安全检查：如果没有学习过，就不要乱动
    if (G_Learned_Type == FILTER_UNKNOW)
    {
        Debug_printf("Error: No model learned yet!\r\n");
        return;
    }

    // 1. 硬件复位 (断开继电器)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

    uint8_t target_mode = 1;
    uint16_t active_relay = 0;

    // 2. 决策逻辑 (根据全局变量 G_Learned_Type)
    switch (G_Learned_Type)
    {
    case FILTER_LOW_PASS: // 低通
        target_mode = 1;
        active_relay = GPIO_PIN_3; // PD3
        break;
    case FILTER_BAND_PASS: // 带通
        target_mode = 1;
        active_relay = GPIO_PIN_1; // PD1
        break;
    case FILTER_HIGH_PASS:
        target_mode = 3;           // Mode 3 支持高通
        active_relay = GPIO_PIN_2; // PD2
        break;
    case FILTER_BAND_STOP:
        target_mode = 1;
        active_relay = GPIO_PIN_2; // PD2
        break;
    default:
        return;
    }

    // 3. 配置 MAX262 (使用全局变量 Fc, Q)
     MAX262_Config(0, target_mode, G_Learned_Q, G_Learned_Fc);
    /*使用STM32 pwm做时钟输入*/
    //MAX262_CLK_SET_PWM(G_Learned_Fc);

    // 配置滤波模块MAX262
    //Filter1(target_mode, G_Learned_Q);

    // 4. 开启继电器
    if (active_relay != 0)
    {
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOD, active_relay, GPIO_PIN_SET);
    }

    Debug_printf("Imitating: Mode=%d, Pin=PD%d\r\n", target_mode,
                 (active_relay == GPIO_PIN_1) ? 1 : (active_relay == GPIO_PIN_2) ? 2
                                                                                 : 3);
}
