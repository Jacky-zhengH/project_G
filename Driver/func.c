#include "header.h"
//*********************************************************************************************************
extern ADC_HandleTypeDef hadc1;
// !! 必须与您在CubeMX中配置的通道号一致 !!
#define V_IN_ADC_CHANNEL ADC_CHANNEL_1  // PA1 -> ADC_CHANNEL_1 输入端
#define V_OUT_ADC_CHANNEL ADC_CHANNEL_2 // PA2 -> ADC_CHANNEL_2 输出端
//
#define DEVICE_MAX_VPP 4.9f       // dds模块在AD9833_AmpSet(255)时可输出的最大幅值（需要实测修改）！！
#define SWEEP_POINT 500           // 扫频点数，先设置为500个点
#define SWEEP_FREQ_START 200.0f   // 扫频起始频率 200Hz
#define SWEEP_FREQ_STEP 200.0f    // 扫频法频率步长 设置为200Hz 扫频范围为200Hz~100KHz
#define SWEEP_STAB_DELAY_ms 5     // 波形稳定延时时长
#define SWEEP_AMP_SETTING 150     // 扫频时信号幅值（1~255）
#define ADC_VPP_SAMPLE_COUNT 1024 // 每次测Vpp的ADC采样点数
//*********************************************************************************************************
static float Learning_Gain_Arry[SWEEP_POINT];
static float g_adc_dma_buffer[ADC_VPP_SAMPLE_COUNT]; // ADC-DMA的数据缓冲区域
//*********************************************************************************************************
/**
 * @name    Read_Vpp_Blocking
 * @brief   (阻塞式) 测量指定ADC通道的Vpp
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

    // 2. 启动ADC DMA (Normal模式)
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_adc_dma_buffer, ADC_VPP_SAMPLE_COUNT) != HAL_OK)
    {
        return 0.0f; // 错误
    }

    // 3. 等待DMA传输完成 (阻塞)
    if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) // 100ms超时
    {
        HAL_ADC_Stop_DMA(&hadc1);
        return 0.0f; // 错误
    }

    // 4. 停止DMA
    HAL_ADC_Stop_DMA(&hadc1);

    // 5. 在缓冲区中查找最大/最小值
    for (int i = 0; i < ADC_VPP_SAMPLE_COUNT; i++)
    {
        if (g_adc_dma_buffer[i] > max_val)
            max_val = g_adc_dma_buffer[i];
        if (g_adc_dma_buffer[i] < min_val)
            min_val = g_adc_dma_buffer[i];
    }

    // 6. 将ADC计数值的差值 (Vpp) 转换为物理电压
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

/**
 * @name   Func_Init(void)
 * @brief  初始化AD9833
 * @note
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
    printf("[Func] 停止所有输出\r\n"); // 调试信息
    AD9833_AmpSet(0);
    AD9833_WaveSeting(0.0, 0, SIN_WAVE, 0);

    // (未来，如果还有其他任务，比如PWM或DAC，也在这里关闭)
}

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
    AD9833_AmpSet(180); // 数字电位器设置为180，大约3.3V左右
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
FilterType Fun_Advanced1_StratLearning(void)
{
    // 设置扫频时的输出幅值
    AD9833_AmpSet(SWEEP_AMP_SETTING);
    // 开始扫频，从200Hz到100KHz，步长为200Hz，一共采集500个点
    for (int i = 0; i < SWEEP_POINT; i++)
    {
        float Sweep_Set_freq = SWEEP_FREQ_START + i * SWEEP_FREQ_STEP; // 设置频率
        AD9833_WaveSeting((double)Sweep_Set_freq, 0, SIN_WAVE, 0);     // 输出信号
        HAL_Delay(SWEEP_STAB_DELAY_ms);                                // 延时5ms，稳定波形方便测量
        /*采集输入电压和输出电压*/
        float V_in = Read_Vpp_Blocking(V_IN_ADC_CHANNEL);
        float V_out = Read_Vpp_Blocking(V_OUT_ADC_CHANNEL);
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
    return result;
}
