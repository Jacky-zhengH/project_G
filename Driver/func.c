#include "header.h"
//*********************************************************************************************************
#define DEVICE_MAX_VPP 3.0f // dds模块在AD9833_AmpSet(255)时可输出的最大幅值（需要实测修改）

//*********************************************************************************************************
/**
 * @name   Convert_Vpp_To_AmpValue(float vpp)
 * @brief  将物理电压Vpp转换为数字电位器的0-255设定值
 * @note   这是一个简化的线性标定, 依赖于 DEVICE_MAX_VPP 的准确性
 * @param  vpp: 探究装置需要输出的Vpp
 * @retval 0-255的幅度设定值
 */
//*********************************************************************************************************
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
//*********************************************************************************************************
/**
 * @name    Calculate_KnownModel_Gain()
 * @brief  计算“已知模型电路” H(s) 在指定频率下的增益 |H(jω)|
 * @note   H(s) = (10^-8 * s^2) / (s^2 + 3*10^-4 * s + 1)
 * @param  freq_hz: 输入频率
 * @retval 幅度增益 (一个 0.0 到 1.0 之间的浮点数)
 */
//*********************************************************************************************************
static float Calculate_KnownModel_Gain(float freq_hz)
{
    // 转换公式 H(jω) = 5 / ( (1 - 10^-8 * ω^2) + j(3*10^-4 * ω) )
    // 1.计算ω和ω^2的值
    float w = 2.0 * PI * freq_hz;
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
/**
 * @name   Func_Init(void)
 * @brief  初始化AD9833
 * @note
 * @param  无
 * @retval none
 */
//*********************************************************************************************************
void Func_Init(void)
{
    // 不需要AD9833_Init()，已经在hal库初始化
    AD9833_WaveSeting(0, 0, SIN_WAVE, 0);
    AD9833_AmpSet(0); // 数字电位器归零
}

//*********************************************************************************************************
/**
 * @name   Func_Basic2_SetSignal
 * @brief  基础部分第二问：生成频率可调（最高不小于1MHZ，步长100Hz）的正弦信号
 * @note   频率距相对误差绝对值不超过5%，各频点输出电压峰峰值的最大值不得小于3V
 * @param  frequence 频率
 * @retval none
 */
//*********************************************************************************************************
void Func_Basic2_SetSignal(float frequency)
{
    AD9833_WaveSeting(frequency, 0, SIN_WAVE, 0);
    AD9833_AmpSet(255); // 数字电位器直接拉到最高，调整到最高幅值
}

//*********************************************************************************************************
/**
 * @name   Fun_Basic3_4_SetSignal
 * @brief  基础部分第三,四问
 * @note
 * @param  frequence:频率
 * @param  target_model_output_vpp:指定输出幅值
 * @retval none
 */
//*********************************************************************************************************

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
