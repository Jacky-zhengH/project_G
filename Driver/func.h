#ifndef __FUNC_H
#define __FUNC_H

typedef enum
{
    FILTER_LOW_PASS,  // 低通
    FILTER_HIGH_PASS, // 高通
    FILTER_BAND_PASS, // 带通
    FILTER_BAND_STOP, // 带阻
    FILTER_UNKNOW,    // 未知
} FilterType;

void Func_Init(void);
void Func_Stop_All_Output(void);
void Func_Basic2_SetSignal(float frequency);
void Func_Basic3_4_SetSignal(float frequency, float target_model_output_vpp);
FilterType Fun_Advanced1_StratLearning(void);

#endif
