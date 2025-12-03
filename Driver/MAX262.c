#include "header.h"
//*********************************************************************************************************
// 内部变量：记录当前的频率档位
static uint8_t Nf = 0;
//*********************************************************************************************************
// uint8_t Qn(float q) // 品质因数关键字
// {
//     uint8_t temp;
//     temp = 128 - (64 / q);
//     if (temp == 0)
//     {
//         temp = 1;
//     } // 具体看芯片手册上有公式
//     return temp;
// }

// void MAX262_CLK_SET_PWM(float fc)
// {
//     if (fc <= 28000)
//     {
//         Nf = 63;
//     }
//     else
//     {
//         Nf = 0;
//     }
//     float ratio = (M_PI / 2.0f) * (float)(Nf + 26);
//     float pwm_freq = fc * ratio;
//     // 计算 ARR = (TimClock / PWM_Freq) - 1
//     uint32_t arr = (uint32_t)((float)TIM_CLOCK / pwm_freq) - 1;
//     if (arr < 2)
//         arr = 2;
//     if (arr > 65535)
//         arr = 65535;

//     __HAL_TIM_SET_AUTORELOAD(PWM_TIM, arr);
//     __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH, arr / 2); // 50% 占空比
//     HAL_TIM_PWM_Start(PWM_TIM, PWM_CH);              // pwm启动

//     Debug_printf("Set PWM Freq: %f \r\n", pwm_freq);
// }
// /**
//  * @brief 设置MAX262滤波器
//  * @param mode:工作模式
//  * @param q:品质因子
//  * @note  只能够设置模式（滤波类型）和传入品质因子Q，时钟输入则需要使用pwm或者SI5351模块
//  */
// void Filter1(uint8_t mode, float q)
// {
//     uint8_t i;
//     uint8_t a = 0x03;
//     uint8_t sq;

//     // uint16_t reload = (uint16_t)(256000000 / (f * 3.1415926 * (26 + Fn))) - 1;
//     // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)reload / 2);
//     // __HAL_TIM_SET_AUTORELOAD(&htim1, reload);
//     // 公式 (Mode 1): f_clk = f0 * (PI/2) * (N + 26)
//     // float ratio = (M_PI / 2.0f) * (float)(Nf + 26);
//     // float pwm_freq = fc * ratio;
//     // // 计算 ARR = (TimClock / PWM_Freq) - 1
//     // uint32_t arr = (uint32_t)((float)TIM_CLOCK / pwm_freq) - 1;

//     // if (arr < 2)
//     //     arr = 2;
//     // if (arr > 65535)
//     //     arr = 65535;

//     // __HAL_TIM_SET_AUTORELOAD(PWM_TIM, arr);
//     // __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH, arr / 2); // 50% 占空比
//     // HAL_TIM_PWM_Start(PWM_TIM, PWM_CH);              // pwm启动

//     i = sq = 0;
//     // 	sf = Fn(f);                     // 求出其频率控制字N
//     sq = Qn(q); // 求出Q对应的控制字N
//     LE_H;       // 使能端拉高
//                 //	Delay_ns(200);
//     HAL_Delay(1);
//     WR_H; // 写端口拉高
//           //	Delay_ns(200);
//     HAL_Delay(1);
//     // 具体可见芯片手册 P15 Table4

//     GPIOG->BSRR = 0x3f00 << 16; // 写入模式的地址
//                                 //	Delay_ns(200);
//     HAL_Delay(1);
//     WR_L; // 写端口拉低
//     //	Delay_ns(200);
//     HAL_Delay(1);
//     // 	GPIOE->BRR = 0x0300;
//     GPIOG->ODR |= ((uint16_t)(mode & 0x03) << 8); // 将模式控制字送给D1, D0
//                                                   //	Delay_ns(200);
//     HAL_Delay(1);
//     WR_H; // 写端口拉高
//           //	Delay_ns(200);
//     HAL_Delay(1);
//     for (i = 0; i < 3; i++)
//     {
//         GPIOG->BSRR = 0x3f00 << 16;            // 现将地址与数据位清0
//         GPIOG->ODR |= (uint16_t)(i + 1) << 10; // 写入地址
//                                                //		Delay_ns(200);
//         HAL_Delay(1);
//         WR_L; // 写入拉低
//               //		Delay_ns(200);
//         HAL_Delay(1);
//         // 		GPIOE->BRR = 0x0300;            //数据位清0
//         GPIOG->ODR |= ((uint16_t)(Nf & a) << (8 - 2 * i)); // 将f的控制字N写入
//                                                            //		Delay_ns(200);
//         HAL_Delay(1);
//         WR_H;       // 写入拉高
//         a = a << 2; // a左移2位
//     }

//     a = 0x03;

//     for (i = 0; i < 4; i++)
//     {
//         GPIOG->BSRR = 0x3f00 << 16;            // 现将地址与数据位清0
//         GPIOG->ODR |= (uint16_t)(i + 4) << 10; // 写入地址
//                                                //		Delay_ns(200);
//         HAL_Delay(1);
//         WR_L; // 写入拉低
//               //		Delay_ns(200);
//         HAL_Delay(1);
//         // 		GPIOE->BRR = 0x0300;              //数据位清0
//         GPIOG->ODR |= ((uint16_t)(sq & a) << (8 - 2 * i)); // 将Q的控制字N写入
//                                                            //		Delay_ns(200);
//         HAL_Delay(1);
//         WR_H;       // 写入拉高
//         a = a << 2; // a左移2位
//     }
// }
static void MAX262_Delay(void)
{
    // 适当增加延时循环，确保信号稳定
    for (volatile int i = 0; i < 20; i++)
        __NOP();
}

/**
 * @name    MAX262_WriteNibble()
 * @brief   写入半字节数据
 */
static void MAX262_WriteNibble(uint8_t addr, uint8_t data)
{
    // 1. 设置地址 A0-A3
    HAL_GPIO_WritePin(BUS_PORT, A0_PIN, (addr & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUS_PORT, A1_PIN, (addr & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUS_PORT, A2_PIN, (addr & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUS_PORT, A3_PIN, (addr & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // 2. 设置数据 D0-D1
    HAL_GPIO_WritePin(BUS_PORT, D0_PIN, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUS_PORT, D1_PIN, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    MAX262_Delay();

    // 3. 产生 WR 负脉冲 (写入数据)
    // 注意：LE 已经在 Init 或 Config 开始时拉高，这里不需要重复操作
    WR_L; // 拉低
    MAX262_Delay();
    WR_H; // 拉高 (上升沿锁存数据)
    MAX262_Delay();
}

/**
 * @name    MAX262_Init()
 * @brief   初始化MAX262模块 GPIO 状态并开启 PWM
 */
void MAX262_Init(void)
{
    // 1. 初始化控制引脚状态
    // 根据标准库代码，LE 拉高使能，WR 拉高空闲
    LE_H;
    WR_H;

    // 2. 启动 PWM
    HAL_TIM_PWM_Start(PWM_TIM, PWM_CH);
}

/**
 * @name    MAX262_Config()
 * @brief   配置 MAX262 的模式、Q值和频率
 * @param   filter_id: 0 = Filter A, 1 = Filter B
 * @param   mode: 工作模式 (1-4)
 * @param   q: 品质因数 (0.5 - 64.0)
 * @param   fc: 截止频率 (Hz)
 */
void MAX262_Config(uint8_t filter_id, uint8_t mode, float q, float fc)
{
    // 补全缺失的变量定义
    uint8_t Nf = 0;

    // 再次确保 LE 为高 (防止意外被拉低)
    LE_H;

    // 计算基地址偏移 (Filter A=0, Filter B=8)
    uint8_t base = (filter_id == 0) ? 0 : 8;

    // ------------------------------------------------------
    // 1. 设置模式 (Mode) -> 地址 0
    // ------------------------------------------------------
    uint8_t mode_code = (mode - 1) & 0x03;
    MAX262_WriteNibble(base + 0, mode_code);

    // ------------------------------------------------------
    // 2. 设置频率字 (Fn) -> 地址 1, 2, 3
    // ------------------------------------------------------
    // 你的策略：固定 N 值，通过改变 PWM 频率来调整 fc
    if (fc < 1000.0f)
    {
        Nf = 63; // 低频范围使用最大分频 N=63
    }
    else
    {
        Nf = 0; // 高频范围使用最小分频 N=0
    }

    // Fn 是 6 位数据，分3次写
    MAX262_WriteNibble(base + 1, Nf & 0x03);        // Bit 0-1
    MAX262_WriteNibble(base + 2, (Nf >> 2) & 0x03); // Bit 2-3
    MAX262_WriteNibble(base + 3, (Nf >> 4) & 0x03); // Bit 4-5

    // ------------------------------------------------------
    // 3. 设置 Q 值 (Nq) -> 地址 4, 5, 6, 7
    // ------------------------------------------------------
    // 公式: Nq = 128 - (64 / Q)
    if (q < 0.5f)
        q = 0.5f;
    if (q > 64.0f)
        q = 64.0f;

    int nq_int = (int)(128.0f - (64.0f / q) + 0.5f);
    // 边界检查
    if (nq_int < 0)
        nq_int = 0;
    if (nq_int > 127)
        nq_int = 127;

    uint8_t nq = (uint8_t)nq_int;

    // Nq 是 7 位数据，分4次写
    MAX262_WriteNibble(base + 4, nq & 0x03);        // Bit 0-1
    MAX262_WriteNibble(base + 5, (nq >> 2) & 0x03); // Bit 2-3
    MAX262_WriteNibble(base + 6, (nq >> 4) & 0x03); // Bit 4-5
    MAX262_WriteNibble(base + 7, (nq >> 6) & 0x01); // Bit 6

    // ------------------------------------------------------
    // 4. 设置 STM32 PWM (Fclk)
    // ------------------------------------------------------
    // 注意：MAX262 的 Mode 1 公式: f_clk / fc = PI/2 * (N + 26)
    // 所以 f_clk = fc * (PI/2) * (N + 26)

    // 如果两个滤波器需要不同的 f_clk，这里只能满足其中一个 (假设共用 TIM1_CH1)
    // 通常 Filter A 设置一次即可
    if (filter_id == 0)
    {
        float ratio = (M_PI / 2.0f) * (float)(Nf + 26);
        float pwm_freq = fc * ratio;

        // 防止除零
        if (pwm_freq < 1.0f)
            pwm_freq = 1.0f;

        // 计算 ARR = (TimClock / PWM_Freq) - 1
        uint32_t arr = (uint32_t)((float)TIM_CLOCK / pwm_freq) - 1;

        // 硬件限制检查
        if (arr < 2)
            arr = 2;
        if (arr > 65535)
            arr = 65535;

        // 更新定时器参数
        __HAL_TIM_SET_AUTORELOAD(PWM_TIM, arr);
        __HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CH, arr / 2); // 50% 占空比

        // 关键：产生更新事件，使 ARR 立即生效 (否则要等到下个溢出才生效)
        HAL_TIM_GenerateEvent(PWM_TIM, TIM_EVENTSOURCE_UPDATE);
    }
}
//*********************************************************************************************************
