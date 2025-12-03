#ifndef __SI5351_H
#define __SI5351_H

#include "main.h"
#include "stdbool.h"
//****************************************
// 该模块使用I2C通信（PB6,PB7）
//
//		 Pin assign
//			STM32			SI5351
//		GPIOB_Pin_6 		---> SCL
//		GPIOB_Pin_7 		---> SDA
//
//****************************************
// I2C地址映射
#define SI5351_I2C_ADDR (0x60 << 1)
#define SI5351_CRYSTAL_FREQ 25000000
//*********************************************************************************************************
/* =========================================================================
   寄存器地址映射 (Register Map)
   ========================================================================= */
#define SI_REG_DEVICE_STATUS 0
#define SI_REG_OUTPUT_ENABLE 3
#define SI_REG_CLK0_CTRL 16
#define SI_REG_CLK1_CTRL 17
#define SI_REG_CLK2_CTRL 18
#define SI_REG_CLK3_CTRL 19
#define SI_REG_CLK4_CTRL 20
#define SI_REG_CLK5_CTRL 21
#define SI_REG_CLK6_CTRL 22
#define SI_REG_CLK7_CTRL 23
#define SI_REG_PLL_A_BASE 26
#define SI_REG_PLL_B_BASE 34
#define SI_REG_MS0_BASE 42
#define SI_REG_MS1_BASE 50
#define SI_REG_MS2_BASE 58
#define SI_REG_PLL_RESET 177
#define SI_REG_CRYSTAL_LOAD 183

/* =========================================================================
   宏定义：晶振负载电容 (Reg 183)
   ========================================================================= */
#define SI_XTAL_6PF (1 << 6)
#define SI_XTAL_8PF (2 << 6)
#define SI_XTAL_10PF (3 << 6) // 默认推荐值

/* =========================================================================
   宏定义：时钟控制 (Reg 16-23)
   ========================================================================= */
// 驱动能力 (Drive Strength)
#define SI_DRIVE_2MA 0x00
#define SI_DRIVE_4MA 0x01
#define SI_DRIVE_6MA 0x02
#define SI_DRIVE_8MA 0x03 // 默认最强驱动

// 时钟输入源 (Input Source)
#define SI_SRC_XTAL (0 << 2)  // 直接输出晶振频率
#define SI_SRC_CLKIN (1 << 2) // 直接输出 CLKIN 引脚频率
#define SI_SRC_MS (3 << 2)    // 正常模式：经由 MultiSynth 输出

// 其他控制位
#define SI_CLK_INV (1 << 4)   // 反转时钟相位
#define SI_SRC_PLL_A (0 << 5) // MultiSynth 源自 PLLA
#define SI_SRC_PLL_B (1 << 5) // MultiSynth 源自 PLLB
#define SI_INT_MODE (1 << 6)  // 整数模式 (降低抖动)
#define SI_CLK_PDN (1 << 7)   // 掉电 (关闭该通道)

/* =========================================================================
  宏定义：PLL 复位 (Reg 177)
   ========================================================================= */
#define SI_PLL_RESET_A (1 << 5)
#define SI_PLL_RESET_B (1 << 7)

/* =========================================================================
   枚举与结构体
   ========================================================================= */
typedef enum
{
    SI_PLL_A = 0,
    SI_PLL_B
} si5351_pll_t;

typedef enum
{
    SI_R_DIV_1 = 0,
    SI_R_DIV_2 = 1,
    SI_R_DIV_4 = 2,
    SI_R_DIV_8 = 3,
    SI_R_DIV_16 = 4,
    SI_R_DIV_32 = 5,
    SI_R_DIV_64 = 6,
    SI_R_DIV_128 = 7,
} si5351_r_div_t;

//*********************************************************************************************************
/*Function announced*/
void Si5351_EnableOutputs(bool enabled);
void Si5351_Init(I2C_HandleTypeDef *hi2c);

#endif
