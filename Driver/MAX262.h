#ifndef __MAX262_H
#define __MAX262_H

#include "main.h"
// 定义 PI
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
//***************************
//		Pin assign
//			STM32			继电器
//		GPIOD_Pin_1 		---> 带通通道
//		GPIOD_Pin_2 		---> 高通/带阻通道
//		GPIOD_Pin_3		    ---> 低通通道
//**************************

//*********************************************************************************************************
// 宏定义（移植HAL库）
//		Pin assign
//			STM32			MAX262
//		GPIOG_Pin_8 		---> D0
//		GPIOG_Pin_9	     	---> D1
//		GPIOG_Pin_10		---> A0
//		GPIOG_Pin_11		---> A1
//      GPIOG_Pin_12        ---> A2
//      GPIOG_Pin_13        ---> A3
//
//      GPIOE_Pin_14        ---> LE
//      GPIOE_Pin_15        ---> WR
//
//*********************************************************************************************************
/**
 * <MAX262引脚映射>
 * DATA:D0,D1
 * ADRESS:A0,A1,A2,A3
 * CONTROL:WR-->PE14,LE--->PE15
 * */
#define BUS_PORT GPIOG
#define CTRL_PORT GPIOE

#define D0_PIN GPIO_PIN_8
#define D1_PIN GPIO_PIN_9

#define A0_PIN GPIO_PIN_10
#define A1_PIN GPIO_PIN_11
#define A2_PIN GPIO_PIN_12
#define A3_PIN GPIO_PIN_13

#define LE_PIN GPIO_PIN_14
#define WR_PIN GPIO_PIN_15
/*写入和使能*/
#define WR_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET)
#define WR_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET)

#define LE_L HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET)
#define LE_H HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET)

// ------------------------------------------------
// 定时器配置
// ------------------------------------------------
extern TIM_HandleTypeDef htim1; // TIM1 CH1
#define PWM_TIM &htim1
#define PWM_CH TIM_CHANNEL_1
// F407:APB2 Timer Clock 168MHz
#define TIM_CLOCK 168000000
// ------------------------------------------------
// 接口声明
// ------------------------------------------------
void MAX262_Init(void);
void MAX262_Config(uint8_t filter_id, uint8_t mode, float q, float fc);
// void Filter1(uint8_t mode, float q);
// void MAX262_CLK_SET_PWM(float fc);
//*********************************************************************************************************

#endif
