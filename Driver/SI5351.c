#include "header.h"
//*********************************************************************************************************
static I2C_HandleTypeDef *_si5351_hi2c;
//*********************************************************************************************************
/*内部辅助函数*/

/**
 * @brief SI5351 I2C写入函数
 * @note  内部辅助函数：写寄存器
 */
static void Si5351_Write8(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(_si5351_hi2c, SI5351_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/**
 * @brief SI5351 I2C读取函数
 * @note  内部辅助函数：读寄存器
 */
static uint8_t Si5351_Read8(uint8_t reg)
{
	uint8_t value = 0;
	HAL_I2C_Mem_Read(_si5351_hi2c, SI5351_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
	return value;
}
//*********************************************************************************************************

/**
 * @brief 启用或禁用所有输出通道
 * @param enabled: true=开启, false=禁用(高阻态或拉低，取决于配置)
 */
void Si5351_EnableOutputs(bool enabled)
{
	// Reg 3: 0 = Enable, 1 = Disable
	Si5351_Write8(SI_REG_OUTPUT_ENABLE, enabled ? 0x00 : 0xFF);
}

/**
 * @brief 初始化 Si5351，复位状态并配置晶振负载
 */
void Si5351_Init(I2C_HandleTypeDef *hi2c)
{
	_si5351_hi2c = hi2c;

	// 1. 禁用所有输出 (Reg 3 = 0xFF)
	Si5351_EnableOutputs(false);

	// 2. 将常用通道设为掉电状态，防止意外输出
	// 使用宏定义：设置为掉电模式
	uint8_t pdn_config = SI_CLK_PDN;
	Si5351_Write8(SI_REG_CLK0_CTRL, pdn_config);
	Si5351_Write8(SI_REG_CLK1_CTRL, pdn_config);
	Si5351_Write8(SI_REG_CLK2_CTRL, pdn_config);
	Si5351_Write8(SI_REG_CLK3_CTRL, pdn_config);
	Si5351_Write8(SI_REG_CLK4_CTRL, pdn_config);
	Si5351_Write8(SI_REG_CLK5_CTRL, pdn_config);

	// 3. 设置晶振内部负载电容 (Reg 183)
	// 默认使用 10pF，适配大多数市面上的模块
	Si5351_Write8(SI_REG_CRYSTAL_LOAD, SI_XTAL_10PF);

	// 4. 复位 PLL A 和 PLL B (Reg 177)
	Si5351_Write8(SI_REG_PLL_RESET, SI_PLL_RESET_A | SI_PLL_RESET_B);
}
