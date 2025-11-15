#include "header.h"
//*********************************************************************************************************
extern UART_HandleTypeDef huart1; // HMI 控制串口
extern UART_HandleTypeDef huart2; // printf 调试串口
//*********************************************************************************************************
static uint8_t hmi_rx_buffer[50]; // HMI 接收缓冲区
static uint8_t hmi_cmd_flag = 0;  // 新指令标志位 (0 = false)
static uint16_t hmi_cmd_size = 0; // 新指令长度
//*********************************************************************************************************
/**
 * @name    HMI_Process_Init()
 * @brief   启动HMI的处理逻辑
 * @note    启动第一次DMA接收
 * @param   无
 */
void HMI_Process_Init(void)
{
    // 启动HMI串口(USART1)的空闲中断DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hmi_rx_buffer, sizeof(hmi_rx_buffer));
}

/**
 * @name    HMI_Send_Cmd()
 * @brief   向 HMI 串口屏 (USART1) 发送原始指令 (带 0xFF 结尾)
 * @note    可以用于比如切换页面（page pagenam）
 * @param   *cmd_string：指令（类型为char*）
 */
void HMI_Send_Cmd(const char *cmd_string)
{
    char cmd_buffer[100]; // 缓冲区
    int len = snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\xff\xff\xff", cmd_string);

    // 使用阻塞式发送
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd_buffer, len, HAL_MAX_DELAY);
}

/**
 * @name    HMI_Process_Task_Poll()
 * @brief   HMI处理任务轮询循环
 * @note    放在while(1)循环中，接受串口指令，解析并调用功能函数完成任务
 * @param   无
 */
void HMI_Process_Task_Poll(void)
{
    if (hmi_cmd_flag == 1) // 检测到接收状态位为一，说明有串口指令
    {
        // 1、开始解析串口指令
        // printf("HMI Recv: %.*s\r\n", hmi_cmd_size, hmi_rx_buffer)//用于USART2查看是否接收到指令
        //--------------------------------------------------------------------------------------
        if (hmi_rx_buffer[0] == 'B') // 基础部分第二问
        {
            char *p = (char *)hmi_rx_buffer; // 指针p指向hmi_rx_buffer[0]
            if (p)
            {
                float f_E = atof(p + 1); // 跳过'B',频率
                Func_Basic2_SetSignal(f_E);
                /*USART2串口通信*/
                // sprintf();
                // HAL_UART_Transmit();
            }
        }
        else if (hmi_rx_buffer[0] == 'A') // 基础部分第三，四问
        {
            char *p = strchr((char *)hmi_rx_buffer, ','); // p作为指针，利用strchr()指向第一个‘,’位置
            if (p)
            {
                *p = '\0';
                float Ux0 = atof((char *)hmi_rx_buffer + 1); // 跳过'B',指定输出电压值
                float f_E = atof(p + 1);                     // 频率
                Func_Basic3_4_SetSignal(f_E, Ux0);
            }
        }

        else if (hmi_rx_buffer[0] == 'C') // 发挥部分第一问
        {
            FilterType T = Fun_Advanced1_StratLearning();
            /*判断滤波类型并在串口屏展示*/
            switch (T)
            {
                /* code */
            case FILTER_HIGH_PASS:
                HMI_Send_Cmd("page page3");
                break;
            case FILTER_LOW_PASS:
                HMI_Send_Cmd("page page4");
                break;
            case FILTER_BAND_PASS:
                HMI_Send_Cmd("page page5");
                break;
            case FILTER_BAND_STOP:
                HMI_Send_Cmd("page page6");
                break;
            }
        }
        else if (hmi_rx_buffer[0] == 'D') // 发挥部分第二问
        {
        }
        else if (hmi_rx_buffer[0] == 'E') // 清零,g
        {
            Func_Stop_All_Output();
        }
        hmi_cmd_flag = 0; // 重新开始接收
    }
}
//*********************************************************************************************************
/**
 * @brief 重定向 printf (到 USART2 调试串口)
 */
int fputc(int c, FILE *stream)
{
    uint8_t ch[1] = {c};
    // 发送到 huart2 !!
    HAL_UART_Transmit(&huart2, ch, 1, 0xFFFF);
    return c;
}

/**
 * @brief 重新定义USART中断回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) // 检测目前串口是否是USART1，即串口屏
    {
        if (hmi_cmd_flag == 0) // 上一轮的接收是否已经完成（完成后flag会置零）
        {
            hmi_cmd_flag = 1;    // 重新开始接收下一轮指令（flag重新置一，说明正在接收）
            hmi_cmd_size = Size; // 保存指令长度（用于解析指令）
        }
        //
        HAL_UARTEx_ReceiveToIdle_DMA(huart, hmi_rx_buffer, sizeof(hmi_rx_buffer));
    }
}
