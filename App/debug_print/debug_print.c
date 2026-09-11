#include <stdio.h>
#include <stdarg.h>
#include <debug_print.h>
#include <string.h>

#include "usart.h"


static float data[10] = {0};
const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};


//该函数用于通过UART发送调试数据(适配于VOFA+的JustFloat协议)
void debug_print(int num, ...)
{
    if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
    {
        return;
    }

    va_list args;
    va_start(args, num);

    for (int i = 0; i < num; i++)
    {
        data[i] = (float)va_arg(args, double);
    }

    data[num] = *(float *)tail;

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data,
                          (uint16_t)((num + 1) * sizeof(float)));

    va_end(args);
}