#include "my_can.h"



FDCAN_TxHeaderTypeDef TxHeader =
{
    .TxFrameType = FDCAN_DATA_FRAME,            // 数据帧
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,    // 错误指示状态
    .BitRateSwitch = FDCAN_BRS_OFF,             // 比特率切换关闭
    .FDFormat = FDCAN_CLASSIC_CAN,              // 经典 CAN 格式
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,   // 不使用发送事件 FIFO
    .MessageMarker = 0,                         // 消息标记
};


uint32_t can_size2dlc(uint16_t size)
{
    if (size > 8)
    {
        size = 8;
    }
    return size;
}

uint16_t can_dlc2size(uint32_t dlc)
{
    if (dlc > 8)
    {
        return 8; // 最大8字节
    }
    return (uint16_t)dlc;
}


void can_filter_init(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (HAL_FDCAN_ConfigGlobalFilter(fdcanHandle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(fdcanHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_FDCAN_ConfigTxDelayCompensation(fdcanHandle, fdcanHandle->Init.DataPrescaler * fdcanHandle->Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(fdcanHandle);

    if (HAL_FDCAN_Start(fdcanHandle) != HAL_OK)
    {
        Error_Handler();
    }
}


void can_send(FDCAN_HandleTypeDef *hfdcanx, uint32_t id, uint8_t *data, uint8_t len)
{
    TxHeader.Identifier = id;

    if(id > 0x7ff)
    {
        TxHeader.IdType = FDCAN_EXTENDED_ID;
    }
    else
    {

        TxHeader.IdType = FDCAN_STANDARD_ID;
    }

    TxHeader.DataLength = can_size2dlc(len);
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &TxHeader, data);
}


