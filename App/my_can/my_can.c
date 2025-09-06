#include "my_can.h"

FDCAN_RxHeaderTypeDef fdcan_rx_header1;
uint8_t fdcan1_rdata[24] = {0};

motor_state_t motor_state;
uint8_t motor_read_flag = 0;

uint8_t can_send(FDCAN_HandleTypeDef *hfdcanx, uint32_t id, uint8_t *msg, uint8_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};

    TxHeader.Identifier = id;	//设置扩展ID
    TxHeader.IdType = FDCAN_EXTENDED_ID; 	//使用扩展ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; //数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; //数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;// 错误指示状态
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; //比特率切换关闭，不适用于经典CAN
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; //经典CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 不适用发送事件FIFO
    TxHeader.MessageMarker = 0; //消息标记


    //复制数据到发送缓冲区
    for(int i = 0; i < len; i++)
    {
        TxData[i] = msg[i];
    }

    // 发送CAN指令
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &TxHeader, TxData) != HAL_OK)
    {
        // 发送失败处理
        Error_Handler();
        return 1; // 返回非零值以表示发送失败
    }
    return 0; // 发送成功
}

void fdcan_filter_init(FDCAN_HandleTypeDef *fdcanHandle)
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

    //HAL_FDCAN_Start(fdcanHandle);
}

uint8_t Fdcan_Dlc_To_Len(uint32_t size)
{
    uint32_t fdcan_dlc = 0;

    if(size == 0)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_0;
    }
    else if(size <= 1)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_1;
    }
    else if(size <= 2)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_2;
    }
    else if(size <= 3)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_3;
    }
    else if(size <= 4)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_4;
    }
    else if(size <= 5)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_5;
    }
    else if(size <= 6)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_6;
    }
    else if(size <= 7)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_7;
    }
    else if(size <= 8)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_8;
    }
    else if(size <= 12)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_12;
    }
    else if(size <= 16)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_16;
    }
    else if(size <= 20)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_20;
    }
    else if(size <= 24)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_24;
    }
    else if(size <= 32)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_32;
    }
    else if(size <= 48)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_48;
    }
    else if(size <= 64)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_64;
    }
    return fdcan_dlc;
}
