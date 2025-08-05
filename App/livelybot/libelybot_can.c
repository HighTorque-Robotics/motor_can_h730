#include "libelybot_can.h"
#include "fdcan.h"
#include <string.h>


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



/**
 * @brief DQ电压控制
 * @param id 电机ID
 * @param vol Q相电压，单位：0.1v，如 vol = 10 表示 Q 相电压为 1V
 */
void motor_control_volt(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t vol)
{
    static uint8_t tdata[7] = {0x01, 0x00, 0x08, 0x05, 0x1b, 0x00, 0x00};

    *(int16_t *)&tdata[5] = vol;

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief DQ电流控制
 * @param id 电机ID
 * @param cur Q相电流，单位：0.1A，如 cur = 10 表示 Q 相电压为 1A
 */
void motor_control_cur(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t cur)
{
    static uint8_t tdata[7] = {0x01, 0x00, 0x09, 0x05, 0x1c, 0x00, 0x00};

    *(int16_t *)&tdata[5] = cur;

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 位置控制
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param tqe：最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM，不想控制力矩建议给值 0x8000 （表示无限制）
 */
void motor_control_Pos(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int32_t pos, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x0A, 0x05, 0x00, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = pos;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hfdcanx, id, tdata, 8);
}


/**
 * @brief 速度控制
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM，不想控制力矩建议给值 0x8000 （表示无限制）
 */
void motor_control_Vel(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t vel, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[4] = vel;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hfdcanx, id, tdata, 8);
}


/**
 * @brief 力矩模式
 * @param id 电机ID
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示力矩为 1.1NM
 */
void motor_control_tqe(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int32_t tqe)
{
    uint8_t tdata[8] = {0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = tqe;

    can_send(hfdcanx, id, tdata, 4);
}


/**
 * @brief 电机位置-速度-前馈力矩(最大力矩)控制，int16型
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param val 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM，不想控制力矩建议给值 0x8000 （表示无限制）
 */
void motor_control_pos_val_tqe(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t val, int16_t tqe)
{
    static uint8_t tdata[8] = {0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    *(int16_t *)&tdata[2] = val;
    *(int16_t *)&tdata[4] = tqe;
    *(int16_t *)&tdata[6] = pos;

    can_send(hfdcanx, id, tdata, 8);
}


/**
 * @brief 将当前位置设为电机零位(此指令只是在 RAM 中修改，还需配合 `conf write` 指令保存到 flash 中)
 * @param id 电机ID
 */
void rezero_pos(FDCAN_HandleTypeDef *hfdcanx, uint8_t id)
{
    uint8_t tdata[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0a};

    can_send(hfdcanx, 0x8000 | id, tdata, sizeof(tdata));
    HAL_Delay(1000);  // 建议延时1s

    conf_write(hfdcanx, id);  // 保存设置
}


/**
 * @brief 将电机 RAM 中设置保存到 flash 中(使用此指令后建议给电机重新上电)
 * @param id 电机ID
 */
void conf_write(FDCAN_HandleTypeDef *hfdcanx, uint8_t id)
{
    uint8_t tdata[] = {0x05, 0xb3, 0x02, 0x00, 0x00};

    can_send(hfdcanx, 0x8000 | id, tdata, sizeof(tdata));
}


/**
 * @brief 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样)
 * @param id 电机ID
 * @param t 返回周期（单位：ms）
 */
void timed_return_motor_status(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t t_ms)
{
    uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&tdata[3] = t_ms;

    can_send(hfdcanx, 0x8000 | id, tdata, sizeof(tdata));
}


/**
 * @brief 电机停止，注意：需让电机停止后再重置零位，否则无效
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x00};

    can_send(fdcanHandle, id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车
 * @param fdcanHandle &hfdcanx
 * @param motor id 电机ID
 */
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    static uint8_t cmd[] = {0x01, 0x00, 0x0f};

    can_send(fdcanHandle, id, cmd, sizeof(cmd));
}


/**
 * @brief 读取电机位置、速度、力矩指令
 * @param id 电机ID
 */
void motor_read(FDCAN_HandleTypeDef *hfdcanx, uint8_t id)
{
    static uint8_t tdata[8] = {0x17, 0x01};

    can_send(hfdcanx, 0x8000 | id, tdata, sizeof(tdata));
}


static uint8_t Fdcan_Dlc_To_Len(uint32_t dlc)
{
    uint8_t len = 0;
    uint8_t tab_dlc_to_len[] = {12, 16, 20, 24, 32, 48, 64};
    const uint16_t shift = __builtin_ctz(FDCAN_DLC_BYTES_1);

    if (dlc <= FDCAN_DLC_BYTES_8)
    {
        len = dlc >> shift;
    }
    else
    {
        len = tab_dlc_to_len[(dlc >> shift) - 9];
    }

    return len;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)  // FDCAN FIFO 0 回调函数
{
    uint8_t len = 0;
    if(hfdcan->Instance == FDCAN1 || hfdcan->Instance == FDCAN2 || hfdcan->Instance == FDCAN3)  // 这里就不管是哪个can通道的都接受了
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_rx_header1, fdcan1_rdata);
        if (fdcan_rx_header1.DataLength != 0)
        {
            len = Fdcan_Dlc_To_Len(fdcan_rx_header1.DataLength);
            motor_state.motor.id = fdcan_rx_header1.Identifier;  // 获取电机 id

            memcpy(&motor_state.data[4], &fdcan1_rdata[2], len - 2);  // 获取电机状态数据，协议
			
//            motor_state.motor.position = *(int16_t *)&fdcan1_rdata[2];  // 这里和上面直接用memcpy函数的效果是相同的
//            motor_state.motor.velocity = *(int16_t *)&fdcan1_rdata[4];
//            motor_state.motor.torque = *(int16_t *)&fdcan1_rdata[6];
            motor_read_flag = 1;
        }
    }
}

