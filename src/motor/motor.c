#include "motor.h"
#include <stdio.h>



/************************************下面为需要修改的部分*******************************************/

static motor_state_s motor_state_port[MOTOR_PORT_NUM][MOTOR_MAX_NUM];


const port_mapping_s port_maping[MOTOR_PORT_NUM] =  // 通道映射表
{
    {
        .port = PORT1,
        .fdcan = &hfdcan1,
        .state = motor_state_port[0],
    },

    // {
    //     .port = PORT2,
    //     .fdcan = &hcan2,
    //     .state = motor_state_port[1],
    // },
};

/*******************************************END***************************************************/


p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (fdcanHandle->Instance == port_maping[i].fdcan->Instance)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


p_motor_state_s motor_get_state_pointer2(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].state;
        }
    }

    MOTOR_ERR();
    return NULL;
}


FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx)
{
    for (uint8_t i = 0; i < MOTOR_PORT_NUM; i++)
    {
        if (portx == port_maping[i].port)
        {
            return port_maping[i].fdcan;
        }
    }

    MOTOR_ERR();
    return NULL;
}



void motor_print_state()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            motor_state_s *p_motor_state = motor_get_state(portx, id);
            printf("PORT: %d, ID: %2d, mode: %2d, temp: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", portx, id, p_motor_state->mode, p_motor_state->temp,
                   p_motor_state->fault, p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
        }
        printf("\r\n");
    }
}


void motor_print_version()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            const p_version_s p_version = &(motor_get_state(portx, id)->version);

            printf("PORT: %d, ID: %2d, version = %d.%d.%d\r\n", portx, id, p_version->major, p_version->minor, p_version->patch);
        }
        printf("\r\n");
    }
}



/**
 * @brief 获取指定端口和ID的电机状态指针
 * @param portx 指定电机所在的端口，可能的值为 PORT1 或 PORT2
 * @param id 电机 ID
 * @return 返回类型为 `p_motor_state_s` 的指针
 */
p_motor_state_s motor_get_state(port_t portx, uint8_t id)
{
    const uint8_t index = id - 1;

    return &(motor_get_state_pointer2(portx)[index]);
}


/**
 * @brief 解析电机返回信息
 * @param fdcanHandle
 * @param id 电机 ID
 * @param id_type CAN ID bits[17:16] 数据类型 (TINT16/TINT32/TFLOAT)
 * @param p_data can 帧数据指针
 * @param len can 数据长度
 */
static void motor_process_state(FDCAN_HandleTypeDef *fdcanHandle, const uint8_t id, const uint32_t id_type, const uint8_t *p_data, const uint8_t len)
{
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);
    const uint8_t id_index = id - 1;

    switch (p_data[0])
    {
    // ===================== FLAUT_POS_VEL_TQE (0x0E) 响应 =====================
    // 返回帧: 查询码(0x0E) | 错误码 | 位置 | 速度 | 力矩, 无模式字段
    // 字段宽度由 CAN ID 类型位决定: TINT16=2B
    case FLAUT_POS_VEL_TQE:
    {
        switch (id_type)
        {
        case TINT16:
        {
            int16_t pos = 0, vel = 0, tqe = 0;

            my_memcpy((uint8_t *)&pos, p_data + 2, sizeof(int16_t));
            my_memcpy((uint8_t *)&vel, p_data + 4, sizeof(int16_t));
            my_memcpy((uint8_t *)&tqe, p_data + 6, sizeof(int16_t));

            p_motor_state[id_index].fault     = p_data[1];
            p_motor_state[id_index].position  = conv_from_turns(pos_int2float(pos, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].velocity  = conv_from_turns(vel_int2float(vel, TINT16), MOTOR_DATA_TYPE_FLAG);
            p_motor_state[id_index].torque    = tqe_int2float(tqe, TINT16);
            break;
        }
        default:
            break;
        }
        break;
    }
    // ===================== 电机固件版本 (0x04) =====================
    // 返回帧: 04 | patch | minor | major (各1字节)
    case FW_VERSION:
    {
        p_motor_state[id_index].version.major = p_data[3];
        p_motor_state[id_index].version.minor = p_data[2];
        p_motor_state[id_index].version.patch = p_data[1];
        break;
    }
    // ===================== 电机硬件版本 (0x05) =====================
    // 返回帧: 05 | patch | minor | major (各1字节)
    case HW_VERSION:
    {
        p_motor_state[id_index].hw_version.major = p_data[3];
        p_motor_state[id_index].hw_version.minor = p_data[2];
        p_motor_state[id_index].hw_version.patch = p_data[1];
        break;
    }
    // ===================== 电机型号查询响应 =====================
    case MODEL:
    {
        const uint8_t model_len = p_data[1];

        if (model_len > 0 && model_len <= 15 && len >= model_len + 2)
        {
            char model_str[25] = {0};

            // 型号数据为 ASCII 字符直读 (如 0x35='5', 0x5F='_'), 直接复制即可
            for (uint8_t i = 0; i < model_len; i++)
            {
                model_str[i] = (char)p_data[2 + i];
            }

            // 保存电机型号到状态结构体
            my_memcpy((uint8_t *)p_motor_state[id_index].model, (uint8_t *)model_str, sizeof(model_str));
        }
        break;
    }
    case SYSTEM:
    {
        if (len >= 2)
        {
            const uint8_t result = p_data[1];

            //  result == 0 表示成功 (03 00), 非 0 表示失败 (03 XX)
            // motor_config_closed_loop 以"非 0"视为确认成功
            // 成功 → ack = 1(非零); 失败 → ack = 0
            p_motor_state[id_index].ack = (result == 0) ? 1 : 0;
        }
        break;
    }
    default:
        break;
    }
}




static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8] = {0};

/**
 * @brief 解析所有 CAN 通道 FIFO 中的电机状态数据
 *
 */
void motor_process_state_all()
{
    for (int i = 0; i < MOTOR_PORT_NUM; i++)
    {
        while (HAL_FDCAN_GetRxMessage(port_maping[i].fdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            if (rx_header.Identifier != FDCAN_EXTENDED_ID)  // 电机返回帧为 29 位扩展帧, 标准帧直接丢弃
            {
                continue;
            }

            const uint16_t len = get_fdcan_data_size(rx_header.DataLength);

            if (len != 0)
            {
                const uint32_t id_type  = (rx_header.Identifier >> 16) & 0x3;   // 提取 bits[17:16] 数据类型, 得 0~2 对应 data_type_t (TINT16_NOHDR=0, TINT16=1)
                const uint8_t  motor_id = (rx_header.Identifier >> 8) & 0x7F;   // 提取 bits[14:8] 主机ID (电机返回ID, 1~127)
                const uint8_t  dir      = (rx_header.Identifier >> 15) & 0x1;   // 提取 bit[15] 帧方向: 1=控制帧, 0=返回帧

                /* 帧方向判断: bit[15]=1 为控制(发送)帧, 非返回帧直接丢弃 */
                if (dir != 0)
                {
                    continue;
                }

                if (motor_id > 0 && motor_id <= MOTOR_MAX_NUM)
                {
                    motor_process_state(port_maping[i].fdcan, motor_id, id_type, rx_data, len);
                }
            }
        }
    }
}


/**
 * @brief CAN 接收 FIFO0 中断回调 (主循环轮询之外的中断兜底)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
    motor_process_state_all();
}
