#include "motor_config.h"


/**
 * @brief 配置类指令闭环确认辅助 (清零 → 循环重发 → 等待确认字段非 0)
 * @param action 每次循环重发的动作, 填 NULL 表示不重发、仅等待返回帧
 * @param fdcanHandle &hcanx
 * @param id 电机 ID (状态数组下标 +1)
 * @param p_flag 待确认字段指针 (如 &state.ack), 非 0 视为确认成功
 * @return 0-确认成功, 1-超时未确认
 * @note 调用前须保证 id 在 1~MOTOR_MAX_NUM 内, 否则会越界访问状态数组
 */
static uint8_t motor_config_closed_loop(void (*action)(FDCAN_HandleTypeDef *, uint8_t),
                                        FDCAN_HandleTypeDef *fdcanHandle,
                                        const uint8_t id,
                                        uint8_t *p_flag)
{
    uint8_t flag = 1;

    *p_flag = 0;
    for (int i = 0; i < 10; i++)
    {
        if (action != NULL)
        {
            action(fdcanHandle, id);
        }
        HAL_Delay(50);
        motor_process_state_all();
        if (*p_flag != 0)
        {
            flag = 0;
            break;
        }
    }

    return flag;
}


/**
 * @brief 重置电机零位
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @return 0-成功，1-重置零位失败
 */
uint8_t motor_pos_reset(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);

    hightorque_reset(fdcanHandle, id);
    HAL_Delay(200);

    if (motor_config_closed_loop(hightorque_pos_rezero, fdcanHandle, id, &p_motor_state[id - 1].ack) != 0)
    {
        return 1;
    }

    hightorque_reset(fdcanHandle, id);
    HAL_Delay(200);

    return 0;
}

/**
 * @brief 更改电机ID
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param old_id 当前电机 ID
 * @param new_id 新电机 ID
 * @return 0-成功，1-修改ID失败
 * @note 修改ID 记得修改 motor.h 的电机数量, 使用时 ID 不能超过这个数量
 */
uint8_t motor_set_id(port_t portx, const uint8_t old_id, const uint8_t new_id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);

    hightorque_id(fdcanHandle, old_id, new_id);
    HAL_Delay(100);

    if (motor_config_closed_loop(NULL, fdcanHandle, old_id, &p_motor_state[old_id - 1].ack) != 0)
    {
        return 1;
    }

    hightorque_reset(fdcanHandle, new_id);
    HAL_Delay(200);

    return 0;
}


/**
 * @brief 设置电机周期返回状态
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param t_us 周期时间, 单位: 1us (4字节小端); 填 0 停止周期返回; 小于 100us 电机会报错 03 01
 * @return 0-成功，1-设置失败
 * @note 周期返回是运行时功能, 末尾不做软重启 (重启会打断/清除该设置),
 *       这点与 motor_pos_reset / motor_set_id 不同
 */
uint8_t motor_timed_return_status(port_t portx, const uint8_t id, const uint32_t t_us)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    p_motor_state_s p_motor_state = motor_get_state_pointer1(fdcanHandle);

    /* id 超出本工程管理的电机数量时, 返回帧不会被解析 (见 motor.c), 且会越界访问状态数组 */
    if (id < MOTOR_ID_MIN || id > MOTOR_MAX_NUM)
    {
        MOTOR_ERR();
        return 1;
    }

    hightorque_request_timed_return(fdcanHandle, id, t_us);
    HAL_Delay(100);


    if (motor_config_closed_loop(NULL, fdcanHandle, id, &p_motor_state[id - 1].ack) != 0)
    {
        return 1;
    }

    return 0;
}

