#include "motor_control.h"
#include "motor.h"



/**
 * @brief DQ 电压模式（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V
 */
void motor_set_dq_vlot(port_t portx, const uint8_t id, const float volt)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp = vol_float2int(volt, TINT16);

    motor_control_volt(fdcanHandle, id, temp);

}

/**
 * @brief DQ 电流模式（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A
 */
void motor_set_dq_current(port_t portx, const uint8_t id, const float cur)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp = cur_float2int(cur, TINT16);

    motor_control_cur(fdcanHandle, id, temp);

}

/**
 * @brief 位置模式，使用最大速度和加速度运动到目标位置（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos(port_t portx, const uint8_t id, const float pos)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float temp2 = pos_float2int(temp1, TINT16);

    motor_control_Pos(fdcanHandle, id, temp2, INT16_NAN);

}

/**
 * @brief 速度模式，以最大加速度加速到指定速度（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_vel(port_t portx, const uint8_t id, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float temp2 = vel_float2int(temp1, TINT16);

    motor_control_Vel(fdcanHandle, id, temp2, INT16_NAN);

}

/**
 * @brief 力矩模式（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_set_tqe(port_t portx, const uint8_t id, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float temp1 = tqe_adjust(tqe, motor_get_model2(portx, id));
    const float temp2 = tqe_float2int(temp1, TINT16);
    motor_control_tqe(fdcanHandle, id, temp2);

}

/**
 * @brief 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param type 通信协议的数据类型，影响数据的精度和量程（具体请参考FDCAN文档）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_set_pos_vel_MAXtqe(port_t portx, const uint8_t id,
                              const float pos, const float vel, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos1 = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel1 = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe1 = tqe_adjust(tqe, motor_get_model2(portx, id));
    const float pos2 = pos_float2int(pos1, TINT16);
    const float vel2 = vel_float2int(vel1, TINT16);
    const float tqe2 = tqe_float2int(tqe1, TINT16);

    motor_control_pos_val_tqe(fdcanHandle, id, pos2, vel2, tqe2);

}

/**
 * @brief 停止模式，电机三相都断开（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_set_stop(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_stop(fdcanHandle, id);
}

/**
 * @brief 刹车模式（阻尼模式），电机三相都接地（并让电机返回状态信息）
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_set_brake(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_brake(fdcanHandle, id);
}


/**
 * @brief 发送查询电机状态指令（电机会返回位置、速度、力矩）
 * @param id 电机 ID
 */
void motor_get_state_send(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    send_read_motor_state(fdcanHandle, id);
}


/**
 * @brief 发送查询电机固件版本号指令（在motor_process_state中解析）
 * @param fdcanHandle &hfdcanx
 * @param id 电机 ID
 */
void motor_get_version_send(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    // for (uint8_t i = 0; i < 5; i++)
    {
        send_read_motor_version(fdcanHandle, id);
    }
}

