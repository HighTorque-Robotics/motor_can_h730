#include "motor_control.h"
#include "motor.h"



/**
 * @brief DQ 电压模式（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V
 */
void motor_set_dq_vlot(port_t portx, const uint8_t id, const float volt)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float volt_raw = vol_float2int(volt, TINT16);

    motor_control_volt(fdcanHandle, id, volt_raw);

}

/**
 * @brief DQ 电流模式（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A
 */
void motor_set_dq_current(port_t portx, const uint8_t id, const float cur)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float cur_raw = cur_float2int(cur, TINT16);

    motor_control_cur(fdcanHandle, id, cur_raw);

}

/**
 * @brief 位置模式，使用最大速度和加速度运动到目标位置（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos(port_t portx, const uint8_t id, const float pos)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, TINT16);

    motor_control_pos(fdcanHandle, id, pos_raw, INT16_NAN);

}

/**
 * @brief 速度模式，以最大加速度加速到指定速度（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_vel(port_t portx, const uint8_t id, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float vel_raw = vel_float2int(vel_turns, TINT16);

    motor_control_vel(fdcanHandle, id, vel_raw, INT16_NAN);




}

/**
 * @brief 力矩模式（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_set_tqe(port_t portx, const uint8_t id, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float tqe_val_adjust = tqe_adjust(tqe, motor_get_model2(portx, id));
    const float tqe_raw = tqe_float2int(tqe_val_adjust, TINT16);

    motor_control_tqe(fdcanHandle, id, tqe_raw);

}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，不限制加速度和最大输出力矩（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_vel(port_t portx, const uint8_t id, const float pos, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, TINT16);
    const float vel_raw = vel_float2int(vel_turns, TINT16);

    motor_control_pos_vel_MAXtqe(fdcanHandle, id, pos_raw, vel_raw, INT16_NAN);
}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（Nm），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_set_pos_vel_MAXtqe(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe_val_adjust = tqe_adjust(tqe, motor_get_model2(portx, id));
    const float pos_raw = pos_float2int(pos_turns, TINT16);
    const float vel_raw = vel_float2int(vel_turns, TINT16);
    const float tqe_raw = tqe_float2int(tqe_val_adjust, TINT16);

    motor_control_pos_vel_MAXtqe(fdcanHandle, id, pos_raw, vel_raw, tqe_raw);
}


/**
 * @brief 位置、速度、加速度模式（梯形控制）（电机固件 v4.6.0 开始支持）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_vel_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const float pos_raw = pos_float2int(pos_turns, TINT16);
    const float vel_raw = vel_float2int(vel_turns, TINT16);
    const float acc_raw = acc_float2int(acc_turns, TINT16);

    motor_control_pos_vel_acc(fdcanHandle, id, pos_raw, vel_raw, acc_raw);
}


/**
 * @brief 运控模式 (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩)（电机固件 v4.6.0 开始支持）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（Nm），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 * @param kp 单位可为牛米每转（Nm/rev）、牛米每弧度（Nm/rad）、或牛米每度（Nm/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param kd 单位可为牛米秒每转（Nm·s/rev）、牛米秒每弧度（Nm·s/rad）、或牛米秒每度（Nm·s/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_set_pos_vel_tqe_kp_kd(port_t portx, const uint8_t id, const float pos, const float vel, float tqe, float kp, float kd)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const motor_type_t model = motor_get_model2(portx, id);

    /* 将单位转换成转 */
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float kp_turns = conv_from_turns(kp, MOTOR_DATA_TYPE_FLAG);
    const float kd_turns = conv_from_turns(kd, MOTOR_DATA_TYPE_FLAG);

    /* 力矩修正 */
    const float tqe_val_adjust = tqe_adjust(tqe, model);
    const float kp_val_adjust = pid_adjust(kp_turns, model);
    const float kd_val_adjust = pid_adjust(kd_turns, model);

    /* float -> int */
    const uint16_t pos_raw = mit_float2int(pos_turns, -3.2768f, 3.2767f, 16);
    const uint16_t vel_raw = mit_float2int(vel_turns, -2.0f, 2.0f, 12);
    const uint16_t tqe_raw = mit_float2int(tqe_val_adjust, -10.0f, 10.0f, 12);
    const uint16_t kp_raw = mit_float2int(kp_val_adjust, -400, 400, 12);
    const uint16_t kd_raw = mit_float2int(kd_val_adjust, -100, 100, 12);

    motor_control_pos_vel_tqe_kp_kd(fdcanHandle, id, pos_raw, vel_raw, tqe_raw, kp_raw, kd_raw);
}


/**
 * @brief 停止模式，电机三相都断开（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_set_stop(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_stop(fdcanHandle, id);
}

/**
 * @brief 刹车模式（阻尼模式），电机三相都接地（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_set_brake(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    set_motor_brake(fdcanHandle, id);
}


/**
 * @brief 发送查询查询电机状态信息的指令（在motor_process_state中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_get_state_send(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    send_read_motor_state(fdcanHandle, id);
}


/**
 * @brief 发送查询电机固件版本号指令（在motor_process_state中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
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


/**
 * @brief 周期返回电机位置、速度、力矩数据(返回数据格式和使用 0x17，0x01 指令获取的格式一样)
 * @param id 电机ID
 * @param t 返回周期（单位：ms），0--关闭定时返回电机状态，重启电机也会终止返回。
 */
void motor_set_timed_return_status_send(port_t portx, const uint8_t id, int16_t t_ms)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    timed_return_motor_status(fdcanHandle, id, t_ms);
}

