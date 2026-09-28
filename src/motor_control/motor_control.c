#include "motor_control.h"
#include "motor.h"



/**
 * @brief DQ 电压模式（并让电机返回状态信息）— d轴=0, q轴=实际电压
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V（D 轴固定为 0）
 */
void motor_dq_volt(port_t portx, const uint8_t id, const float volt)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const int16_t volt_raw = (int16_t)vol_float2int(volt, TINT16);

    hightorque_dq_volt_int16(fdcanHandle, id, 0, volt_raw);
}


/**
 * @brief DQ 电流模式（并让电机返回状态信息）— d轴=0, q轴=实际电流
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A（D 轴固定为 0）
 */
void motor_dq_current(port_t portx, const uint8_t id, const float cur)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const int16_t cur_raw = (int16_t)cur_float2int(cur, TINT16);

    hightorque_dq_current_int16(fdcanHandle, id, 0, cur_raw);
}


/**
 * @brief 位置模式，使用最大速度和加速度运动到目标位置（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_pos(port_t portx, const uint8_t id, const float pos)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_raw = (int16_t)pos_float2int(pos_turns, TINT16);

    hightorque_pos_int16(fdcanHandle, id, pos_raw);
}


/**
 * @brief 速度模式，以最大加速度加速到指定速度（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_vel(port_t portx, const uint8_t id, const float vel)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_raw = (int16_t)vel_float2int(vel_turns, TINT16);

    hightorque_vel_int16(fdcanHandle, id, vel_raw);
}


/**
 * @brief 力矩模式（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（Nm）
 */
void motor_tqe(port_t portx, const uint8_t id, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const int16_t tqe_raw = (int16_t)tqe_float2int(tqe, TINT16);

    hightorque_torque_int16(fdcanHandle, id, tqe_raw);
}


/**
 * @brief 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（Nm）
 */
void motor_pos_vel_MAXtqe(port_t portx, const uint8_t id,
                          const float pos, const float vel, const float tqe)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_raw = (int16_t)pos_float2int(pos_turns, TINT16);
    const int16_t vel_raw = (int16_t)vel_float2int(vel_turns, TINT16);
    const int16_t tqe_raw = (int16_t)tqe_float2int(tqe, TINT16);

    hightorque_pos_vel_tqe_int16(fdcanHandle, id, pos_raw, vel_raw, tqe_raw);
}


/**
 * @brief 位置、速度、加速度模式（梯形控制）（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_pos_velmax_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_raw = (int16_t)pos_float2int(pos_turns, TINT16);
    const int16_t vel_raw = (int16_t)vel_float2int(vel_turns, TINT16);
    const int16_t acc_raw = (int16_t)acc_float2int(acc_turns, TINT16);

    hightorque_pos_vel_acc_int16(fdcanHandle, id, pos_raw, vel_raw, acc_raw);
}


/**
 * @brief 速度、加速度模式，以目标加速度加速到目标速度（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转每秒平方（rev/s^2）、弧度每秒平方（rad/s^2）、或度每秒平方（°/s^2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_vel_acc(port_t portx, const uint8_t id, const float vel, const float acc)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_raw = (int16_t)vel_float2int(vel_turns, TINT16);
    const int16_t acc_raw = (int16_t)acc_float2int(acc_turns, TINT16);

    hightorque_vel_acc_int16(fdcanHandle, id, vel_raw, acc_raw);
}


/**
 * @brief MIT模式 (输出力矩 = 位置偏差 * kp + 速度偏差 * kd + 前馈力矩)（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 * @param pos 位置，单位可为转（rev）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转每秒（rps）、弧度每秒（rad/s）、或度每秒（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（Nm）
 * @param kp 单位可为牛米每转（Nm/rev）、牛米每弧度（Nm/rad）、或牛米每度（Nm/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param kd 单位可为牛米秒每转（Nm·s/rev）、牛米秒每弧度（Nm·s/rad）、或牛米秒每度（Nm·s/°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_pos_vel_tqe_kp_kd(port_t portx, const uint8_t id,
                             const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    /* 单位转换成转 */
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float kp_turns = conv_from_turns(kp, MOTOR_DATA_TYPE_FLAG);
    const float kd_turns = conv_from_turns(kd, MOTOR_DATA_TYPE_FLAG);


    const int16_t pos_raw = (int16_t)mit_pos_float2uint(pos_turns);
    const int16_t vel_raw = (int16_t)mit_vel_float2uint(vel_turns);
    const int16_t tqe_raw = (int16_t)mit_tqe_float2uint(tqe);
    const int16_t kp_raw = (int16_t)mit_kp_float2uint(kp_turns);
    const int16_t kd_raw = (int16_t)mit_kd_float2uint(kd_turns);

    hightorque_pos_vel_tqe_kp_kd_int16(fdcanHandle, id, pos_raw, vel_raw, tqe_raw, kp_raw, kd_raw);
}


/**
 * @brief 发送查询电机状态信息的指令（在motor_process_state中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_request_state(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);
    hightorque_request_state_int16(fdcanHandle, id);
}


/**
 * @brief 发送查询电机固件版本号指令（在motor_process_state中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_request_fw_version(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    hightorque_request_fw_version(fdcanHandle, id);
}


/**
 * @brief 发送查询电机型号指令（在 motor_process_state 中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_request_model(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    hightorque_request_model(fdcanHandle, id);
}


/**
 * @brief 发送查询电机硬件版本号指令（cmd: 0x00 0x05，在 motor_process_state 中解析）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_request_hw_version(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    hightorque_request_hw_version(fdcanHandle, id);
}


/**
 * @brief 停止模式，电机三相都断开（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_stop(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    hightorque_stop_int16(fdcanHandle, id);
}


/**
 * @brief 刹车模式（阻尼模式），电机三相都接地（并让电机返回状态信息）
 * @param portx CAN 通道选择，用于指定通信的 CAN 端口
 * @param id 电机 ID
 */
void motor_brake(port_t portx, const uint8_t id)
{
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    hightorque_brake_int16(fdcanHandle, id);
}
