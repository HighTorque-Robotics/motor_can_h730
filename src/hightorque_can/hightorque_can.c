#include "hightorque_can.h"
#include "my_can.h"
#include "motor.h"


/**
 * @brief 底层发送: 自动置 bit[15]=1 (发送控制帧), >0x7FF 自动走 29 位扩展帧
 * @param hcan &hcanx
 * @param id CAN ID (标题宏 | 电机ID)
 * @param data 数据指针
 * @param size 数据长度 (经典 CAN 最大 8 字节)
 */
void fdcan_send(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint16_t size)
{
    can_send(hcan, id | 0x8000u, data, size);  // 自动置 bit[15]=1 (发送控制帧)
}


/**
 * @brief 电压控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param d d轴电压，单位：0.1V（通常设为 0）
 * @param q q轴电压，单位：0.1V
 */
void hightorque_dq_volt_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t d, int16_t q)
{
    static uint8_t cmd[] = {MODE_VOLT, FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[4], &q, sizeof(q));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电流控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param d d轴电流，单位：0.1A（通常设为 0）
 * @param q q轴电流，单位：0.1A
 */
void hightorque_dq_current_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t d, int16_t q)
{
    static uint8_t cmd[] = {MODE_CUR, FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &d, sizeof(d));
    my_memcpy(&cmd[4], &q, sizeof(q));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 力矩控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param torque 力矩：单位 0.01 Nm，如 torque = 35 表示 0.35 Nm
 */
void hightorque_torque_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t torque)
{
    static uint8_t cmd[] = {MODE_TQE, FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &torque, sizeof(torque));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 */
void hightorque_pos_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t pos)
{
    static uint8_t cmd[] = {MODE_POS, FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 速度控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 */
void hightorque_vel_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t vel)
{
    static uint8_t cmd[] = {MODE_VEL, FLAUT_POS_VEL_TQE, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、力矩控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param torque 力矩：单位 0.01 Nm，如 torque = 100 表示 1.0 Nm
 */
void hightorque_pos_vel_tqe_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t pos, int16_t vel, int16_t torque)
{
    static uint8_t cmd[] = {MODE_POS_VEL_TQE, FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel, sizeof(vel));
    my_memcpy(&cmd[6], &torque, sizeof(torque));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 速度、加速度控制 int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param acc 加速度：单位 0.001 转/秒^2，如 acc = 1000 表示 1.0 转/秒^2
 */
void hightorque_vel_acc_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t vel, int16_t acc)
{
    static uint8_t cmd[] = {MODE_VEL_ACC, FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &vel, sizeof(vel));
    my_memcpy(&cmd[4], &acc, sizeof(acc));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 位置、速度、加速度限制（梯形控制）int16
 * @param hcan &hcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 * @param vel_max 最大速度：单位 0.00025 转/秒，如 vel = 2000 表示 0.5 转/秒
 * @param acc 加速度：单位 0.001 转/秒^2，如 acc = 100 表示 0.1 转/秒^2
 */
void hightorque_pos_vel_acc_int16(FDCAN_HandleTypeDef *hcan, uint8_t id, int16_t pos, int16_t vel_max, int16_t acc)
{
    static uint8_t cmd[] = {MODE_POS_VEL_ACC, FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[2], &pos, sizeof(pos));
    my_memcpy(&cmd[4], &vel_max, sizeof(vel_max));
    my_memcpy(&cmd[6], &acc, sizeof(acc));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 运控模式 int16 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩) (Mkp 表示电机内部 kp, Mkd 表示电机内部 kd)
 * @param hcan &hcanx
 * @param id 电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置
 * @param vel 速度：单位 0.00025 转/秒，如 vel = 400 表示 0.1 转/秒
 * @param tqe 前馈力矩（单位见文档）
 * @param kp Mkp = kp * 0.1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kd * 0.1 (Mkd 表示电机内部 kd)
 */
void hightorque_pos_vel_tqe_kp_kd_int16(FDCAN_HandleTypeDef *hcan, uint8_t id,
                                        int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
{
    static uint8_t tdata[8] = {0};

    tdata[0] = pos & 0xff;
    tdata[1] = (pos >> 8) & 0xff;
    tdata[2] = vel & 0xff;
    tdata[3] = ((vel >> 8) & 0x0f) | ((tqe & 0x0f) << 4);
    tdata[4] = (tqe >> 4) & 0xff;
    tdata[5] = kp & 0xff;
    tdata[6] = (kp >> 8) & 0x0f | ((kd & 0x0f) << 4);
    tdata[7] = kd >> 4;

    can_send(hcan, 0x58000 | id, tdata, sizeof(tdata));
}


/**
 * @brief 电机停止 int16 (三相断开, 自由停转)
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_stop_int16(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t cmd[] = {MODE_STOP, FLAUT_POS_VEL_TQE};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机刹车 int16 (三相接地, 阻尼制动)
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_brake_int16(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t cmd[] = {MODE_BRAKE, FLAUT_POS_VEL_TQE};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 获取电机状态 int16 (查询码 0x0E, 返回帧 8 字节: 查询码/错误码/位置/速度/力矩, 适配经典 CAN 数据区)
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_request_state_int16(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    const uint8_t cmd[] = {0x00, FLAUT_POS_VEL_TQE};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机固件版本
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_request_fw_version(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    const uint8_t cmd[] = {0x00, FW_VERSION};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 获取电机硬件版本
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_request_hw_version(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    const uint8_t cmd[] = {0x00, HW_VERSION};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 查询电机型号
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_request_model(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    const uint8_t cmd[] = {0x00, MODEL};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, (uint8_t *)cmd, sizeof(cmd));
}


/**
 * @brief 周期请求电机状态返回 (TINT16)
 * @param hcan &hcanx
 * @param id 电机ID
 * @param t_us 周期时间, 单位: 1us, 4字节小端; 填 0 表示停止周期返回
 * @note 发送: 0x03 0x00 0x05 <查询码> + 4字节微秒
 *       返回数据格式由 cmd[3] 查询码决定, 当前 FLAUT_POS_VEL_TQE(0x0E)
 *       即 错误码/位置/速度/力矩 (8字节, 适配经典 CAN 数据区)
 */
void hightorque_request_timed_return(FDCAN_HandleTypeDef *hcan, uint8_t id, uint32_t t_us)
{
    static uint8_t cmd[] = {MODE_SYSTEM, 0x00, 0x05, FLAUT_POS_VEL_TQE, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&cmd[4], &t_us, sizeof(uint32_t));

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 重设电机零位
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_pos_rezero(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t cmd[] = {MODE_SYSTEM, 0x03, 0x03};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 保存电机设置
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_conf_write(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t cmd[] = {MODE_SYSTEM, 0x03, 0x02};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 电机软重启
 * @param hcan &hcanx
 * @param id 电机ID
 */
void hightorque_reset(FDCAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t cmd[] = {MODE_SYSTEM, 0x03, 0x01};

    fdcan_send(hcan, ID_PREFIX_TINT16 | id, cmd, sizeof(cmd));
}


/**
 * @brief 更改电机ID
 * @param hcan &hcanx
 * @param old_id 当前电机ID
 * @param new_id 新电机ID
 */
void hightorque_id(FDCAN_HandleTypeDef *hcan, uint8_t old_id, uint8_t new_id)
{
    /* old_id / new_id 需在 1 ~ MOTOR_MAX_NUM 范围内, 否则返回帧不会被解析 (见 motor.c) */
    if (old_id < MOTOR_ID_MIN || old_id > MOTOR_MAX_NUM || new_id < MOTOR_ID_MIN || new_id > MOTOR_MAX_NUM)
    {
        MOTOR_ERR();
        return;
    }

    static uint8_t cmd[] = {MODE_SYSTEM, 0x03, 0x04, 0x00};

    cmd[3] = new_id;

    fdcan_send(hcan, ID_PREFIX_TINT16 | old_id, cmd, sizeof(cmd));
}
