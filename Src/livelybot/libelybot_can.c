#include "libelybot_can.h"



/**
 * @brief DQ电压控制
 * @param id 电机ID
 * @param vol Q相电压，单位：0.1v，如 vol = 10 表示 Q 相电压为 1V
 */
void motor_control_volt(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t vol)
{
    static uint8_t tdata[] = {0x01, 0x00, 0x08, 0x05, 0x1b, 0x00, 0x00};

    // *(int16_t *)&tdata[5] = vol;
    my_memcpy(&tdata[5], &vol, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief DQ电流控制
 * @param id 电机ID
 * @param cur Q相电流，单位：0.1A，如 cur = 10 表示 Q 相电压为 1A
 */
void motor_control_cur(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t cur)
{
    static uint8_t tdata[] = {0x01, 0x00, 0x09, 0x05, 0x1c, 0x00, 0x00};

    // *(int16_t *)&tdata[5] = cur;
    my_memcpy(&tdata[5], &cur, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 位置控制
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param tqe：最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM，不想控制力矩建议给值 0x8000 （表示无限制）
 */
void motor_control_pos(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int32_t pos, int16_t tqe)
{
    static uint8_t tdata[] = {0x07, 0x07, 0x0A, 0x05, 0x00, 0x00, 0x80, 0x00};

    // *(int16_t *)&tdata[2] = pos;
    // *(int16_t *)&tdata[6] = tqe;
    my_memcpy(&tdata[2], &pos, sizeof(int16_t));
    my_memcpy(&tdata[6], &tqe, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 速度控制
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM，不想控制力矩建议给值 0x8000 （表示无限制）
 */
void motor_control_vel(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t vel, int16_t tqe)
{
    static uint8_t tdata[] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    // *(int16_t *)&tdata[4] = vel;
    // *(int16_t *)&tdata[6] = tqe;
    my_memcpy(&tdata[4], &vel, sizeof(int16_t));
    my_memcpy(&tdata[6], &tqe, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 力矩模式
 * @param id 电机ID
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示力矩为 1.1NM
 */
void motor_control_tqe(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int32_t tqe)
{
    static uint8_t tdata[] = {0x05, 0x13, 0x00, 0x80};

    // *(int16_t *)&tdata[2] = tqe;
    my_memcpy(&tdata[2], &tqe, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 电机位置-速度-最大力矩，int16型
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param val 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM，不想控制力矩建议给值 0x8000 （表示无限制）
 */
void motor_control_pos_vel_MAXtqe(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t vel, int16_t tqe)
{
    static uint8_t tdata[] = {0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // *(int16_t *)&tdata[2] = val;
    // *(int16_t *)&tdata[4] = tqe;
    // *(int16_t *)&tdata[6] = pos;
    my_memcpy(&tdata[2], &vel, sizeof(int16_t));
    my_memcpy(&tdata[4], &tqe, sizeof(int16_t));
    my_memcpy(&tdata[6], &pos, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 梯形控制（电机固件 v4.6.0 开始支持）
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param val 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param acc 加速度：单位 0.01 转/秒^2，如 vel = 40 表示 0.4 转/秒^2
 */
void motor_control_pos_vel_acc(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t vel, int16_t acc)
{
    static uint8_t tdata[] = {0x07, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    my_memcpy(&tdata[2], &pos, sizeof(int16_t));
    my_memcpy(&tdata[4], &vel, sizeof(int16_t));
    my_memcpy(&tdata[6], &acc, sizeof(int16_t));

    can_send(hfdcanx, id, tdata, sizeof(tdata));
}


/**
 * @brief 运控模式（MIT模式），电机固件 v4.6.0 开始支持
 */
void motor_control_pos_vel_tqe_kp_kd(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t vel, int16_t tqe, int16_t kp, int16_t kd)
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

    can_send(hfdcanx, 0x18000 | id, tdata, sizeof(tdata));
}


/**
 * @brief 将当前位置设为电机零位(此指令只是在 RAM 中修改，还需配合 `conf write` 指令保存到 flash 中)
 * @param id 电机ID
 */
void rezero_pos(FDCAN_HandleTypeDef *hfdcanx, uint8_t id)
{
    uint8_t tdata[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0a};

    can_send(hfdcanx, 0x8000 | id, tdata, sizeof(tdata));
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
void send_read_motor_state(FDCAN_HandleTypeDef *hfdcanx, uint8_t id)
{
    static uint8_t tdata[] = {0x17, 0x01};

    can_send(hfdcanx, 0x8000 | id, tdata, sizeof(tdata));
}


/**
 * @brief 获取电机固件版本
 * @param fdcanHandle &hfdcanx
 * @param id id 电机ID
 */
void send_read_motor_version(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id)
{
    const uint8_t cmd[] = {0x15, 0xB5, 0x02};

    can_send(fdcanHandle, 0x8000 | id, (uint8_t *)cmd, sizeof(cmd));
}





