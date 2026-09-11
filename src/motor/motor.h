#ifndef _MOTOR_H
#define _MOTOR_H


#define  MOTOR_PORT_NUM  1  // 使用 CAN 通道数量
#define  MOTOR_MAX_NUM   1  // 单个 CAN 通道所连接的最大电机数量


#include "fdcan.h"
#include "my_can.h"
#include "convert.h"
#include "hightorque_can.h"


typedef enum __packed  // 电机控制模式码
{
    MODE_NULL        = 0,     // 空模式 (占位)
    MODE_STOP        = 0x01,  // 停止 (惯性停止)
    MODE_BRAKE       = 0x18,  // 刹车 (主动制动)
    MODE_CMD         = 0x02,  // 普通控制 (当前未启用)
    MODE_SYSTEM      = 0x03,  // 系统命令 (查询/设置/保存/重启/改ID)
    MODE_VOLT        = 0x19,  // DQ 电压控制
    MODE_CUR         = 0x1A,  // DQ 电流控制
    MODE_TQE         = 0x1B,  // 力矩控制
    MODE_VEL         = 0x1C,  // 速度控制
    MODE_POS         = 0x1D,  // 位置控制
    MODE_VEL_ACC     = 0x1E,  // 速度 + 加速度
    MODE_POS_VEL_TQE = 0x1F,  // 位置 + 速度 + 前馈力矩
    MODE_POS_VEL_ACC = 0x20,  // 位置 + 速度 + 加速度(梯形)
    MODE_MIT         = 0x21,  // MIT 运控 (位置+速度+力矩+Kp+Kd)
} motor_mode_t;


typedef enum __packed  // 电机查询码
{
    QUERY_SYSTEM                           = 0x03,  // 查询系统信息
    QUERY_FW_VERSION                       = 0x04,  // 查询固件版本
    QUERY_HW_VERSION                       = 0x05,  // 查询硬件版本
    QUERY_MODEL                            = 0x07,  // 查询电机型号
    QUERY_FLAUT_POS_VEL_TQE                = 0x0E,  // 错误码、位置、速度、力矩（查询指令 0x0E, 返回帧 8 字节, 适配经典 CAN）

} motor_query_t;


typedef enum __packed
{
    PNULL = 0,
    PORT1,
    PORT2,
    PORT3,
} port_t;


typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} version_s, *p_version_s;


typedef struct
{
    motor_mode_t mode;      // 模式（对应 motor_mode_t 枚举）
    uint8_t fault;          // 错误码
    float position;         // 位置
    float velocity;         // 速度
    float torque;           // 力矩
    int8_t  temp;           // 温度（单位：摄氏度，分辨率：1度）
    uint8_t ack;            // 应答，用于电机设置相关的应答
    version_s version;      // 电机固件版本号
    version_s hw_version;   // 电机硬件版本号

    char model[25];         // 电机型号 (如 "5036_02")
    // float i_d;           // d轴电流
    // float i_q;           // q轴电流
} motor_state_s, *p_motor_state_s;  // 这个结构体会定义成结构体数组，其中数组下标 +1 即为电机 ID


typedef struct
{
    const port_t port;
    FDCAN_HandleTypeDef *fdcan;
    const p_motor_state_s state;
} port_mapping_s, *p_port_mapping_s;






p_motor_state_s motor_get_state(port_t portx, uint8_t id);

FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx);
p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle);
p_motor_state_s motor_get_state_pointer2(port_t portx);
void motor_process_state_all(void);




#define  MOTOR_SDK_VERSION   "4.0.0-can1"

#endif
