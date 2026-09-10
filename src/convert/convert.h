#ifndef _CONVERT_H
#define _CONVERT_H


#include "main.h"
#include "math.h"
#include "led.h"


#define  MOTOR_DATA_TYPE_FLAG  TURNS
#define  BROADCAST_ID  0x7F
#define  MOTOR_ID_MIN  1                    // 电机 ID 下限
#define  MOTOR_ID_MAX  (BROADCAST_ID - 1)   // 电机 ID 上限 = 126 (127 为广播地址, 不可作为电机 ID)


#ifdef  LED_ERR_FLAG  // 这个宏定义在 led.h 中
#define  MOTOR_ERR    led_toggle_err  // 所有 led 闪烁
#else
static inline void MOTOR_ERR(void) {}
#endif


#define MY_2PI (6.28318530717f)
#define MY_PI  (3.14159265358f)


typedef enum
{
    RADIAN_2PI = 0,  // 弧度制
    ANGLE_360,       // 角度制
    TURNS,           // 圈数
} pos_vel_type_t;


typedef enum __packed  // 数据类型 (枚举值 = CAN ID bits[17:16])
{
    TINT16_NOHDR = 0,
    TINT16       = 1,
    TINT32       = 2,
    TFLOAT       = 3,
} data_type_t;


/* 控制电机用 */
float conv_to_turns(const float in_data, const pos_vel_type_t type);
float cur_float2int(const float in_data, const data_type_t type);
float vol_float2int(const float in_data, const data_type_t type);
float pos_float2int(const float in_data, const data_type_t type);
float vel_float2int(const float in_data, const data_type_t type);
float tqe_float2int(const float in_data, const data_type_t type);
float acc_float2int(const float in_data, const data_type_t type);
float pid_float2int(const float in_data, const data_type_t type);


/* 读取电机用 */
float conv_from_turns(const float in_data, const pos_vel_type_t type);
float cur_int2float(const float in_data, const data_type_t type);
float vol_int2float(const float in_data, const data_type_t type);
float pos_int2float(const float in_data, const data_type_t type);
float vel_int2float(const float in_data, const data_type_t type);
float tqe_int2float(const float in_data, const data_type_t type);
float acc_int2float(const float in_data, const data_type_t type);
float pid_int2float(const float in_data, const data_type_t type);
float temp_int2float(const float in_data, const data_type_t type);


/* MIT 运控编码: 按量程无符号偏移映射到 n bit (饱和限幅), 量程对应 can_mit_int2float */
uint16_t mit_pos_float2uint(const float in_data);    /* 16bit, ±3.2768 圈 */
uint16_t mit_vel_float2uint(const float in_data);    /* 12bit, ±10 圈/s */
uint16_t mit_tqe_float2uint(const float in_data);    /* 12bit, ±100 Nm */
uint16_t mit_kp_float2uint(const float in_data);     /* 12bit, ±800 */
uint16_t mit_kd_float2uint(const float in_data);     /* 12bit, ±200 */


/* 数据搬运 */
void my_memcpy(void *p1, const void *p2, const int16_t len);


#endif
