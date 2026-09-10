#include "convert.h"

#ifdef __MICROLIB  // 有无启用MicroLIB库
#include <string.h>
#endif


static float data_limit(const float in_data, const float max, const float min)
{
    if (in_data >= max)
    {
        return max;
    }
    else if (in_data <= min)
    {
        return min;
    }

    return in_data;
}


static float data_float2int(const float in_data, const data_type_t type, const float rint16, const float rint32)
{
    switch (type)
    {
    case TINT16_NOHDR:
    case TINT16:
        return data_limit(in_data * rint16, 32767.0f, -32768.0f);
    case TINT32:
        return data_limit(in_data * rint32, 2147483647.0f, -2147483648.0f);
    case TFLOAT:
        return in_data;
    default:
        MOTOR_ERR();
        return 0;
    }
}


static float data_int2float(const float in_data, const data_type_t type, const float rint16, const float rint32)
{
    switch (type)
    {
    case TINT16_NOHDR:
    case TINT16:
        return in_data / rint16;
    case TINT32:
        return in_data / rint32;
    case TFLOAT:
        return in_data;
    default:
        MOTOR_ERR();
        return 0;
    }
}


/**
 * @brief 将弧度或角度转化成圈数
 * @param in_data 弧度值或角度值
 * @param type in_data的类型
 * @return 圈数值
 */
float conv_to_turns(const float in_data, const pos_vel_type_t type)
{
    switch (type)
    {
    case RADIAN_2PI:
        return in_data / MY_2PI;
    case ANGLE_360:
        return in_data / 360.0f;
    case TURNS:
        return in_data;
    default:
        MOTOR_ERR();
        return 0.0f;
    }
}


/**
 * @brief 将圈数转化成弧度或角度
 * @param in_data 圈数值
 * @param type 要转换成的类型
 * @return 弧度值或角度值
 */
float conv_from_turns(const float in_data, const pos_vel_type_t type)
{
    switch (type)
    {
    case RADIAN_2PI:
        return in_data * MY_2PI;
    case ANGLE_360:
        return in_data * 360.0f;
    case TURNS:
        return in_data;
    default:
        MOTOR_ERR();
        return 0.0f;
    }
}


float cur_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 10.0f, 1000.0f);
}


float cur_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 10.0f, 1000.0f);
}


float vol_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 10.0f, 1000.0f);
}


float vol_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 10.0f, 1000.0f);
}


float pos_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 10000.0f, 100000.0f);
}


float pos_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 10000.0f, 100000.0f);
}


float vel_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 4000.0f, 100000.0f);
}


float vel_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 4000.0f, 100000.0f);
}


float tqe_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 100.0f, 1000.0f);
}


float tqe_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 100.0f, 1000.0f);
}


float acc_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 1000.0f, 100000.0f);
}


float acc_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 1000.0f, 100000.0f);
}


float pid_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 10.0f, 1000.0f);
}


float pid_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 10.0f, 1000.0f);
}


float temp_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 10.0f, 1000.0f);
}


/**
 * @brief MIT 运控编码: float -> n-bit 无符号偏移映射 (饱和限幅), 对应 can_mit_int2float 量程
 * @param in_data 物理值
 * @param min 量程下限
 * @param max 量程上限
 * @param bits 位宽
 * @return 0 ~ (2^bits - 1) 的无符号原始值
 * @note 电机端按 raw/(2^bits-1)×(max-min)+min 反解: 物理 0 落在量程中点
 */
static uint16_t mit_data_float2uint(const float in_data, const float min, const float max, const unsigned int bits)
{
    const float span = max - min;

    if (span <= 0.0f)
    {
        return 0;
    }

    const float a = data_limit(in_data, max, min);  // 先钳位到量程内

    return (uint16_t)((a - min) * ((1 << bits) - 1) / span);  // 再映射到 n-bit 偏移码
}


uint16_t mit_pos_float2uint(const float in_data)
{
    return mit_data_float2uint(in_data, -3.2768f, 3.2767f, 16);
}


uint16_t mit_vel_float2uint(const float in_data)
{
    return mit_data_float2uint(in_data, -10.0f, 10.0f, 12);
}


uint16_t mit_tqe_float2uint(const float in_data)
{
    return mit_data_float2uint(in_data, -100.0f, 100.0f, 12);
}


uint16_t mit_kp_float2uint(const float in_data)
{
    return mit_data_float2uint(in_data, -800.0f, 800.0f, 12);
}


uint16_t mit_kd_float2uint(const float in_data)
{
    return mit_data_float2uint(in_data, -200.0f, 200.0f, 12);
}


void my_memcpy(void *p1, const void *p2, const int16_t len)
{
    if (len <= 0)
    {
        MOTOR_ERR();
        return;
    }

#ifdef __MICROLIB  // 有无启用MicroLIB库
    memcpy(p1, p2, len);
#else
    uint8_t *p11 = (uint8_t *)p1;
    const uint8_t *p22 = (uint8_t *)p2;
    for (int i = 0; i < len; i++)
    {
        p11[i] = p22[i];
    }
#endif
}
