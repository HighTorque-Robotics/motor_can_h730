#ifndef _TEST_MODES_MOTOR_H
#define _TEST_MODES_MOTOR_H



#include "motor_control.h"
#include "fdcan.h"

#define POS_TOLERANCE 0.05f
#define VEL_TOLERANCE 0.01f
#define VEL_MIN_RESPONSE 0.08f  // 最小有效运动速度
typedef struct
{
    float alpha;    // 滤波系数，范围 (0,1)
    float last_val; // 上一次输出
} lpf_t;

void test_modes_motor(const uint8_t id);
void test_single_motor(uint8_t id, uint8_t mode, float target_val);
void test_all_motors(uint8_t mode, float target_val);
#endif

