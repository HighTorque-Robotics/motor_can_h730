#ifndef _TEST_MOTOR_H
#define _TEST_MOTOR_H



#include "motor_control.h"
#include "fdcan.h"
#include "motor_config.h"


void test_motor_control(const uint8_t id);
void test_motor_print_state(void);
void test_motor_print_version(void);
#endif

