#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H


#include "hightorque_can.h"
#include "convert.h"
#include "motor.h"


void motor_dq_volt(port_t portx, const uint8_t id, const float volt);
void motor_dq_current(port_t portx, const uint8_t id, const float cur);
void motor_pos(port_t portx, const uint8_t id, const float pos);
void motor_vel(port_t portx, const uint8_t id, const float vel);
void motor_tqe(port_t portx, const uint8_t id, const float tqe);
void motor_pos_vel(port_t portx, const uint8_t id, const float pos, const float vel);
void motor_pos_vel_MAXtqe(port_t portx, const uint8_t id,
                          const float pos, const float vel, const float tqe);
void motor_pos_velmax_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc);
void motor_vel_acc(port_t portx, const uint8_t id, const float vel, const float acc);
void motor_pos_vel_tqe_kp_kd(port_t portx, const uint8_t id,
                             const float pos, const float vel, const float tqe, const float kp, const float kd);

void motor_request_state(port_t portx, const uint8_t id);
void motor_request_fw_version(port_t portx, const uint8_t id);
void motor_request_model(port_t portx, const uint8_t id);
void motor_request_hw_version(port_t portx, const uint8_t id);


void motor_stop(port_t portx, const uint8_t id);
void motor_brake(port_t portx, const uint8_t id);
#endif
