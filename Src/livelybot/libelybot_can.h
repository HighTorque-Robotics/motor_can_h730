#ifndef _LIBELYBOT_CAN_H
#define _LIBELYBOT_CAN_H


#include "my_can.h"
#include "convert.h"

/* NAN 表示不限制 */
#define  INI8_NAN   0x80
#define  INT16_NAN  0x8000
#define  INT32_NAN  0x80000000


typedef struct
{
    uint32_t id;
    int16_t position;
    int16_t velocity;
    int16_t torque;
} motor_state_s;

typedef struct
{
    union
    {
        motor_state_s motor;
        uint8_t data[24];
    };
} motor_state_t;


extern motor_state_t motor_state;
extern uint8_t motor_read_flag;





void motor_control_volt(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t vol);
void motor_control_cur(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t cur);
void motor_control_pos(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int32_t pos, int16_t tqe);
void motor_control_vel(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t vel, int16_t tqe);
void motor_control_tqe(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int32_t tqe);
void motor_control_pos_vel_MAXtqe(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t val, int16_t tqe);
void motor_control_pos_vel_acc(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t vel, int16_t acc);
void motor_control_pos_vel_tqe_kp_kd(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t pos, int16_t val, int16_t tqe, int16_t kp, int16_t kd);

void rezero_pos(FDCAN_HandleTypeDef *hfdcanx, uint8_t id);
void conf_write(FDCAN_HandleTypeDef *hfdcanx, uint8_t id);
void timed_return_motor_status(FDCAN_HandleTypeDef *hfdcanx, uint8_t id, int16_t t_ms);
void set_motor_stop(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);
void set_motor_brake(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);

void send_read_motor_state(FDCAN_HandleTypeDef *hfdcanx, uint8_t id);
void send_read_motor_version(FDCAN_HandleTypeDef *fdcanHandle, uint8_t id);


#endif
