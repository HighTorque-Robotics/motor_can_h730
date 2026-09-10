#include "test_motor.h"
#include <stdio.h>


void test_motor_control(const uint8_t id)
{
    const uint8_t mode = 3;
    const data_type_t type = TINT16;  // 经典 CAN 数据区仅 8 字节, 只支持 int16
    const port_t portx = PORT1;

    switch (mode)
    {
    case 0:
        motor_dq_vlot(portx, type, id, 0.0);
        break;
    case 1:
        motor_dq_current(portx, type, id, 0.5f);
        break;
    case 2:
        motor_pos(portx, type, id, 0.0);
        break;
    case 3:
        motor_vel(portx, type, id, 0.1f);
        break;
    case 4:
        motor_tqe(portx, type, id, 0.5f);
        break;
    case 5:
        motor_pos_vel(portx, type, id, 0.0, 0.5f);
        break;
    case 6:
        motor_pos_vel_MAXtqe(portx, type, id, 0.0, 0.5f, 0.5f);
        break;
    case 7:
        motor_pos_velmax_acc(portx, type, id, 0, 1.0f, 0.1);
        break;
    case 8:
        motor_pos_vel_tqe_kp_kd(portx, type, id, 3, 0, 0, 10, 0);
        break;
    case 9:
        motor_vel_acc(portx, type, id, 1.0f, 0.1);
        break;
    case 10:
        motor_stop(portx, type, id);
        break;
    case 11:
        motor_request_state(portx, type, id);
        break;
    case 12:
        motor_request_fw_version(portx, id);
        break;
    case 13:
        motor_brake(portx, type, id);
        break;
    case 14:
        motor_request_model(portx, id);
        break;
    default:
        break;
    }
}
