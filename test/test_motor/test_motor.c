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
        motor_dq_vlot(portx, id, 0.0);
        break;
    case 1:
        motor_dq_current(portx, id, 0.5f);
        break;
    case 2:
        motor_pos(portx, id, 0.1f);
        break;
    case 3:
        motor_vel(portx, id, 0.1f);
        break;
    case 4:
        motor_tqe(portx, id, 0.5f);
        break;
    case 5:
        motor_pos_vel(portx, id, 0.0, 0.5f);
        break;
    case 6:
        motor_pos_vel_MAXtqe(portx,id, -0.5, 0.5f, 0.5f);
        break;
    case 7:
        motor_pos_velmax_acc(portx, id, 0, 1.0f, 0.1);
        break;
    case 8:
        motor_pos_vel_tqe_kp_kd(portx, id, -0.7, 0.5, 0.2, 5, 1);
        break;
    case 9:
        motor_vel_acc(portx, id, -1.0f, 0.1);
        break;
    case 10:
        motor_stop(portx, id);
        break;
    case 11:
        motor_request_state(portx, id);
        break;
    case 12:
        motor_request_fw_version(portx, id);
        break;
    case 13:
        motor_brake(portx, id);
        break;
    case 14:
        motor_request_model(portx, id);
        break;
    default:
        break;
    }
}

void test_motor_print_state()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            motor_state_s *p_motor_state = motor_get_state(portx, id);
            printf("PORT: %d, ID: %2d, mode: %2d, temp: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", portx, id, p_motor_state->mode, p_motor_state->temp,
                   p_motor_state->fault, p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
        }
        printf("\r\n");
    }
}

void test_motor_print_version()
{
    for (uint8_t portx = PORT1; portx < PORT1 + MOTOR_PORT_NUM; portx++)
    {
        for (uint8_t id = 1; id <= MOTOR_MAX_NUM; id++)
        {
            const p_version_s p_version = &(motor_get_state(portx, id)->version);

            printf("PORT: %d, ID: %2d, version = %d.%d.%d\r\n", portx, id, p_version->major, p_version->minor, p_version->patch);
        }
        printf("\r\n");
    }
}
