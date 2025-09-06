#include "test_motor.h"




void test_motor_control(const uint8_t id)
{
    const uint8_t mode = 3;
    const port_t portx = PORT1;

    switch (mode)
    {
    case 0:
        motor_set_dq_vlot(portx, id, 2);
        break;
    case 1:
        motor_set_dq_current(portx, id, 0.5);
        break;
    case 2:
        motor_set_pos(portx, id, 3);
        break;
    case 3:
        motor_set_vel(portx, id, 0.1f);
        break;
    case 4:
        motor_set_tqe(portx, id, 0.5f);
        break;
    case 5:
        motor_set_pos_vel_MAXtqe(portx, id, 2, 0.1, 10);
        break;

    default:
        break;
    }

    motor_get_state_send(portx, id);
}

