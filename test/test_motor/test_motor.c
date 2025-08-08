#include "test_motor.h"




void test_motor_control(const uint8_t id)
{
    const uint8_t mode = 3;

    switch (mode)
    {
    case 0:
        motor_set_dq_vlot(PORT1, id, 2);
        break;
    case 1:
        motor_set_dq_current(PORT1, id, 0.5);
        break;
    case 2:
        motor_set_pos(PORT1, id, 3);
        break;
    case 3:
        motor_set_vel(PORT1, id, 0.1f);
        break;
    case 4:
        motor_set_tqe(PORT1, id, 0.5f);
        break;
    case 5:
        motor_set_pos_vel_MAXtqe(PORT1, id, 2, 0.1, NAN);
        break;

    default:
        break;
    }
}

