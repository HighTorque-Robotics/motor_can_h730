#include "test_modes_motor.h"
#include "motor_config.h"
#include <stdbool.h>



void test_modes_motor(const uint8_t id)
{
	static uint32_t tick_1ms = 0;
	static int16_t t = 3;
	static int16_t num = 0;
	static uint8_t mode = 0;
	static uint8_t test_phase = 2; // 用于测试模式切换
	const int8_t rollback = -1;
	static uint16_t a = 1;

	const data_type_t type = TFLOAT;
	static bool error_flag = false;
	static int16_t fault;

	static float vlot = 1.5f;
	static float cur = 0.5f;
	static float pos = 3.0f;
	static float vel = 0.3f;
	static float tqe = 0.6f;
	static float acc = 0.5f;
	static float kp = 1.0f;
	static float kd = 1.0f;
	const char *test_stage_name[] = {
		"单电机模式测试"};
	// 单电机模式名称
	const char *single_motor_mode_name[] = {
		"电压模式",
		"电流模式",
		"位置模式",
		"速度模式",
		"力矩模式",
		"位置+速度模式",
		"位置+速度+最大力矩模式",
		"位置+速度+加速度模式",
		"位置+速度+力矩+KP+KD模式2"};
		// 单电机模式指令
	const char *single_motor_mode_instruction[] = {
		"motor_set_dq_vlot",
		"motor_set_dq_current",
		"motor_set_pos",
		"motor_set_vel",
		"motor_set_tqe",
		"motor_set_pos_vel",
		"motor_set_pos_vel_MAXtqe",
		"motor_set_pos_velmax_acc",
		"otor_set_pos_vel_tqe_kp_kd_2"};



//	motor_get_state_send(PORT1, type, id);
	const p_motor_state_s p_motor_state = motor_get_state(PORT1, id);
	// 错误处理
	if (!error_flag && p_motor_state->fault != 0)
	{
		fault = p_motor_state->fault;
		error_flag = true;
	}

	// 发生错误后不再继续测试
	if (error_flag)
	{
		printf("在当前模式下检测到故障 %d: fault = %d\n", mode, fault);
		led_toggle_err();
		motor_set_stop(PORT1,id);
	}

	if (HAL_GetTick() - tick_1ms >= 1000 * t)
	{
		tick_1ms = HAL_GetTick();
		t = 5;
		if (num > 3)
		{
			mode++;
			num = 0;
			printf("\n");
			printf("\n");
			printf("\n");
			motor_set_stop(PORT1, id);
			return;
		}
		// 在切换 test 时输出阶段名称
		static uint8_t last_test_phase = 255;
		if (test_phase != last_test_phase)
		{
			printf("\n");
			printf("\n");
			printf("\n");
			printf("=== 切换到测试阶段: %s ===\r\n", test_stage_name[test_phase]);
			last_test_phase = test_phase;
		}
		if (test_phase == 0)
		{
			static int8_t last_mode = -1; // 用于打印模式切换提示

			if (mode != last_mode)
			{
				printf(">>> 当前单电机测试模式: %s <<<\r\n", single_motor_mode_name[mode]);
				printf(">>> 当前测试模式指令: %s <<<\r\n",single_motor_mode_instruction[mode]);
				
				last_mode = mode;
			}
			switch (mode)
			{
			case 0:
				motor_set_dq_vlot(PORT1, id, vlot);
				vlot *= rollback;
				break;
			case 1:
				motor_set_dq_current(PORT1, id, cur);
				cur *= rollback;
				break;
			case 2:
				motor_set_pos(PORT1, id, pos);
				pos *= rollback;
				break;
			case 3:
				motor_set_vel(PORT1, id, vel);
				vel *= rollback;
				break;
			case 4:
				motor_set_tqe(PORT1, id, tqe);
				tqe *= rollback;
				break;
			case 5:
				motor_set_pos_vel(PORT1, id, pos, vel);
				pos *= rollback;
				break;
			case 6:
				motor_set_pos_vel_MAXtqe(PORT1, id, pos, vel, tqe);
				pos *= rollback;
				break;
//			case 8:
//				motor_set_pos_vel_tqe_kp_kd(PORT1, type, id, pos, vel, tqe, kp, kd);
//				pos *= rollback;
//				break;
			case 7:
				motor_set_pos_vel_tqe_kp_kd(PORT1, id, pos, vel, tqe, kp, kd);
				pos *= rollback;
				break;
			default:
				test_phase = 1;
				mode = 0;
				num =0;
				last_mode = -1; // 切换阶段时重置
				break;
			}
			num++;
			printf("motor ID: %2d, mode: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", id, mode, p_motor_state->fault,
				   p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
		}

		

		else if (test_phase == 2)
		{
			switch (a)
			{
			case 1:
				printf("\n\n\n");
				printf("正在测试：电机刹车\n");
				motor_set_vel(PORT1, id, vel);
				break;
			case 2:
				motor_set_brake(PORT1, id);
				break;
			case 3:
				printf("\n\n\n");
				printf("正在测试：重置电机零位\n");
				motor_set_vel(PORT1, id, vel);
				break;
			case 4:
				motor_pos_reset(PORT1, id);
				break;
			case 5:
				motor_get_state_send(PORT1, id);
				break;
			case 8:
				mode = 0;
				num = 0;
				test_phase = 0;
				a = 0; // 重置 a
				break;
			default:
				break;
			}
			printf("ID: %2d, mode: %2d, fault: %2d, pos: %.3lf, vel: %.3lf, tqe: %.3lf\r\n", id, mode, p_motor_state->fault,
				   p_motor_state->position, p_motor_state->velocity, p_motor_state->torque);
			a++;
		}
	}
	motor_get_state_send(PORT1, id);
	return;
}

