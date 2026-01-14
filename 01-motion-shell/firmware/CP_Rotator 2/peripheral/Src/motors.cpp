#include <motors.hpp>
#include "stepper_setting.hpp"

#include "stdio.h"
#include "cmath"
#include "rotator.h"
#include "can.h"

Stepper stepper;
int16_t incremental_speed = -1;
uint16_t target = 0;

void Stepper_Init(TIM_HandleTypeDef *htim) {
	stepper.htim_stepper = htim;

    stepper.STEP_PORT = GPIOB;
    stepper.STEP_PIN = GPIO_PIN_0;
    stepper.DIR_PORT = GPIOB;
    stepper.DIR_PIN = GPIO_PIN_1;
    stepper.ENN_PORT = GPIOA;
    stepper.ENN_PIN = GPIO_PIN_6;

    stepper.steps_to_go = 0;
    stepper.current_steps = 0;
    stepper.location_steps = 0;
    stepper.time = 0;

    stepper.dir = 0;
    stepper.is_running = 0;
    stepper.step_period = 70;  // Initial period

    // Timer setup example (modify based on your clock)
    htim -> Init.Period = stepper.step_period - 1;
    htim -> Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(htim);
}

uint8_t Stepper_Move(int32_t steps) {
	if (!homing) Stepper2_Move(steps);

	static uint8_t dir;

	if (!stepper.is_running) {
		dir = steps < 0 ? 0 : 1;
//		sg_result = 0;

		HAL_GPIO_WritePin(stepper.DIR_PORT, stepper.DIR_PIN, dir ? GPIO_PIN_RESET : GPIO_PIN_SET);
		stepper.steps_to_go = abs(steps);
		stepper.current_steps = 0;
		stepper.dir = dir;
		stepper.is_running = 1;
		stepper.time = 0;

		HAL_TIM_Base_Start_IT(stepper.htim_stepper);

		return 1;
	} else {
		return 0;
	}
}

void Stepper_Stop() {
	if (!homing) Stepper2_Stop();

	stepper.current_steps = stepper.steps_to_go;
	Stepper_TIM_Interrupt();
}

void Stepper_TIM_Interrupt() {
//	static uint16_t time_before;

	if (stepper.current_steps < stepper.steps_to_go) {
		// Toggle step pin
		HAL_GPIO_WritePin(stepper.STEP_PORT, stepper.STEP_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(stepper.STEP_PORT, stepper.STEP_PIN, GPIO_PIN_RESET);

		stepper.current_steps++;
		stepper.dir ? stepper.location_steps++ : stepper.location_steps--;

		stepper.time += __HAL_TIM_GET_AUTORELOAD(stepper.htim_stepper);
	} else {
		// Movement complete
		stepper.is_running = 0;

		if (on_program) {
			on_program = 0;
			uint8_t packet[8];

			packet[0] = 0x10;

			CAN_Tx(packet, 8, MASTER_ID);
		}

//		if (target) {
//			__HAL_TIM_SET_AUTORELOAD(stepper.htim_stepper, stepper.step_period + target - 1);
//			incremental_speed = target;
//		} printf("end\n");

		HAL_TIM_Base_Stop_IT(stepper.htim_stepper);

		if (homing && stepper.dir) rotator_reset(1);
	}

//	if (incremental_speed >= 0 && stepper.time / 50000 != time_before) {
//		time_before = stepper.time / 50000;
//		__HAL_TIM_SET_AUTORELOAD(stepper.htim_stepper, stepper.step_period + (incremental_speed--) - 1);
//	}
}

void Stepper_Enable(uint8_t toggle) {
	Stepper2_Enable(toggle);

	HAL_GPIO_WritePin(stepper.ENN_PORT, stepper.ENN_PIN,
			toggle ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Stepper_Reset() {
	if (!homing) Stepper2_Reset();

	Stepper_Enable(0);
	for (volatile uint8_t i = 0; i < 10; i++);
	Stepper_Enable(1);
}

void Stepper_SetSpeed(uint32_t step_interval) {
	if (!homing) Stepper2_SetSpeed(step_interval);

    if(step_interval > 0) {
    	stepper.step_period = step_interval;

    	if (step_interval < 640 / USTEP_RATE) {
    		incremental_speed = (640 / USTEP_RATE) - step_interval;
    		target = incremental_speed;
    	} else {
    		incremental_speed = -1;
    		target = 0;
    	} __HAL_TIM_SET_AUTORELOAD(stepper.htim_stepper, stepper.step_period + target - 1);
    }
}
