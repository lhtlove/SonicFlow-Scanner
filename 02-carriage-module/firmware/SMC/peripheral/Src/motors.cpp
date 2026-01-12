#include <motors.hpp>
#include "cmath"
#include "smc.hpp"

Stepper stepper;
DCM DCL; DCM DCR;

void Stepper_Init(TIM_HandleTypeDef *htim) {
	stepper.htim_stepper = htim;

    stepper.STEP_PORT = GPIOB;
    stepper.STEP_PIN = GPIO_PIN_0;
    stepper.DIR_PORT = GPIOB;
    stepper.DIR_PIN = GPIO_PIN_1;
    stepper.steps_to_go = 0;
    stepper.current_steps = 0;
    stepper.location_steps = 0;

    stepper.dir = 0;
    stepper.is_running = 0;
    stepper.step_period = 50;  // Initial period

    // Timer setup example (modify based on your clock)
    htim -> Init.Period = stepper.step_period - 1;
    htim -> Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(htim);
}

void DC_Init(TIM_HandleTypeDef *htim, uint8_t id) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;
	DC -> id = id;
	DC -> htim_dc = htim;

	DC -> IN_PORT = GPIOA;
	DC -> IN_PIN = id ? GPIO_PIN_6 : GPIO_PIN_3;
	DC -> OUT_PORT = GPIOA;
	DC -> OUT_PIN = id ? GPIO_PIN_5 : GPIO_PIN_4;
	DC -> counts_to_go = 0;

	DC -> dir = 0;
	DC -> is_running = 0;
	DC -> stopped = 1;

	DC -> period = 200;

	DC -> countE = 0;

	DC -> diag = 0;

    htim -> Init.Period = DC -> period - 1;
    htim -> Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(htim);
}

void Stepper_Move(int32_t steps) {
	static uint8_t dir;

	if (!stepper.is_running) {
		dir = steps < 0 ? 0 : 1;

		HAL_GPIO_WritePin(stepper.DIR_PORT, stepper.DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
		stepper.steps_to_go = abs(steps);
		stepper.current_steps = 0;
		stepper.dir = dir;
		stepper.is_running = 1;

		HAL_TIM_Base_Start_IT(stepper.htim_stepper);
	}
}

void Stepper_Stop() {
	stepper.current_steps = stepper.steps_to_go;
}

void Stepper_TIM_Interrupt() {
	if (stepper.is_running) {
		if (stepper.current_steps < stepper.steps_to_go) {
			// Toggle step pin
			HAL_GPIO_WritePin(stepper.STEP_PORT, stepper.STEP_PIN, GPIO_PIN_SET);
			// Small delay if needed (depends on driver requirements)
			for (volatile uint8_t i = 0; i < 10; i++);
			HAL_GPIO_WritePin(stepper.STEP_PORT, stepper.STEP_PIN, GPIO_PIN_RESET);

			stepper.current_steps++;
			stepper.dir ? stepper.location_steps++ : stepper.location_steps--;
		} else {
			// Movement complete
			stepper.is_running = 0;

			if (on_program) {

			}

			HAL_TIM_Base_Stop_IT(stepper.htim_stepper);
		}
	}
}

uint16_t spd = 0;
uint8_t spdFlag = 0;

uint8_t accelCnt;
uint16_t accel;

void DC_Move(uint8_t id, uint32_t counts, uint8_t dir) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;

	if (!DC -> is_running) {
		HAL_GPIO_WritePin(DC -> OUT_PORT, DC -> OUT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DC -> IN_PORT, DC -> IN_PIN, GPIO_PIN_RESET);

		DC -> error = counts;
		DC -> ms = FULL_SPEED;

		DC -> counts_to_go = dir ?	(DC->countE + counts):
									(DC->countE - counts);
		DC -> counts_going = counts;

		DC -> dir = dir;
		DC -> is_running = 1;
		DC -> stopped = 0;
		DC -> tim = 0;
		DC -> sum = 0;

		spd = 0;
		accelCnt = 0; accel = 0;
		HAL_TIM_Base_Start_IT(DC -> htim_dc);
	}
}

void DC_Reset(uint8_t id) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;

	if (!DC -> is_running && !DC->diag) {
		HAL_GPIO_WritePin(DC -> OUT_PORT, DC -> OUT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DC -> IN_PORT, DC -> IN_PIN, GPIO_PIN_RESET);

		DC -> is_running = 1;
		DC -> stopped = 0;
		DC -> diag = 1;
		DC -> count = 0;
		DC -> duty = 35;
		DC -> dir = 0;
		DC -> tim = 0;

		HAL_TIM_Base_Start_IT(DC -> htim_dc);
	}
}

void DC_TIM_Interrupt(DCM *DC) {
	DC -> tim++;

	if (DC->diag) {
		DC -> count++;
		if(DC->count >= DC->period) DC -> count = 0;

		if (DC->count < DC->duty) HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
			DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
			DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_RESET);

		if (speedTim(DC->id) > 3500 && DC->tim > 5000) DC_Stop(DC->id);

		return;
	}

	switch(DC->ms) {
	case FULL_SPEED:
		HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
			DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_SET);

		if ((float)(DC->counts_going < 1000 ? DC->counts_going : 1000)
				* (0.0002f * (float)DC->sum) * ((float)DC->sum / 1500.0f) >= (float)DC->error) {
			DC->ms = COAST;
			DC->brakepoint = (uint16_t)(0.12f * (float)DC->sum);
		}
		break;

	case COAST:
		if (DC->error <= 15) DC->brakepoint = 700;
//		else if (error <= 5) brakepoint = 1000;

		if (DC -> speedE <= DC->brakepoint && 0 <= DC->error) {
			HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
						DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN,
								DC->brakepoint < 500 ? GPIO_PIN_SET : GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DC -> dir ? DC -> IN_PORT : DC -> OUT_PORT,
					DC -> dir ? DC -> IN_PIN : DC -> OUT_PIN, GPIO_PIN_SET);
		} else if (DC -> speedE > 1100) {
			HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
						DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DC -> dir ? DC -> IN_PORT : DC -> OUT_PORT,
					DC -> dir ? DC -> IN_PIN : DC -> OUT_PIN, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(DC -> OUT_PORT, DC -> OUT_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DC -> IN_PORT, DC -> IN_PIN, GPIO_PIN_RESET);
		}


		if (DC->error <= 8 || speedTim(DC->id) > 5000) { // || DC->tim - timB > 1000
			DC->ms = BRAKE;

//			spd = DC -> tim;
//			spdFlag = 1;
		}

		break;

	case BRAKE:
		DC_Stop(DC->id);

		break;
	}
}


void DC_Stop(uint8_t id) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;

	if (DC->diag) {
		HAL_GPIO_WritePin(DC -> OUT_PORT, DC -> OUT_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DC -> IN_PORT, DC -> IN_PIN, GPIO_PIN_RESET);

		DC -> is_running = 0;
		DC -> stopped = 1;
		DC -> diag = 0;
		DC -> countE = -200;
		HAL_TIM_Base_Stop_IT(DC -> htim_dc);

		return;
	}

	HAL_GPIO_WritePin(DC -> OUT_PORT, DC -> OUT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DC -> IN_PORT, DC -> IN_PIN, GPIO_PIN_SET);

	DC -> is_running = 0;
	DC -> stopped = -1;
	HAL_TIM_Base_Stop_IT(DC -> htim_dc);


//			abs(DC -> counts_to_go - dc_encoder_count(DC -> id));

//	HAL_Delay(100);

//	HAL_GPIO_WritePin(DC -> OUT_PORT, DC -> OUT_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(DC -> IN_PORT, DC -> IN_PIN, GPIO_PIN_RESET);
}

void Stepper_SetSpeed(uint32_t steps_per_second) {
    if(steps_per_second > 0) {
//        stepper1.step_period = (SystemCoreClock / 16) / steps_per_second;
    	stepper.step_period = steps_per_second;
        __HAL_TIM_SET_AUTORELOAD(stepper.htim_stepper, stepper.step_period - 1);
    }
}
