#include <motors.hpp>

Stepper stepper1;
DCM DCL; DCM DCR;

void Stepper_Init(TIM_HandleTypeDef *htim) {
	stepper1.htim_stepper = htim;

    stepper1.STEP_PORT = GPIOB;
    stepper1.STEP_PIN = GPIO_PIN_0;
    stepper1.DIR_PORT = GPIOB;
    stepper1.DIR_PIN = GPIO_PIN_1;
    stepper1.steps_to_go = 0;
    stepper1.current_steps = 0;
    stepper1.location_steps = 0;

    stepper1.dir = 0;
    stepper1.is_running = 0;
    stepper1.step_period = 50;  // Initial period

    // Timer setup example (modify based on your clock)
    htim -> Init.Period = stepper1.step_period - 1;
    htim -> Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(htim);
}

void DC_Init(TIM_HandleTypeDef *htim, uint8_t id) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;
	DC -> id = id;
	DC -> htim_dc = htim;

	DC -> IN_PORT = GPIOA;
	DC -> IN_PIN = id ? GPIO_PIN_5 : GPIO_PIN_4;
	DC -> OUT_PORT = GPIOA;
	DC -> OUT_PIN = id ? GPIO_PIN_6 : GPIO_PIN_3;
	DC -> counts_to_go = 0;

	DC -> dir = 0;
	DC -> is_running = 0;
	DC -> dc_period = 100;
	DC -> tick = 30;
	DC -> compFlag = 1;

    htim -> Init.Period = DC -> dc_period - 1;
    htim -> Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(htim);
}
// Set direction and number of steps
void Stepper_Move(uint32_t steps, uint8_t dir) {
    HAL_GPIO_WritePin(stepper1.DIR_PORT, stepper1.DIR_PIN, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    stepper1.steps_to_go = steps;
    stepper1.current_steps = 0;
    stepper1.dir = dir ? 1 : 0;
    stepper1.is_running = 1;

    // Start timer
    HAL_TIM_Base_Start_IT(stepper1.htim_stepper);
}

void Stepper_Stop() {
	stepper1.current_steps = stepper1.steps_to_go;
}

void Stepper_TIM_Interrupt() {
	if (stepper1.is_running) {
		if (stepper1.current_steps < stepper1.steps_to_go) {
			// Toggle step pin
			HAL_GPIO_WritePin(stepper1.STEP_PORT, stepper1.STEP_PIN, GPIO_PIN_SET);
			// Small delay if needed (depends on driver requirements)
			for (volatile uint8_t i = 0; i < 10; i++);
			HAL_GPIO_WritePin(stepper1.STEP_PORT, stepper1.STEP_PIN, GPIO_PIN_RESET);

			stepper1.current_steps++;
			stepper1.dir ? stepper1.location_steps++ : stepper1.location_steps--;
		} else {
			// Movement complete
			stepper1.is_running = 0;
			HAL_TIM_Base_Stop_IT(stepper1.htim_stepper);
		}
	}
}

void DC_Move(uint8_t id, uint32_t counts, uint8_t dir, uint8_t back) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;
	if (!DC -> is_running) {

		DC -> counts_to_go = dir ? dc_encoder_count(id) + counts
				: dc_encoder_count(id) - counts;
		DC -> dir = dir;
		DC -> is_running = 1;
		DC -> compFlag = back;

		HAL_TIM_Base_Start_IT(DC -> htim_dc);
	}
}

uint16_t pwmCounter = 0;
uint16_t speed;
void DC_TIM_Interrupt(DCM *DC) {
	pwmCounter++;
	if(pwmCounter >= 400) pwmCounter = 0;

	if (pwmCounter < DC -> tick) HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
			DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
			DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_RESET);

	if (DC -> is_running) {
		if (DC -> dir ? dc_encoder_count(DC -> id) >= DC -> counts_to_go
				: dc_encoder_count(DC -> id) <= DC -> counts_to_go) {
			DC -> is_running = 0;
			HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
					DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_RESET);
			HAL_TIM_Base_Stop_IT(DC -> htim_dc);

			if (DC -> compFlag) {
				DC -> compFlag = 0;
				DC_Move(DC -> id, 2, !(DC -> dir), 0);
			}

//			if (abs(dc_encoder_count(DC -> id) - DC -> counts_to_go) >= 5) {
//				DC -> tick = 20;
//				DC_Move(DC -> id, 600,
//						!(DC -> dir), 0);
//			}
		}
	} else {
		HAL_TIM_Base_Stop_IT(DC -> htim_dc);
	}



//	if (DC -> is_running && speed > 1) {
//		if (DC -> dir ? dc_encoder_count(DC -> id) < DC -> counts_to_go
//				: dc_encoder_count(DC -> id) > DC -> counts_to_go)
//		{
//			HAL_GPIO_TogglePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
//					DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN);
//		}
//
//		else {
//			DC -> is_running = 0;
//			HAL_GPIO_WritePin(DC -> dir ? DC -> OUT_PORT : DC -> IN_PORT,
//								DC -> dir ? DC -> OUT_PIN : DC -> IN_PIN, GPIO_PIN_RESET);
//			HAL_TIM_Base_Stop_IT(DC -> htim_dc);
//		}
//	}
}

void DC_Reset(uint8_t id) {
	DCM *DC;
	id ? DC = &DCR : DC = &DCL;

	DC -> dir = 0;
	DC -> is_running = 1;

	dc_encoder_speed(1);
	DC_Move(id, 9600, 0, 1);
}

// Set speed by changing timer period
void Stepper_SetSpeed(uint32_t steps_per_second) {
    if(steps_per_second > 0) {
//        stepper1.step_period = (SystemCoreClock / 16) / steps_per_second;
    	stepper1.step_period = steps_per_second;
        __HAL_TIM_SET_AUTORELOAD(stepper1.htim_stepper, stepper1.step_period - 1);
    }
}

void DC_SetSpeed(uint32_t tick, uint8_t id) {
    if(tick > 0) {
    	id ? DCR.tick = tick : DCL.tick = tick;
//    	id ? DCR.dc_period = time_period : DCL.dc_period = time_period;
//        __HAL_TIM_SET_AUTORELOAD(id ? DCR.htim_dc : DCL.htim_dc, time_period - 1);
    }
}
