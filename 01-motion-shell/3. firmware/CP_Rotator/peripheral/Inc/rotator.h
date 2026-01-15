#include "stm32f1xx_hal.h"

#ifndef INC_ROTATOR_H_
#define INC_ROTATOR_H_

typedef enum {
	HOMING_DONE,
	STEP1,
	STEP2,
} homing_steps;

extern uint8_t homing;
extern uint8_t on_program;
extern uint16_t speed_default;

void home(uint8_t dir);
void rotator_reset(uint8_t dir);
void full_home();
float rotator_move(float pos);

void Stepper2_Move(int32_t step);
void Stepper2_Stop();
void Stepper2_SetSpeed(uint16_t speed);
void Stepper2_Reset();
void Stepper2_Enable(uint8_t en);

#endif /* INC_ROTATOR_H_ */
