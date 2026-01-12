#include "stm32f1xx_hal.h"

#ifndef INC_ROTARY_H_
#define INC_ROTARY_H_

typedef enum {
	HOMING_DONE,
	STEP1,
	STEP2,
} homing_steps;

extern homing_steps phase;
extern uint8_t on_program;
extern uint32_t speed_default;
extern float absPos;

void home();
float rotary_move(float pos);

void Stepper2_Move(int32_t step);
void Stepper2_Stop();
void Stepper2_SetSpeed(uint16_t speed);
void Stepper2_Enable(uint8_t en);

#endif /* INC_ROTARY_H_ */
