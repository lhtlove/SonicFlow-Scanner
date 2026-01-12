#include "stm32f1xx_hal.h"

#ifndef INC_SMC_HPP_
#define INC_SMC_HPP_

typedef enum {
	HOMING_DONE,
	STEP1,
	STEP2,
	STEP3,
} homing_steps;
typedef enum {
	DONE,
	SWAP,
} swap_steps;

extern homing_steps phase;
extern swap_steps swap;

extern uint8_t on_program;
extern uint8_t dcl_program;
extern uint8_t dcr_program;

extern int8_t side;
extern float target_ori;

void setup(uint8_t id);
void module_move(float pos);
void module_placed();

void dc_home();
void dc_orient(float ori);
void dc_placed(uint8_t id);


#endif /* INC_SMC_HPP_ */
