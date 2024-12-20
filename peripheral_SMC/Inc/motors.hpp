#include "stm32f1xx_hal.h"
#include "encoders.hpp"

#ifndef MOTORS_INC_
#define MOTORS_INC_

typedef struct {
	TIM_HandleTypeDef *htim_stepper;

    GPIO_TypeDef* STEP_PORT;
    uint16_t STEP_PIN;
    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    uint32_t steps_to_go;
    uint32_t current_steps;
    uint32_t location_steps;

    uint8_t dir;
    uint8_t is_running;
    uint32_t step_period;  // timer ticks per step
} Stepper;

typedef struct {
	uint8_t id;
	TIM_HandleTypeDef *htim_dc;

    GPIO_TypeDef* IN_PORT;
    uint16_t IN_PIN;
    GPIO_TypeDef* OUT_PORT;
    uint16_t OUT_PIN;
    int16_t counts_to_go;

    uint8_t dir;
    uint8_t is_running;
    uint32_t dc_period;
    uint8_t tick;
    uint8_t compFlag;
} DCM;

extern Stepper stepper1;
extern DCM DCL;
extern DCM DCR;

void Stepper_Init(TIM_HandleTypeDef *htim);
void DC_Init(TIM_HandleTypeDef *htim, uint8_t id);

void Stepper_Move(uint32_t steps, uint8_t dir);
void Stepper_Stop();
void Stepper_TIM_Interrupt();

void DC_Move(uint8_t id, uint32_t counts, uint8_t dir, uint8_t back);
void DC_TIM_Interrupt(DCM *DC);
void DC_Reset(uint8_t id);

void Stepper_SetSpeed(uint32_t steps_per_second);
void DC_SetSpeed(uint32_t time_period, uint8_t id);

#endif
