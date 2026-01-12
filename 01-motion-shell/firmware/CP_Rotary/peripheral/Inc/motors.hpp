#include "stm32f1xx_hal.h"

#ifndef MOTORS_INC_
#define MOTORS_INC_

#define INFI 2147000000

typedef struct {
	TIM_HandleTypeDef *htim_stepper;

    GPIO_TypeDef* STEP_PORT;
    uint16_t STEP_PIN;
    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    GPIO_TypeDef* ENN_PORT;
    uint16_t ENN_PIN;

    uint32_t steps_to_go;
    uint32_t current_steps;
    int32_t location_steps;
    uint64_t time;

    uint8_t dir;
    uint8_t is_running;
    uint8_t is_decelerating;
    uint32_t step_period;  // timer ticks per step
} Stepper;
extern Stepper stepper;

extern uint16_t accel_rate;
extern uint16_t accel_th;

void Stepper_Init(TIM_HandleTypeDef *htim);

uint8_t Stepper_Move(int32_t steps);
void Stepper_Stop();
void Stepper_TIM_Interrupt();

void Stepper_Enable(uint8_t toggle);
void Stepper_Reset();
void Stepper_SetSpeed(uint32_t steps_per_second);

#endif
