#include "stm32f1xx_hal.h"
#include "encoders.hpp"

#ifndef MOTORS_INC_
#define MOTORS_INC_

typedef enum {
    FULL_SPEED,    // 100% power until coast point
    COAST,         // Let physics slow it down
    BRAKE,         // Full stop at target
} motor_state_t;

typedef struct {
	TIM_HandleTypeDef *htim_stepper;

    GPIO_TypeDef* STEP_PORT;
    uint16_t STEP_PIN;
    GPIO_TypeDef* DIR_PORT;
    uint16_t DIR_PIN;
    uint32_t steps_to_go;
    uint32_t current_steps;
    int32_t location_steps;

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
    int32_t counts_to_go;
    int32_t counts_going;

    motor_state_t ms;
    uint8_t dir;
    uint8_t is_running;
    int8_t stopped;
    uint32_t period;

    uint32_t tim;
    int32_t countE;
    uint32_t speedE;
    uint16_t sum;
    int32_t error;
    uint16_t brakepoint;

    uint8_t diag;
    uint8_t duty;
    uint8_t count;
} DCM;

extern Stepper stepper;
extern DCM DCL;
extern DCM DCR;

extern uint16_t spd; extern uint8_t spdFlag;

extern uint16_t accel; extern uint8_t accelCnt;

extern uint32_t speedTim(uint8_t id);

void Stepper_Init(TIM_HandleTypeDef *htim);
void DC_Init(TIM_HandleTypeDef *htim, uint8_t id);

void Stepper_Move(int32_t steps);
void Stepper_Stop();
void Stepper_TIM_Interrupt();

void DC_Move(uint8_t id, uint32_t counts, uint8_t dir);
void DC_TIM_Interrupt(DCM *DC);
void DC_Stop(uint8_t id);
void DC_Reset(uint8_t id);

void Stepper_SetSpeed(uint32_t steps_per_second);

#endif
