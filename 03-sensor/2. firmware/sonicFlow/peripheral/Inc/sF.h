#ifndef SF_H
#define SF_H

#include "stm32g4xx_hal.h"

#define BURST_FREQUENCY     300000
#define AWD_THRESHOLD_COUNT  20

extern uint32_t echo_timer_value;
extern uint8_t echo_detected;
extern uint8_t measurement_active;

extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

void writeRegister(uint16_t data);
uint32_t CalcFTW(uint32_t freq);
void sineWave(uint32_t FTW);

void DDS_Init(void);
void DDS_Off(void);
void DDS_On(void);
void DDS_Sleep(void);
void DDS_Wakeup(void);

void Ultrasonic_SendBurst(uint32_t cycles);
void Ultrasonic_StartMeasurement(void);
void Ultrasonic_StopMeasurement(void);

void delayUs(uint32_t us);

#endif
