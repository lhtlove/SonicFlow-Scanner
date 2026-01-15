#include "sF.h"
#include "stdio.h"
#include "can.h"

uint32_t echo_timer_value = 0;
uint8_t echo_detected = 0;
uint8_t measurement_active = 0;
volatile uint8_t awd_count = 0;  // 추가

void DDS_Off(void)
{
	writeRegister(0x2080);
}

void DDS_On(void)
{
	writeRegister(0x2000);
}

void DDS_Sleep(void)
{
	writeRegister(0x20C0);
}

void DDS_Wakeup(void)
{
	writeRegister(0x2000);
}

uint32_t CalcFTW(uint32_t freq)
{
	uint32_t FTW;
	float refFreq = 16000000;
	float FTWResolution = 0x10000000;

	FTW = (FTWResolution / refFreq) * freq;
	return FTW;
}

void writeRegister(uint16_t data)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}

void sineWave(uint32_t FTW)
{
	int MSB;
	int LSB;
	int phase = 0;

	MSB = (uint32_t)((FTW & 0xFFFC000) >> 14);
	LSB = (uint32_t)(FTW & 0x3FFF);

	LSB |= 0x4000;
	MSB |= 0x4000;
	phase &= 0xC000;

	writeRegister(0x2100);
	writeRegister(LSB);
	writeRegister(MSB);
	writeRegister(phase);

	writeRegister(0x2000);
}

void DDS_Init(void)
{
	HAL_Delay(50);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_Delay(10);

	writeRegister(0x2100);
	HAL_Delay(10);
}

void Ultrasonic_SendBurst(uint32_t cycles)
{
	DDS_Wakeup();
	sineWave(CalcFTW(BURST_FREQUENCY));

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);

	DDS_On();

	uint32_t burst_us = (cycles * 1000000UL) / BURST_FREQUENCY;
	delayUs(burst_us);

	DDS_Off();
}

void Ultrasonic_StartMeasurement(void)
{
	echo_detected = 0;
	echo_timer_value = 0;
	measurement_active = 1;
	awd_count = 0;

//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	HAL_TIM_Base_Start(&htim2);

	HAL_ADC_Start_IT(&hadc1);
}

void Ultrasonic_StopMeasurement(void)
{
	measurement_active = 0;
	HAL_ADC_Stop_IT(&hadc1);
	HAL_TIM_Base_Stop(&htim2);

	uint8_t packet[8];
	packet[0] = 0x40; send_int32(echo_timer_value, packet + 1);
	CAN_Tx(packet, 8, DEVICE_MASTER);

//	printf("detected!: %d\n", echo_timer_value);
}

void delayUs (uint32_t us) {
	__HAL_TIM_SET_COUNTER (&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{

    if (hadc->Instance == ADC1 && measurement_active)
    {
//        echo_timer_value = __HAL_TIM_GET_COUNTER(&htim2);
//        echo_detected = 1;
//        Ultrasonic_StopMeasurement();
    	awd_count++;
    	if (awd_count >= AWD_THRESHOLD_COUNT) {
			uint8_t packet[8];
			packet[0] = 0x13;
			CAN_Tx(packet, 8, 0x401);

			measurement_active = 0;
			HAL_ADC_Stop_IT(&hadc1);
    	}
    }
}
