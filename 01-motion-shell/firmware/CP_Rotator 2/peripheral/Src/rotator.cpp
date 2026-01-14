#include "stdio.h"
#include "cmath"

#include "rotator.h"
#include "motors.hpp"
#include "stepper_setting.hpp"
#include "can.h"

uint8_t homing = 0;
uint8_t on_program = 0;

int16_t stepLock;
homing_steps step = HOMING_DONE;
uint16_t speed_default = 300;

void home(uint8_t dir) {
	homing = 1;

	if (!dir) {
		Stepper_SetSpeed(250);
		write(0x40, 75);
		currentSetup(driver_current_, hold_current, stall_current);
		Stepper_Move(-INFI);
	} else Stepper_Move(1000);
}

void rotator_reset(uint8_t dir) {
	if (!dir) {
		Stepper_Stop();
		Stepper_Reset();
	}

	write(0x40, 50);
	currentSetup(driver_current_, hold_current, current);
	Stepper_SetSpeed(speed_default);

	homing = 0;

//	uint8_t packet[8];
//	packet[0] = 0x31;
//	send_int16(abs(stepper.location_steps), packet + 1);
//	CAN_Tx(packet, 8, MASTER_ID);
//	printf("pos: %d\n", stepper.location_steps);

	if (!stepper.dir) stepper.location_steps = -1000;
//	else stepper.location_steps = 0;

	if (step == STEP1) {
		step = STEP2;
		home(0);
	} else if (step == STEP2) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		step = HOMING_DONE;
	}
}

void full_home() {
	if (step == HOMING_DONE) {
		uint8_t send[8];
		send[0] = 0x22;
		CAN_Tx(send, 8, SLAVE_ID);

		step = STEP1;
		home(1);
	}
}

float rotator_move(float pos) {
	if (pos < 0) pos = 0;
	if (pos > 90) pos = 90;
	uint16_t target = (uint16_t)(((USTEP_RATE * 600) * pos) / 90.0f + 0.5f);

	on_program = 1;
	if (pos == 0) {
		Stepper_Move(-1000 - stepper.location_steps);
		return 0.0f;
	} else {
		Stepper_Move(target - stepper.location_steps);
		return (float)(target * 90) / (float)(USTEP_RATE * 600);
	}
}

void Stepper2_Move(int32_t step) {
	uint8_t packet[8];

	packet[0] = 0x10;
	send_int32(abs(step), packet + 1);
	packet[5] = step < 0 ? 0 : 1;

	CAN_Tx(packet, 8, SLAVE_ID);
} void Stepper2_Stop() {
	uint8_t packet[8];

	packet[0] = 0x11;

	CAN_Tx(packet, 8, SLAVE_ID);
} void Stepper2_SetSpeed(uint16_t speed) {
	uint8_t packet[8];

	packet[0] = 0x14;
	send_int16(speed, packet + 1);

	CAN_Tx(packet, 8, SLAVE_ID);
} void Stepper2_Reset() {
	uint8_t packet[8];

	packet[0] = 0x13;

	CAN_Tx(packet, 8, SLAVE_ID);
} void Stepper2_Enable(uint8_t en) {
	uint8_t packet[8];

	packet[0] = 0x12;
	packet[1] = en;

	CAN_Tx(packet, 8, SLAVE_ID);
}





