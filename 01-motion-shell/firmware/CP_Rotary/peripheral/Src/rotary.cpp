#include <rotary.h>
#include "stdio.h"
#include "cmath"

#include "motors.hpp"
#include "stepper_setting.hpp"
#include "can.h"

homing_steps phase = HOMING_DONE;
int16_t stepLock;
uint8_t on_program = 0;
uint32_t speed_default = 300;

void home() {
	if (phase == HOMING_DONE) {
		phase = STEP1;
		Stepper_Move(-INFI);
	} else if (phase == STEP1){
		phase = STEP2;
		Stepper_Stop();
		stepper.location_steps = 0;
		Stepper_Move(INFI);
	} else {
		phase = HOMING_DONE;
		Stepper_Stop();

		if (stepper.location_steps > (int32_t)roundf((360.0f / 31.0f) * USTEP_RATE * 100)) stepper.location_steps /= 2;
		else stepper.location_steps = (stepper.location_steps / 2) + (int32_t)roundf((360.0f / 31.0f) * USTEP_RATE * 100);

		uint8_t packet[8];
		packet[0] = 0x21;
		CAN_Tx(packet, 8, MASTER_ID);
	}
}

float rotary_move(float pos) {
	int32_t target = (int32_t)roundf((360.0f / 31.0f) *
			(USTEP_RATE * 200.0f) * pos / 360.0f);
	on_program = 1;

	Stepper_Move(target - stepper.location_steps);
	return (float)(target) / (float)(360.0f / 31.0f * USTEP_RATE * 200) * 360.0f;
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
} void Stepper2_Enable(uint8_t en) {
	uint8_t packet[8];

	packet[0] = 0x12;
	packet[1] = en;

	CAN_Tx(packet, 8, SLAVE_ID);
}










