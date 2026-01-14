#include "cmath"

#include "smc.hpp"
#include "motors.hpp"
#include "can.hpp"
#include "stepper_setting.hpp"

homing_steps phase = HOMING_DONE;
swap_steps swap = DONE;

uint8_t on_program = 0;
uint8_t dcl_program = 0;
uint8_t dcr_program = 0;

int8_t side = 0;
float target_ori;

uint8_t packet[8];

void setup(uint8_t id) {
	if (!id) id = DEVICE_ID & 0xF;
	currentSetup(driver_current_, hold_current, current);

	switch (id) {
	case 1:
		stepper.location_steps = 90133; // 84.5/360
		break;
	case 2:
		stepper.location_steps = 75733; // 71/360
		break;
	case 3:
		stepper.location_steps = 61333; // 57.5/360
		break;
	case 4:
		stepper.location_steps = 46933; // 44/360
		break;
	}
} void module_move(float pos) {
	int32_t target = (int32_t)roundf((360.0f / 6.0f) *
			(32 * 200)* pos / 360.0f);
	on_program = 1;

	Stepper_Move(target - stepper.location_steps);
} void module_placed() {
	on_program = 0;

	packet[0] = 0x30;
	CAN_Tx(packet, 8, DEVICE_MASTER);
}

void dc_home() {
	if (phase == HOMING_DONE) {
		phase = STEP1;

		DC_Reset(0);
		DC_Reset(1);
	} else if (phase == STEP1) {
		phase = STEP2;
		DC_Move(0, 200, 1);
	} else if (phase == STEP2) {
		phase = STEP3;
		DC_Move(1, 200, 1);
	} else {
		phase = HOMING_DONE;

		packet[0] = 0x31;
		CAN_Tx(packet, 8, DEVICE_MASTER);
	}
} void dc_orient(float ori) {
	if (ori > M_PI / 3) ori = M_PI / 3;
	if (ori < -M_PI / 3) ori = -M_PI / 3;
	int16_t target = abs((int16_t)roundf(tan(ori) * 8 / 20.082 * 1800));

	uint8_t id = ori < 0 ? 1 : 0;

	if (side == (id ? -1 : 1) || !side) {
		id ? dcr_program = 1 : dcl_program = 1;
		if (swap == SWAP) swap = DONE;
		side = id ? -1 : 1;

		DC_Move(id, abs(target - (id ? DCR.countE : DCL.countE)),
				(target - (id ? DCR.countE : DCL.countE)) > 0 ? 1 : 0);
	} else if (!ori || swap == SWAP) {
		side == -1 ? id = 1 : id = 0;
		if (!ori) id ? dcr_program = 1 : dcl_program = 1;
		else target_ori = ori;
		side = 0;

		DC_Move(id, abs(-(id ? DCR.countE : DCL.countE)),
				-(id ? DCR.countE : DCL.countE) > 0 ? 1 : 0);
	} else {
		swap = SWAP;
		dc_orient(ori);
	}
} void dc_placed(uint8_t id) {
	if (!id) {
		dcl_program = 0;

		packet[0] = 0x32; send_int16(DCL.countE, packet + 2);
		CAN_Tx(packet, 8, DEVICE_MASTER);
	} else {
		dcr_program = 0;

		packet[0] = 0x33; send_int16(DCR.countE, packet + 2);
		CAN_Tx(packet, 8, DEVICE_MASTER);
	}
}













