#include "can.hpp"


void send_int32(int32_t value, uint8_t *data) {
    data[0] = (value >> 24) & 0xFF;  // MSB
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;          // LSB
}

int32_t receive_int32(uint8_t* data) {
    return ((int32_t)data[0] << 24) |
           ((int32_t)data[1] << 16) |
           ((int32_t)data[2] << 8) |
            (int32_t)data[3];
}

uint8_t replyData[8];
void stp(uint8_t* data, uint8_t len) {
	if (data[1] == 0x00) stepper1.location_steps = 0;

	else if (data[1] == 0x01) Stepper_Move(receive_int32(data + 3), data[2]);
	else if (data[1] == 0x02) Stepper_Stop();
	else if (data[1] == 0x03) Stepper_SetSpeed(receive_int32(data + 3));
}

void dc(uint8_t* data, uint8_t len) {
	if (data[1] == 0x00) DC_SetSpeed(data[3], data[2]);
	else if (data[1] == 0x01) DC_Move(data[2], receive_int32(data + 4), data[3], 1);
	else if (data[1] == 0x02) data[2] ? DCR.is_running = 0 : DCL.is_running = 0;
}

void enc(uint8_t* data, uint8_t len) {
	if (data[1] == 0x00) dc_encoder_reset();
	else if (data[1] == 0x01) {
		replyData[0] = data[0];
		replyData[1] = data[1];
		replyData[2] = data[2];
		replyData[3] = dc_encoder_count(data[2]);

		CAN_Tx(replyData, 8, DEVICE_ID);
	} else if (data[1] == 0x02) {
		replyData[0] = data[0];
		replyData[1] = data[1];
		replyData[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
		replyData[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
		replyData[4] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		replyData[5] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

		CAN_Tx(replyData, 8, DEVICE_ID);
	}

	else if (data[1] == 0x10) {
		replyData[0] = data[0];
		replyData[1] = data[1];
		replyData[2] = data[2];

		uint32_t scaled = (uint32_t)(stepper_encoder_angle(data[2]) * 1000);
		send_int32(scaled, replyData + 3);
	}
}

CANHandler handlers[10];
//handlers
////handlers[1] = {0x01, dc};
////handlers[2] = {0x02, enc};

const uint8_t handlers_length = sizeof(handlers) / 16;

CAN_HandleTypeDef *hCan;

void CAN_INIT(CAN_HandleTypeDef *hcan, uint16_t device_id, uint16_t mask_id) {
    CAN_FilterTypeDef filter;

    filter.FilterIdHigh 		= device_id << 5;
    filter.FilterMaskIdHigh 	= mask_id << 5;        // 0x7FF -> exact match filter
//    filter.FilterIdLow = device_id << 5;
//    filter.FilterMaskIdLow= mask_id << 5;
    filter.FilterScale 		= CAN_FILTERSCALE_32BIT;
    filter.FilterMode 		= CAN_FILTERMODE_IDMASK;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank 		= 0;
    filter.FilterActivation 	= ENABLE;

    HAL_CAN_ConfigFilter(hcan, &filter);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    hCan = hcan;

    handlers[0].id = 0x00; handlers[0].handler = stp;
    handlers[1].id = 0x01; handlers[1].handler = dc;
    handlers[2].id = 0x02; handlers[2].handler = enc;
}

// Send data
void CAN_Tx(uint8_t *data, uint8_t len, uint16_t device_id) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = device_id;
    TxHeader.RTR = CAN_RTR_DATA;         // Normal data frame
    TxHeader.IDE = CAN_ID_STD;           // Standard ID
    TxHeader.DLC = len;                  // Data length
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hCan, &TxHeader, data, &TxMailbox);
}

// Receive callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
    	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
//        if(RxHeader.StdId == CAN_DEVICE_ID_2) Process(data, RxHeader.DLC);
    	for (int h = 0; h < handlers_length; h++) {
    		if (handlers[h].id == data[0]) handlers[h].handler(data, RxHeader.DLC);
    	}
    }
}
