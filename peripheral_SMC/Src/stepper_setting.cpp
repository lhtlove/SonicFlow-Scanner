#include "stepper_setting.hpp"

void gconfSetup(GlobalConfig & gc) {
    gc.bytes                = 0;
    gc.multistep_filt       	= 1;

    gc.i_scale_analog       = 1;
    gc.internal_rsense      = 1;
    gc.enable_spread_cycle  = 0;
    gc.pdn_disable          = 1;
    gc.mstep_reg_select     = 1;
}

void cconfSetup(ChopperConfig & cc) {
	cc.bytes = 0;

	cc.toff		= 3;
	cc.hstart	= 4;
	cc.hend		= 0;
	cc.tbl		= 1;
	cc.vsense	= 0;
	cc.mres		= 0b0011;
	cc.interpolation = 1;
}

void currentSetup(DriverCurrent & dc) {
	dc.bytes	= 0;

	dc.ihold	= 0;
	dc.irun		= 31;
	dc.iholddelay = 0;
}

template<typename Datagram>
void sendDatagramUnidirectional(Datagram & datagram, uint8_t datagram_size, UART_HandleTypeDef *huart)
{
    uint8_t byte;

    for (uint8_t i=0; i<datagram_size; ++i)
    {
        byte = (datagram.bytes >> (i * 8)) & 0xFF;
        //printf("0x%x, %lu / ", byte, sizeof(byte));
        HAL_UART_Transmit(huart, &byte, 1, 100);
        //serialWrite(byte);
    }
}

template<typename Datagram>
uint8_t calculateCrc(Datagram & datagram,
                              uint8_t datagram_size)
{
    uint8_t crc = 0;
    uint8_t byte;
    for (uint8_t i=0; i<(datagram_size - 1); ++i)
    {
        byte = (datagram.bytes >> (i * 8)) & 0xFF;
        for (uint8_t j=0; j<8; ++j)
        {
            if ((crc >> 7) ^ (byte & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = crc << 1;
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

uint32_t reverseData(uint32_t data)
{
    uint32_t reversed_data = 0;
    uint8_t right_shift;
    uint8_t left_shift;
    for (uint8_t i=0; i<DATA_SIZE; ++i)
    {
        right_shift = (DATA_SIZE - i - 1) * 8;
        left_shift = i * 8;
        reversed_data |= ((data >> right_shift) & 0xFF) << left_shift;
    }
    return reversed_data;
}

void write(uint8_t register_address, uint32_t data, UART_HandleTypeDef *huart)
{
	WriteReadReplyDatagram write_datagram;
    //WriteReadReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = 0x00;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = reverseData(data);
    write_datagram.crc = calculateCrc(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

    sendDatagramUnidirectional(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE, huart);
}
