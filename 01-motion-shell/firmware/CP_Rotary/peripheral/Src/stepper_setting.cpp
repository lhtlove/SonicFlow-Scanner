#include "stepper_setting.hpp"
#include "stdio.h"

GlobalConfig global_config_;
ChopperConfig chopper_config_;
DriverCurrent driver_current_;
CoolConfig cool_config_;

uint16_t sg_result = 0;
uint8_t current = 31, stall_current = 16, hold_current = 8;

void gconfSetup(GlobalConfig & gc) {
    gc.bytes                = 0;
    gc.multistep_filt       	= 1;

    gc.i_scale_analog       = 0;
    gc.internal_rsense      = 0;
    gc.enable_spread_cycle  = 0;
    gc.pdn_disable          = 1;
    gc.mstep_reg_select     = 1;

    write(ADDRESS_GCONF, gc.bytes);
}

void cconfSetup(ChopperConfig & cc, uint8_t ustep) {
	cc.bytes = 0;

	cc.toff		= 3;
	cc.hstart	= 4;
	cc.hend		= 0;
	cc.tbl		= 1;
	cc.vsense	= 0;
	cc.mres		= ustep;
	cc.interpolation = 1;

	write(ADDRESS_CHOPCONF, cc.bytes);
}

void currentSetup(DriverCurrent & dc, uint8_t hold, uint8_t run) {
	dc.bytes	= 0;

	dc.ihold	= hold;
	dc.irun		= run;
	dc.iholddelay = 0;

	write(ADDRESS_IHOLD_IRUN, dc.bytes);
}

void coolSetup(CoolConfig & cs) {
	cs.bytes = 0;

	cs.semin = 5;
	cs.seup = 0b10;
	cs.semax = 6;
	cs.sedn = 0b10;
	cs.seimin = 0;

	write(ADDRESS_COOLCONF, cs.bytes);
}

template<typename Datagram>
void sendDatagramUnidirectional(Datagram & datagram, uint8_t datagram_size)
{
    uint8_t byte;

    for (uint8_t i=0; i<datagram_size; ++i)
    {
        byte = (datagram.bytes >> (i * 8)) & 0xFF;
        //printf("0x%x, %lu / ", byte, sizeof(byte));
        HAL_UART_Transmit(&huart3, &byte, 1, 100);
        //serialWrite(byte);
    }
}

template<typename Datagram>
uint8_t calculateCrc(Datagram & datagram, uint8_t datagram_size)
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

uint32_t reverseData(uint32_t data) // 0x12345678 -> 0x78563412 : byte unit reverse
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

void print_bin64(uint64_t val) {
    printf("bytes: ");
    for (int i = 0; i < 64; i++) {
        printf("%d", (int)((val >> i) & 1));
        if ((i + 1) % 8 == 0 && i != 63) printf(" ");
    }
    printf("\n");
}

void print_bin32(uint64_t val) {
    printf("bytes: ");
    for (int i = 0; i < 32; i++) {
        printf("%d", (int)((val >> i) & 1));
        if ((i + 1) % 8 == 0 && i != 63) printf(" ");
    }
    printf("\n");
}

void write(uint8_t register_address, uint32_t data)
{
	WriteReplyDatagram write_datagram;
    write_datagram.bytes = 0;
    write_datagram.sync = SYNC;
    write_datagram.serial_address = 0x00;
    write_datagram.register_address = register_address;
    write_datagram.rw = RW_WRITE;
    write_datagram.data = reverseData(data);
    write_datagram.crc = calculateCrc(write_datagram, WRITE_REPLY_DATAGRAM_SIZE);

//    print_bin64(write_datagram.bytes);
    sendDatagramUnidirectional(write_datagram, WRITE_REPLY_DATAGRAM_SIZE);
}

void read(uint8_t register_address)
{
	ReadDatagram read_datagram;
	read_datagram.bytes = 0;
	read_datagram.sync = SYNC;
	read_datagram.serial_address = 0x00;
	read_datagram.register_address = register_address;
	read_datagram.rw = RW_READ;
	read_datagram.crc = calculateCrc(read_datagram, READ_DATAGRAM_SIZE);

//	print_bin32(read_datagram.bytes);
    sendDatagramUnidirectional(read_datagram, READ_DATAGRAM_SIZE);
    HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, 8);
}

void received() {
//	printf("syncTest: %b\n", reply_buffer.sync);

//	if (calculateCrc(reply_buffer, READ_DATAGRAM_SIZE) != reply_buffer.crc) {
//
//		return;
//	}

	if (reply_buffer.register_address == 0x41) {
		sg_result = reverseData(reply_buffer.data);
//		printf("sg_result: %d\n", sg_result);
	} else if (reply_buffer.register_address == 0x12) {
		printf("TSTEP: %d\n", reverseData(reply_buffer.data));
	}
}
