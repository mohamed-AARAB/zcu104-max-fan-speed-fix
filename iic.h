#pragma once

#include <stdint.h>

#include <xiicps.h>
#include <xil_types.h>
#include <xparameters.h>

int iicInit(XIicPs *iicInstance_ptr, UINTPTR baseAddress, u32 freqSCLHz);

int iicWriteThenRead(XIicPs *iic_instance_ptr, u8 *write_msg_ptr,
		s32 write_count, u8 *read_msg_ptr, s32 read_count, u16 slave_addr);

int iicWrite(XIicPs *iic_instance_ptr, u8 *write_msg_ptr, s32 write_count,
		u16 slave_addr);

uint8_t iicReadRegister8(XIicPs *iic_instance_ptr, uint8_t reg_addr, u16 slave_addr);

void iicWriteRegister8(XIicPs *iic_instance_ptr, uint8_t reg_addr, uint8_t value,
		u16 slave_addr);

void iicWriteBitSection(XIicPs *iic_instance_ptr, uint8_t reg_aadr,
		uint8_t bits, uint8_t shift, uint8_t value, u16 slave_addr);

uint8_t iicReadBitSection(XIicPs *iic_instance_ptr, uint8_t reg_addr,
		uint8_t bits,
		uint8_t shift, u16 slave_addr);
