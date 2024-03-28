#include "iic.h"
#include <stdint.h>
#include <stdlib.h>

#include <xiicps.h>
#include <xparameters.h>

/************************ Functions definitions ***************************/

/**
 *
 * Function to initialize the IIC device
 *
 * @return	XST_SUCCESS if successful, otherwise XST_FAILURE.
 *
 * @note		None.
 *
 */
int iicInit(XIicPs *iicInstance_ptr, UINTPTR baseAddress, u32 freqSCLHz) {
  int status;
  XIicPs_Config *config;

  /*
   * Initialize the IIC driver so that it's ready to use
   * Look up the configuration in the config table,
   * then initialize it.
   */
  config = XIicPs_LookupConfig(baseAddress);
  if (NULL == config) {
    return XST_FAILURE;
  }

  status = XIicPs_CfgInitialize(iicInstance_ptr, config, config->BaseAddress);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  /*
   * Perform a self-test to ensure that the hardware was built correctly.
   */
  status = XIicPs_SelfTest(iicInstance_ptr);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  /*
   * Set the IIC serial clock rate.
   */
  XIicPs_SetSClk(iicInstance_ptr, freqSCLHz);

  // enable 10 bit addressing
  // status = XIicPs_SetOptions(&iic_instance, XIICPS_10_BIT_ADDR_OPTION);
  // if (status != XST_SUCCESS) {
  //   return XST_FAILURE;
  // }

  return XST_SUCCESS;
}

/**
 * Function to write to the IIC slave then read from it with repeated
 * start
 *
 * @param iic_instance_ptr is the pointer to an instance of the IIC device
 * @param write_msg_ptr is the pointer to the message to be written to the
 * 			specified register
 * @param write_count is the number of bytes to be written
 * @param read_msg_ptr is the pointer to where the message will be read
 * @param read_count is the number of bytes to be read
 * @param slave_addr
 * @return XST_SUCCESS if it succeeds or XST_FAILURE otherwise
 */
int iicWriteThenRead(XIicPs *iic_instance_ptr, u8 *write_msg_ptr,
                     s32 write_count, u8 *read_msg_ptr, s32 read_count,
                     u16 slave_addr) {
  int status;

  // wait for the IIC bus to be available
  while (XIicPs_BusIsBusy(iic_instance_ptr))
    ;
  status = XIicPs_SetOptions(iic_instance_ptr, XIICPS_REP_START_OPTION);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  while (XIicPs_BusIsBusy(iic_instance_ptr))
    ;

  status = XIicPs_MasterSendPolled(iic_instance_ptr, write_msg_ptr, write_count,
                                   slave_addr);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  status = XIicPs_ClearOptions(iic_instance_ptr, XIICPS_REP_START_OPTION);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  status = XIicPs_MasterRecvPolled(iic_instance_ptr, read_msg_ptr, read_count,
                                   slave_addr);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  return XST_SUCCESS;
}

/**
 * Function to write to a specific register of the IIC slave
 *
 * @param iic_instance_ptr is the pointer to an instance of the IIC device
 * @param write_msg_ptr is the pointer to the message to be written to the
 * 			specified register
 * @param write_count is the number of bytes to be written
 * @param slave_addr is the slave address
 * @return XST_SUCCESS if it succeeds or XST_FAILURE otherwise
 */
int iicWrite(XIicPs *iic_instance_ptr, u8 *write_msg_ptr, s32 write_count,
             u16 slave_addr) {
  int status;

  status = XIicPs_MasterSendPolled(iic_instance_ptr, write_msg_ptr, write_count,
                                   slave_addr);
  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  return XST_SUCCESS;
}

/**
 * High level function to read a specific one byte register from the IIC slave
 *
 * @param iic_instance_ptr is the pointer to an instance of the IIC device
 * @param reg is the address of the register to be read
 * @param slave_addr is the slave address
 * @return the value read from the register
 */
uint8_t iicReadRegister8(XIicPs *iic_instance_ptr, uint8_t reg_aadr,
                         u16 slave_addr) {
  uint8_t buff = reg_aadr;
  iicWriteThenRead(iic_instance_ptr, &buff, 1, &buff, 1, slave_addr);
  return buff;
}

/**
 *
 * High level function to write a value a specific one byte register from
 * the IIC slave
 *
 * @param iic_instance_ptr is the pointer to an instance of the IIC device
 * @param reg is the address of the register to be written to
 * @param value is the value to be written to the register
 * @param slave_addr is the slave address
 */
void iicWriteRegister8(XIicPs *iic_instance_ptr, uint8_t reg_addr,
                       uint8_t value, u16 slave_addr) {
  uint8_t buff[2] = {reg_addr, value};
  iicWrite(iic_instance_ptr, buff, 2, slave_addr);
}

uint8_t iicReadBitSection(XIicPs *iic_instance_ptr, uint8_t reg_addr,
                          uint8_t bits, uint8_t shift, u16 slave_addr) {
  uint8_t val = iicReadRegister8(iic_instance_ptr, reg_addr, slave_addr);

  val >>= shift;
  return val & ((1 << (bits)) - 1);
}

void iicWriteBitSection(XIicPs *iic_instance_ptr, uint8_t reg_addr,
                        uint8_t bits, uint8_t shift, uint8_t value,
                        u16 slave_addr) {

  uint8_t reg_data = iicReadRegister8(iic_instance_ptr, reg_addr, slave_addr);

  // mask off the data before writing
  uint8_t mask = (1 << (bits)) - 1;
  value &= mask;

  mask <<= shift;
  reg_data &= ~mask;          // remove the current data at that spot
  reg_data |= value << shift; // and add in the new data

  iicWriteRegister8(iic_instance_ptr, reg_addr, reg_data, slave_addr);
}
