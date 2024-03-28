#include <stdio.h>

#include <sys/_stdint.h>
#include <xiicps.h>
#include <xil_printf.h>
#include <xparameters.h>
#include <xstatus.h>

#include "iic.h"

#define IIC_SCLK_RATE 100000
#define IIC_DEVICE_BA XPAR_XIICPS_0_BASEADDR

#define IIC_SLAVE_ADDR 0x20

XIicPs iic1; /**< Instance of the IIC Device */

int main(void) {

  if (iicInit(&iic1, IIC_DEVICE_BA, IIC_SCLK_RATE)) { // returns 0 for success
    printf("Error initializing the iic controller\n");
    for (;;)
      ; // freez
  }

  // set direction of MAX6643_FANFAIL_B to output
  uint8_t buff[2] = {0x06, 0b11110111};
  if (iicWrite(&iic1, buff, 2, IIC_SLAVE_ADDR)) {
    printf("Failed to communicate with the io extender\n");
    for (;;)
      ; // freez
  }

  // write 0 to MAX6643_FANFAIL_B
  // send 0x02 command followed by 11110111
  buff[0] = 0x02;
  buff[1] = 0b11110111;

  if (iicWrite(&iic1, buff, 2, IIC_SLAVE_ADDR)) {
    printf("Failed to communicate with the io extender\n");
    for (;;)
      ; // freez
  }

  printf("Success hopefully!!\n");

  return 0;
}
