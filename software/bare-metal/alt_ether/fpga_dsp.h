#ifndef ___FPGA_DSP_H___
#define ___FPGA_DSP_H___

#include <stdint.h>
#include "alt_printf.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include "hps_0.h"

#define  SRC_CTRL_OFST      0x0
#define  SRC_CTRL_CMD_VALID  0x1<<0
#define  SRC_CTRL_CMD_TYPE   0x3<<1
#define  SRC_CTRL_CMD_ADDR   0x1F<<8
#define  SRC_CTRL_CMD_DATA   0xFF<<16

#define  SRC_CTRL_CMD_VALID_OFST  0
#define  SRC_CTRL_CMD_TYPE_OFST   1
#define  SRC_CTRL_CMD_ADDR_OFST   8
#define  SRC_CTRL_CMD_DATA_OFST   16




#define  SRC_STATUS_OFST    0x4
#define  SRC_STATUS_PEND    0x1<<0
#define  SRC_STATUS_RDATA   0xFF<<8

#define  SRC_STATUS_PEND_OFST    0
#define  SRC_STATUS_RDATA_OFST   8

#define  SRC_GPR_OFST 0x8

void writeRamSource(uint8_t addr, uint8_t data);
void readRamSource(uint8_t addr, uint8_t* rdata);
void writeGPRSource(uint32_t data);
void readGPRSource(uint32_t* data);


void writeRamSink(uint8_t addr, uint8_t data);
void readRamSink(uint8_t addr, uint8_t* rdata);
void writeGPRSink(uint32_t data);
void readGPRSink(uint32_t* data);





#endif


