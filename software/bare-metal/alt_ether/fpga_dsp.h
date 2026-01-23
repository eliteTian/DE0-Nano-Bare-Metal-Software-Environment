#ifndef ___FPGA_DSP_H___
#define ___FPGA_DSP_H___

#include <stdint.h>
#include "alt_printf.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include "hps_0.h"
#include <stdlib.h>
#define  RAM_SIZE                       4096

#define  SRC_CTRL_OFST                  0x0
#define  SRC_CTRL_CMD_VALID             0x1<<0
#define  SRC_CTRL_CMD_TYPE              0x3<<1
#define  SRC_CTRL_CMD_ADDR              0xFFF<<4
#define  SRC_CTRL_CMD_DATA              0xFF<<16

#define  SRC_CTRL_CMD_VALID_OFST        0
#define  SRC_CTRL_CMD_TYPE_OFST         1
#define  SRC_CTRL_CMD_ADDR_OFST         4
#define  SRC_CTRL_CMD_DATA_OFST         16
#define  SRC_CTRL_CMD_CLR_CNT_OFST      31

#define  SRC_KEY_HOLE_REG_OFST          



#define  SRC_STATUS_OFST                0x4
#define  SRC_STATUS_PEND                0x1<<0
#define  SRC_STATUS_RDATA               0xFF<<8

#define  SRC_STATUS_PEND_OFST           0
#define  SRC_STATUS_RDATA_OFST          8

#define  SRC_GPR_OFST                   0x8
#define  SRC_DBG_OFST                   0xC

#define  DSP_CTRL_OFST                  0x0
#define  DSP_CTRL_GAIN_OFST             0x0
#define  DSP_CTRL_GAIN                  0x3

#define  DSP_COEFF0_OFST                0x8 //reg2
#define  DSP_COEFF0_TAP0_OFST           0
#define  DSP_COEFF0_TAP1_OFST           8
#define  DSP_COEFF0_TAP2_OFST           16
#define  DSP_COEFF0_TAP3_OFST           24

#define  DSP_COEFF1_OFST                0xC //reg3
#define  DSP_COEFF1_TAP4_OFST           0

#define  DSP_GPR_OFST                   0x4 //reg1

#define  MAX_DATA                       0xFFFFFFFFUL
typedef struct FPGA_DSP_s {
    void *                  location;    /*!< HPS address of I2C instance. */
}
FPGA_DSP_t;


void writeRamSource(uint16_t addr, uint8_t data);
void readRamSource(uint16_t addr, uint8_t* rdata);
void writeGPRSource(uint32_t data);
void readGPRSource(uint32_t* data);
void dumpRamSource(void);
void readDbgSource(uint32_t* data);


void writeRamSink(uint16_t addr, uint8_t data);
void readRamSink(uint16_t addr, uint8_t* rdata);
void writeGPRSink(uint32_t data);
void readGPRSink(uint32_t* data);
void setSinkForDump(void);
void readDbgSink(uint32_t* data);

void writeGPRDSP(uint32_t data);
void readGPRSDSP(uint32_t* data);

void dspSetCoeff(uint8_t tap0,uint8_t tap1,uint8_t tap2,uint8_t tap3,uint8_t tap4);
void getCoeff0(uint32_t* data);
void getCoeff1(uint32_t* data);
void dspGainSet(uint8_t gain);

void readAXISpace(uint32_t offset, uint32_t* rdata);
void writeAXISpace(uint32_t offset, uint32_t wdata);
void axiTest();

void ethSinLoop(uint8_t* eth_src, uint8_t* eth_ret, uint32_t len);
void ethCtlLed(uint8_t* rx_packet );





#endif


