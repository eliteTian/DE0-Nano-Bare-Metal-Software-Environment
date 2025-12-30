#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "alt_ethernet.h"
#include "alt_eth_phy_ksz9031.h"
#include "socal/alt_sysmgr.h"
#include "socal/alt_rstmgr.h"
#include "socal/alt_clkmgr.h"
#include "socal/alt_i2c.h"
#include "socal/hps.h"

#include "alt_printf.h"

#define TIMEOUT_VAL 1000000
#define ADXL345ADDR 0x53
#define DEVID       0x00
#define THRESH_TAP  0x1D
#define OFSX        0x1E
#define OFSY        0x1F
#define OFSZ        0x20

typedef int8_t RET_VAL ;
#define TIMEOUT -2
#define FAIL    -1
#define SUCCESS  0

void dbgRegPtr(void* addr) {
    uint32_t val;
    val = alt_read_word(addr);
    printf("Addr @ 0x%08x value is 0x%08x.\r\n", (unsigned int)(addr), (unsigned int)(val));
}

void dbgRegRaw(uint32_t addr) {
    uint32_t val;
    val = alt_read_word(addr);
    printf("Addr @ 0x%08x value is 0x%08x.\r\n", (unsigned int)(addr), (unsigned int)(val));
}

void initGPIO1(void) {
    //Init GPIO block clock
    alt_setbits_word(ALT_CLKMGR_PERPLL_EN_ADDR,ALT_CLKMGR_PERPLL_EN_GPIOCLK_SET_MSK);
    //Deassert rst
    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK);
}

void initI2C0(void) {
    //Init I2C0 block clock
    alt_setbits_word(ALT_CLKMGR_MAINPLL_EN_ADDR, ALT_CLKMGR_MAINPLL_EN_L4SPCLK_SET_MSK);
    //Deassert rst
    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR,  ALT_RSTMGR_PERMODRST_I2C0_SET_MSK);
}

void initPath(void) {
    //GPLINMUX55: //make sure it is 0 to select GENERALIO7
    alt_clrbits_word(ALT_SYSMGR_PINMUX_GPLINMUX55_ADDR, ALT_SYSMGR_PINMUX_GPLINMUX55_SEL_SET_MSK);
    //GPLINMUX56: //make sure it is 0 to select GENERALIO8
    alt_clrbits_word(ALT_SYSMGR_PINMUX_GPLINMUX56_ADDR, ALT_SYSMGR_PINMUX_GPLINMUX56_SEL_SET_MSK);
    //GPLMUX55: //make sure it is 0 to select LOAN I/O, HPS controls oe
    alt_clrbits_word(ALT_SYSMGR_PINMUX_GPLMUX55_ADDR, ALT_SYSMGR_PINMUX_GPLMUX55_SEL_SET_MSK);
    //GPLMUX56: //make sure it is 0 to select LOAN I/O, HPS controls oe
    alt_clrbits_word(ALT_SYSMGR_PINMUX_GPLMUX56_ADDR, ALT_SYSMGR_PINMUX_GPLMUX56_SEL_SET_MSK);
    //GENERALIO7: //make sure it is 1 to select I2C0
    alt_replbits_word(ALT_SYSMGR_PINMUX_GENERALIO7_ADDR,
                      ALT_SYSMGR_PINMUX_GENERALIO7_SEL_SET_MSK,
                      ALT_SYSMGR_PINMUX_GENERALIO7_SEL_SET(0x1));
    //GENERALIO8: //make sure it is 1 to select I2C0
    alt_replbits_word(ALT_SYSMGR_PINMUX_GENERALIO8_ADDR,
                      ALT_SYSMGR_PINMUX_GENERALIO8_SEL_SET_MSK,
                      ALT_SYSMGR_PINMUX_GENERALIO8_SEL_SET(0x1));
}

void i2c0MasterInit(void) {
//Per programming guide in handbook Cyclone V.
//1.Disable the I2C controller by writing 0 to bit 0 of the IC_ENABLE register. †
    alt_clrbits_word(ALT_I2C0_IC_EN_ADDR, ALT_I2C_EN_EN_SET_MSK );
//2.Write to the IC_CON register to set the maximum speed mode supported for slave operation (bits 2:1)
//and to specify whether the I2C controller starts its transfers in 7/10 bit addressing mode when the
//device is a slave (bit 3).   
    alt_setbits_word(ALT_I2C0_IC_CON_ADDR,
                     ALT_I2C_CON_MST_MOD_SET_MSK |          /*enable master mode*/
                     ALT_I2C_CON_IC_SLV_DIS_SET_MSK        /*disable slave mode*/
                    );
    alt_clrbits_word(ALT_I2C0_IC_CON_ADDR,
                     ALT_I2C_CON_IC_10BITADDR_SLV_SET_MSK |      /*7-bit addr slave mode*/
                     ALT_I2C_CON_IC_10BITADDR_MST_SET_MSK        /*7-bit addr master mode*/
                    );
    alt_replbits_word(ALT_I2C0_IC_CON_ADDR, ALT_I2C_CON_SPEED_SET_MSK, ALT_I2C_CON_SPEED_SET(ALT_I2C_CON_SPEED_E_FAST));
//3.Write to the IC_TAR register the address of the I2C device to be addressed
    alt_replbits_word(ALT_I2C0_IC_TAR_ADDR, ALT_I2C_TAR_IC_TAR_SET_MSK, ALT_I2C_TAR_IC_TAR_SET(ADXL345ADDR));

//4.Enable the I2C controller by writing a 1 in bit 0 of the IC_ENABLE register.
    alt_setbits_word(ALT_I2C0_IC_EN_ADDR, ALT_I2C_EN_EN_SET_MSK );
}

void checkStatus(uint32_t* val) {
    *val = alt_read_word(ALT_I2C0_IC_STAT_ADDR);
    printf("Status value is 0x%08x.\r\n",*val);
}

static inline void pushWrCmd(uint8_t data) {
    uint32_t cmd_val = 0;
    cmd_val |=  ALT_I2C_DATA_CMD_DAT_SET(data);
    cmd_val |=  ALT_I2C_DATA_CMD_CMD_SET(ALT_I2C_DATA_CMD_CMD_E_WR);
    alt_write_word(ALT_I2C0_IC_DATA_CMD_ADDR,cmd_val);
}

static inline void pushRdCmd(void) {
    uint32_t cmd_val = 0;
    cmd_val |=  ALT_I2C_DATA_CMD_CMD_SET(ALT_I2C_DATA_CMD_CMD_E_RD);
    alt_write_word(ALT_I2C0_IC_DATA_CMD_ADDR,cmd_val);
}

static inline void pushRdCmdStop(void) {
    uint32_t cmd_val = 0;
    cmd_val |=  ALT_I2C_DATA_CMD_CMD_SET(ALT_I2C_DATA_CMD_CMD_E_RD);
    cmd_val |=   ALT_I2C_DATA_CMD_STOP_SET(ALT_I2C_DATA_CMD_STOP_E_STOP);    
    alt_write_word(ALT_I2C0_IC_DATA_CMD_ADDR,cmd_val);
}


static inline void pushWrCmdStop(uint8_t data) { 
    uint32_t cmd_val = 0;
    cmd_val |=  ALT_I2C_DATA_CMD_DAT_SET(data);
    cmd_val |=  ALT_I2C_DATA_CMD_CMD_SET(ALT_I2C_DATA_CMD_CMD_E_WR);
    cmd_val |=   ALT_I2C_DATA_CMD_STOP_SET(ALT_I2C_DATA_CMD_STOP_E_STOP);
    alt_write_word(ALT_I2C0_IC_DATA_CMD_ADDR,cmd_val);
}


static inline void rdResult(uint8_t* data) {
    *data = ALT_I2C_DATA_CMD_DAT_GET(alt_read_word(ALT_I2C0_IC_DATA_CMD_ADDR));
}

RET_VAL readADXL345Reg(uint8_t addr, uint8_t* data) {
    uint32_t status;
    uint32_t cnt = TIMEOUT_VAL;
    pushWrCmd(addr);
    pushRdCmdStop();
    checkStatus(&status);
    while(!ALT_I2C_STAT_RFNE_GET(status) && cnt!=0) {
        checkStatus(&status);
        cnt --;
    }
    if(cnt!=0 ) {
        rdResult(data);
        printf("Addr 0x%02x read val is 0x%02x.\r\n",addr,*data);
        return SUCCESS;
    } else {
        return TIMEOUT;
    }
}

RET_VAL writeADXL345Reg(uint8_t addr, uint8_t data) {
    uint32_t status;
    pushWrCmd(addr);
    pushWrCmdStop(data);
    checkStatus(&status);
    return SUCCESS;
}


void adxl345(void) {
    uint8_t val = 0;
    uint32_t status = 0;
    initI2C0();
    initGPIO1();
    initPath();
    i2c0MasterInit();
    checkStatus(&status);
    readADXL345Reg(DEVID,&val);
    
    readADXL345Reg(THRESH_TAP,&val);
    writeADXL345Reg(THRESH_TAP,0x33);
    readADXL345Reg(THRESH_TAP,&val);
    
    readADXL345Reg(OFSX,&val);
    readADXL345Reg(OFSY,&val);
    readADXL345Reg(OFSZ,&val);
    
    checkStatus(&status);
}
