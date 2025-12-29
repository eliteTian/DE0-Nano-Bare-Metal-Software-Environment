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

#include "alt_printf.h"
#include "hps_0.h"


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

void i2c0Config(void) {
    //set master
    //set mode. fast/standard. 7/10bit addressing
    //


void adxl345(void) {
    initI2C0();
    initGPIO1();
    initPath();
}
