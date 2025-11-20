#include "hps_0.h"
#include "fpga_dsp.h"

void writeRamSource(uint8_t addr, uint8_t data){
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= addr<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= data<<SRC_CTRL_CMD_DATA_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;

    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
}

void readRamSource(uint8_t addr, uint8_t* rdata){
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= addr<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= 0x0<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;
    uint32_t temp, pend;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_STATUS_OFST);
    do{
        temp = *reg_addr;
        pend = temp & SRC_STATUS_PEND;
    } while(pend);
    
    *rdata = (temp & SRC_STATUS_RDATA) >> SRC_STATUS_RDATA_OFST ;
}


void writeRamSink(uint8_t addr, uint8_t data){
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= addr<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= data<<SRC_CTRL_CMD_DATA_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;

    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
}

void readRamSink(uint8_t addr, uint8_t* rdata){
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= addr<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= 0x0<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;
    uint32_t temp, pend;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_STATUS_OFST);
    do{
        temp = *reg_addr;
        pend = temp & SRC_STATUS_PEND;
    } while(pend);
    
    *rdata = (temp & SRC_STATUS_RDATA) >> SRC_STATUS_RDATA_OFST ;
}

void writeGPRSource(uint32_t data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_GPR_OFST);
    *reg_addr = data;
}

void readGPRSource(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_GPR_OFST);
    *data = *reg_addr;
}

void writeGPRSink(uint32_t data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_GPR_OFST);
    *reg_addr = data;
}

void readGPRSink(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_GPR_OFST);
    *data = *reg_addr;
}






