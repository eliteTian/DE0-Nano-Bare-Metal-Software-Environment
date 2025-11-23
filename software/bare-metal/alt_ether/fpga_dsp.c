#include "fpga_dsp.h"

static uint8_t pseudoRand(uint8_t seed)
{
    uint8_t lfsr = seed ? seed : 1; // avoid zero lockup
    uint8_t bit;

    // Perform 8 steps to scramble thoroughly
    for(int i=0;i<8;i++) {
        bit = ((lfsr >> 7) ^ (lfsr >> 5) ^ (lfsr >> 4) ^ (lfsr >> 3)) & 1;
        lfsr = (lfsr << 1) | bit;
    }
    return lfsr;
}

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


void readCTRLSource(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_CTRL_OFST);
    *data = *reg_addr;
}

void readCTRLSink(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_CTRL_OFST);
    *data = *reg_addr;
}



void readDbgSource(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_DBG_OFST);
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

void readDbgSink(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_DBG_OFST);
    *data = *reg_addr;
}




void dumpRamSource(void) {
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= 0x2<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
}

void dumpRamSink(void) {
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= 0x2<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
}


void writeGPRDSP(uint32_t data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_GPR_OFST);
    *reg_addr = data;
}

void readGPRDSP(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_GPR_OFST);
    *data = *reg_addr;
}



void gprTest(void) {
    uint32_t gpr;
    readGPRSource(&gpr);
    printf("General Purpose Register is before writing : 0x%08x\r\n", gpr );    
    writeGPRSource(0x26571489);
    readGPRSource(&gpr);
    printf("General Purpose Register is after writing : 0x%08x\r\n", gpr );

    readGPRSink(&gpr);
    printf("General Purpose Register is before writing : 0x%08x\r\n", gpr );    
    writeGPRSink(0x26571489);
    readGPRSink(&gpr);
    printf("General Purpose Register is after writing : 0x%08x\r\n", gpr );

    readGPRDSP(&gpr);
    printf("General Purpose Register is before writing : 0x%08x\r\n", gpr );    
    writeGPRDSP(0x26571489);
    readGPRDSP(&gpr);
    printf("General Purpose Register is after writing : 0x%08x\r\n", gpr );


}

static void ramWriteTestSrc(void) {
    uint8_t index;
    uint8_t rdata = 0;
    uint8_t wdata;
    uint8_t addr;

    for(index = 0; index < 32; index++) {
        addr = index; 
        wdata = pseudoRand(index);
        writeRamSource(addr, wdata);
    }
    for(index = 0; index < 32; index++) {
        addr = index; 
        readRamSource(addr,&rdata);
        if(rdata!=pseudoRand(index)) {
            printf("Source RAM data mismatch: 0x%02x\r\n", rdata );
        }
    }

}

void ramWriteTestSnk(void) {
    uint8_t index;
    uint8_t rdata = 0;
    uint8_t wdata;
    uint8_t addr;

    for(index = 0; index < 32; index++) {
        addr = index; 
        wdata = 32-index;
        writeRamSink(addr, wdata);
    }
    for(index = 0; index < 32; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        if(rdata!=32-index) {
            //printf("Sink ram before dump writing : 0x%02x\r\n", rdata );
            printf("Sink RAM data mismatch: 0x%02x\r\n", rdata );        
        }
    }

}

static void ramReadSnk(void) {
    uint8_t index;
    uint8_t rdata = 0;
    uint8_t addr;

    for(index = 0; index < 32; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        printf("Sink RAM data read is : 0x%02x\r\n", rdata );     
        //if(rdata!=32-index) {
        //    //printf("Sink ram before dump writing : 0x%02x\r\n", rdata );
        //    printf("Sink RAM data mismatch: 0x%02x\r\n", rdata );        
        //}
    }

}


void dspSetCoeff(uint8_t tap0,uint8_t tap1,uint8_t tap2,uint8_t tap3,uint8_t tap4) {
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= (tap0&0xFF)<<DSP_COEFF0_TAP0_OFST;
    reg_val |= (tap1&0xFF)<<DSP_COEFF0_TAP1_OFST;
    reg_val |= (tap2&0xFF)<<DSP_COEFF0_TAP2_OFST;
    reg_val |= (tap3&0xFF)<<DSP_COEFF0_TAP3_OFST;
    reg_addr = (volatile uint32_t* ) (ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_COEFF0_OFST);
    *reg_addr = reg_val;
    reg_val = 0;
    reg_val |= (tap4&0xFF)<<DSP_COEFF1_TAP4_OFST;
    reg_addr = (volatile uint32_t* ) (ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_COEFF1_OFST);
    *reg_addr = reg_val;
}

void getCoeff0(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_COEFF0_OFST);
    *data = *reg_addr;
}

void getCoeff1(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_COEFF1_OFST);
    *data = *reg_addr;
}




void dspTest(uint8_t* dsp_arr ) {
    uint32_t data;
    uint8_t rdata;
    uint8_t index;
    uint8_t addr;

    gprTest(); //general purpose register test. simple wr and rd
    //ramReadSnk();
    ramWriteTestSrc();// fill up source ram with data and do a read back test 
    dumpRamSink(); // set up sink ram state machine in  dump state, expecting data from st IF
    readCTRLSink(&data);
    printf("After arming sink for ST transfer CTRL register is : 0x%08x\r\n", data );
    readDbgSink(&data);
    printf("After arming sink for ST transfer Debug register is : 0x%08x\r\n", data );
    dumpRamSource(); //start master axi st transfer.          //
    readDbgSource(&data);
    printf("After starting ST transfer : 0x%08x\r\n", data );
    ramReadSnk(); //check processed data;

    dspSetCoeff(28,63,95,119,127);
    
    getCoeff0(&data);
    printf("After setting Coeff0 : 0x%08x\r\n", data );

    getCoeff1(&data);
    printf("After setting Coeff1 : 0x%08x\r\n", data );

    dumpRamSink(); // set up sink ram state machine in  dump state, expecting data from st IF
    readCTRLSink(&data);
    printf("After arming sink for ST transfer CTRL register is : 0x%08x\r\n", data );
    readDbgSink(&data);
    printf("After arming sink for ST transfer Debug register is : 0x%08x\r\n", data );
    dumpRamSource(); //start master axi st transfer.          //
    readDbgSource(&data);
    printf("After starting ST transfer : 0x%08x\r\n", data );
    ramReadSnk();

    for(index = 0; index < 32; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        *dsp_arr++=rdata;
    }
    
    
}



