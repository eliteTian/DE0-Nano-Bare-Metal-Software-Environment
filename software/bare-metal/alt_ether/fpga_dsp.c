#include "fpga_dsp.h"

static uint8_t pseudoRand(uint8_t seed){

    uint8_t lfsr = seed ? seed : 1; // avoid zero lockup
    uint8_t bit;

    // Perform 8 steps to scramble thoroughly
    for(int i=0;i<8;i++) {
        bit = ((lfsr >> 7) ^ (lfsr >> 5) ^ (lfsr >> 4) ^ (lfsr >> 3)) & 1;
        lfsr = (lfsr << 1) | bit;
    }
    return lfsr;
}

static const int8_t sine500K[100] = {
     0,  4,  8, 12, 16, 20, 24, 28, 31, 35,
    38, 42, 45, 48, 51, 54, 56, 58, 60, 62,
    62, 63, 63, 63, 62, 62, 60, 58, 56, 54,
    51, 48, 45, 42, 38, 35, 31, 28, 24, 20,
    16, 12,  8,  4,  0, -4, -8, -12, -16, -20,
   -24, -28, -31, -35, -38, -42, -45, -48, -51, -54,
   -56, -58, -60, -62, -62, -63, -63, -63, -62, -62,
   -60, -58, -56, -54, -51, -48, -45, -42, -38, -35,
   -31, -28, -24, -20, -16, -12, -8,  -4,  0,  4,
     8, 12, 16, 20, 24, 28, 31, 35, 38, 42
};

static const int8_t sine10M[5] = { 0, 20, 12, -12, -20 };

static const uint8_t unsigned_mixed[100] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99
};

//(0,0),(1,24),(2,20),(3,0),(4,-4),(5,0),(6,24),(7,8),(8,43),(9,15),
//(10,38),(11,62),(12,57),(13,36),(14,51),(15,74),(16,56),(17,46),(18,60),(19,82),
//(20,62),(21,83),(22,75),(23,51),(24,62),(25,82),(26,72),(27,46),(28,56),(29,34),
//(30,51),(31,68),(32,57),(33,30),(34,38),(35,55),(36,31),(37,16),(38,44),(39,0),
//(40,16),(41,32),(42,20),(43,24),(44,0),(45,16),(46,4),(47,-12),(48,-4),(49,0),
//(50,-24),(51,-8),(52,-19),(53,-23),(54,-18),(55,-22),(56,-32),(57,-8),(58,-39),(59,-14),
//(60,-56),(61,-38),(62,-48),(63,-75),(64,-62),(65,-63),(66,-63),(67,-75),(68,-50),(69,-62),
//(70,-60),(71,-38),(72,-44),(73,-24),(74,-51),(75,-36),(76,-57),(77,-54),(78,-38),(79,-35),
//(80,-31),(81,-8),(82,-12),(83,-8),(84,-20),(85,-8),(86,4),(87,8),(88,31),(89,55),
//(90,38),(91,32),(92,28),(93,8),(94,44),(95,24),(96,31),(97,23),(98,18),(99,22)

static const int8_t mixed[100] = {
     0, 24, 20, 0, -4, 0, 24, 8, 43, 15,
    38, 62, 57, 36, 51, 74, 56, 46, 60, 82,
    62, 83, 75, 51, 62, 82, 72, 46, 56, 34,
    51, 68, 57, 30, 38, 55, 31, 16, 44, 0,
    16, 32, 20, 24, 0, 16, 4, -12, -4, 0,
   -24, -8, -19, -23, -18, -22, -32, -8, -39, -14,
   -56, -38, -48, -75, -62, -63, -63, -75, -50, -62,
   -60, -38, -44, -24, -51, -36, -57, -54, -38, -35,
   -31, -8, -12, -8, -20, -8, 4, 8, 31, 55,
    38, 32, 28, 8, 44, 24, 31, 23, 18, 22
};


static int8_t mixSig(uint16_t index){
    uint16_t mod10m = 0;
    uint16_t mod500k = 0;
    int8_t val = 0;
    
    mod10m = index % 5;
    mod500k = index % 100;
    val = sine500K[mod500k] + sine10M[mod10m];
    return val;

}

//static uint8_t mixSig(uint16_t index){
//    return mixed[index%100];
//}




void writeRamSource(uint16_t addr, uint8_t data){ //bug found here. addr was uint8_t
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= ((uint32_t)(addr & 0xFFF))<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= ((uint32_t)data)<<SRC_CTRL_CMD_DATA_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;

    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
    //printf("Command register into FPGA is: 0x%08x\r\n", reg_val );

}

void readRamSource(uint16_t addr, uint8_t* rdata){//bug found here.
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= ((uint32_t)(addr & 0xFFF))<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= 0x0<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;
    uint32_t temp, pend;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
    //printf("Command register into FPGA is: 0x%08x\r\n", reg_val );

    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SOURCE_0_BASE+SRC_STATUS_OFST);
    do{
        temp = *reg_addr;
        pend = temp & SRC_STATUS_PEND;
    } while(pend);
    
    *rdata = (temp & SRC_STATUS_RDATA) >> SRC_STATUS_RDATA_OFST ;
    //printf("Command register read data is: 0x%02x\r\n", *rdata );

}


void writeRamSink(uint16_t addr, uint8_t data){
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= ((uint32_t)(addr & 0xFFF))<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= ((uint32_t)data)<<SRC_CTRL_CMD_DATA_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;

    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val;
}

void readRamSink(uint16_t addr, uint8_t* rdata){
    uint32_t reg_val = 0;
    volatile uint32_t* reg_addr;
    reg_val |= ((uint32_t)(addr & 0xFFF))<< SRC_CTRL_CMD_ADDR_OFST;
    reg_val |= 0x0<<SRC_CTRL_CMD_TYPE_OFST;
    reg_val |= 0x1<<SRC_CTRL_CMD_VALID_OFST;
    uint32_t temp, pend;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_CTRL_OFST);
    *reg_addr = reg_val; //write rd command to cmd register

    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_STATUS_OFST);
    do{
        temp = *reg_addr;
        pend = temp & SRC_STATUS_PEND;
    } while(pend);
    
    *rdata = (temp & SRC_STATUS_RDATA) >> SRC_STATUS_RDATA_OFST ;
}

void waitStatusComplete(void) {
    uint32_t temp, pend;
    volatile uint32_t* reg_addr;
    reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+FPGA_DATA_SINK_0_BASE+SRC_STATUS_OFST);
    do{
        temp = *reg_addr;
        pend = temp & SRC_STATUS_PEND;
    } while(pend); // can add timeout here.

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
    uint16_t  index;
    uint8_t   rdata = 0;
    uint8_t   wdata;
    uint16_t  addr;

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        wdata = pseudoRand((uint8_t)index);
        writeRamSource(addr, wdata);
    }
    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        readRamSource(addr,&rdata);
        if(rdata!=pseudoRand((uint8_t)index)) {
            printf("Source RAM data mismatch: 0x%02x\r\n", rdata );
        }
    }

}

void ramWriteTestSnk(void) {
    uint16_t index;
    uint8_t rdata = 0;
    uint8_t wdata;
    uint16_t addr;

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        wdata = pseudoRand((uint8_t)index);
        writeRamSink(addr, wdata);
    }
    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        if(rdata!=pseudoRand((uint8_t)index)) {
            //printf("Sink ram before dump writing : 0x%02x\r\n", rdata );
            printf("Sink RAM data mismatch: 0x%02x\r\n", rdata );        
        }
    }

}


void ramReadSnk(void) {
    uint16_t index;
    uint8_t rdata = 0;
    uint16_t addr;

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        printf("Sink RAM data read is : 0x%02x\r\n", rdata );     
    }

}

void ramReadSrc(void) {
    uint16_t index;
    uint8_t rdata = 0;
    uint16_t addr;

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        readRamSource(addr,&rdata);
        printf("Source RAM data read is : 0x%02x\r\n", rdata );     
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


void ramWriteSinWav(){
    uint16_t  index;
    uint8_t   rdata = 0;
    int8_t    signed_wdata;
    uint8_t   raw_wdata;
    uint16_t  addr;

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        signed_wdata = mixSig(index);
        raw_wdata = (uint8_t)signed_wdata;
        //printf("Source RAM data written at index %d: is %d  raw is 0x%02x \r\n", index, signed_wdata, raw_wdata );

        writeRamSource(addr, raw_wdata);
    }
    for(index = 0; index < RAM_SIZE; index++) {
        addr = index;
        readRamSource(addr,&rdata);
        signed_wdata = mixSig(index);
        raw_wdata = (uint8_t)signed_wdata;
        if(rdata!=raw_wdata) {
            printf("Source RAM data mismatch: at index %d: %d vs %d \r\n", index, (int8_t)rdata, (int8_t)raw_wdata );
        }
    }

}


void ramWriteSinUnsinged(void) {
    uint16_t index;
    uint8_t rdata = 0;
    uint8_t wdata;
    uint16_t addr;
    uint8_t cdata;

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        wdata = mixSig(index);
        writeRamSource(addr, wdata);
        
    }
    for(index = 0; index < RAM_SIZE; index++) {
        addr = index;
        cdata = mixSig(index);
        readRamSource(addr,&rdata);
        if(rdata!= cdata) {
            printf("Source RAM data mismatch: 0x%02x vs 0x%02x\r\n", rdata, cdata );        
        }
    }

}

void dspGainSet(uint8_t gain) {
    if(gain >3) {
        printf("Wrong input value, only 4 choices: 0,1,2,3\r\n" );
        return;
    } 

    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_CTRL_OFST);
    volatile uint32_t val;
    val = (((uint32_t)gain & DSP_CTRL_GAIN)<<DSP_CTRL_GAIN_OFST);
    *reg_addr = val;
}

void dspGainGet(uint32_t* data){
    volatile uint32_t* reg_addr = (volatile uint32_t* ) ( ALT_LWFPGASLVS_OFST+DSP_APB_0_BASE+DSP_CTRL_OFST);
    *data = *reg_addr;
}





void sinTest(uint8_t* dsp_arr) {
    uint32_t data;
    uint8_t rdata;
    uint16_t index;
    uint16_t addr;

    for (int i=0; i<100; i++)
        printf("%d ", mixed[i]);
    printf("\n");

    //ramWriteTestSrc();
    //ramWriteSinUnsinged();  //fill up source ram with unsigned data  
    ramWriteSinWav();// fill up source ram with data and do a read back test  
    dspGainSet(3);
    dspSetCoeff(28,63,95,119,127);
    dspGainGet(&data);
    printf("After setting gain :   0x%08x\r\n", data );

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
    waitStatusComplete();
    readDbgSink(&data);
    printf("After starting ST transfer : 0x%08x\r\n", data );
    
    //ramReadSnk();

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        *dsp_arr++=rdata;
    }
  
}



void dspTest(uint8_t* dsp_arr ) {
    uint32_t data;
    uint8_t rdata;
    uint16_t index;
    uint16_t addr;
    

    gprTest(); //general purpose register test. simple wr and rd
    //ramReadSnk();
    ramWriteTestSrc();// fill up source ram with data and do a read back test 
    dumpRamSink(); // set up sink ram state machine in  dump state, expecting data from st IF
    readCTRLSink(&data);
    printf("After arming sink for ST transfer CTRL register is : 0x%08x\r\n", data );
    readDbgSink(&data);
    printf("After arming sink for ST transfer Debug register is : 0x%08x\r\n", data );
    dumpRamSource(); //start master axi st transfer.          //
    readDbgSink(&data);
    printf("After starting ST transfer : 0x%08x\r\n", data );
    //ramReadSnk(); //check processed data;
    dspGainSet(2);
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
    waitStatusComplete();
    readDbgSink(&data);
    printf("After starting ST transfer : 0x%08x\r\n", data );
    
    //ramReadSnk();

    for(index = 0; index < RAM_SIZE; index++) {
        addr = index; 
        readRamSink(addr,&rdata);
        *dsp_arr++=rdata;
    }
    
    
}



