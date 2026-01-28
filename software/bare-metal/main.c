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
#include "alt_printf.h"
#include "alt_p2uart.h"
#include "alt_watchdog.h"
#include "alt_bridge_manager.h"
#include "hps_0.h"
#include "fpga_dsp.h"
#include "alt_cache.h"


//#define FPGA_TEST
//#define ETH_TEST
#define DMA_TEST
//#define PERIPH_TEST
#define FRAM_BUF_SIZE 384
extern UART_INFO_t term0_info;
extern void dspTest(uint8_t* dsp_arr);
extern void sinTest(uint8_t* dsp_arr);
extern void ethSinTest(uint8_t* eth_src, uint8_t* dsp_arr);

extern void ethernet_raw_frame_gen(uint32_t len, uint8_t* dst_addr_arr, uint8_t* arr);
extern void dump_frame_buf(alt_eth_emac_instance_t * emac );
extern uint8_t MAC_ADDR[6]; 
extern uint8_t rx_frame_buffer[ETH_BUFFER_SIZE];
extern uint8_t tx_frame_buffer[ETH_BUFFER_SIZE];

extern void mysleep(uint32_t cycles);
extern void dbgReg(uint32_t addr);
extern void swap_addr(uint8_t* arr, size_t len);
extern void adxl345(void);

#ifdef ETH_TEST
static alt_eth_emac_instance_t emac1  __attribute__((section(".bss.oc_ram"), aligned(32))); //create emac buffer in FPGA OCRAM
//static alt_eth_emac_instance_t emac1;                                                                                            //
int eth_main(alt_eth_emac_instance_t* emac);
#endif
#ifdef DMA_TEST
extern int dma_main(void);
#endif

static uint8_t dsp_arr[4096];

extern void clkMgrTest(void);
void rstMgrTest(void);

void ledTest(void);
extern void fpgaTest(void);
void fpgaCustomTest(uint8_t* eth_src);

extern ALT_STATUS_CODE bridgeTest(void);
ALT_STATUS_CODE socfpga_int_start(void);
ALT_STATUS_CODE socfpga_watchdog_start(ALT_WDOG_TIMER_t tmr_id, ALT_WDOG_RESET_TYPE_t type,  uint32_t val);
ALT_STATUS_CODE watchDogInit(void);


int main(void) {
    //set a variable to hold status of each operation
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    //initialize uart by passing the term0_info address to init function       
    status = init_uart(&term0_info);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: UART_INIT failed, %" PRIi32 ".\n", status);
    }
    status = alt_cache_system_enable();
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: CACHE_ENABLE failed, %" PRIi32 ".\n", status);
    }     
    status = socfpga_int_start();
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: Interrupt init failed, %" PRIi32 ".\n", status);
    } 
    //enable watchdog to do a warm reset upon timeout
    watchDogInit();
    
    //clkMgrTest();
    //fpgaCustomTest();      
    //rstMgrTest();
    //fpgaTest();
#ifdef PERIPH_TEST
    //ledTest();
    adxl345();
#endif
     
    //ethernet module test, including init
#ifdef ETH_TEST
    eth_main(&emac1);
#endif
    
#ifdef DMA_TEST
    dma_main();
#endif

#ifdef FPGA_TEST   
    axiTest();
#endif
    
    while(1);

    
	return( 0 );
}





int eth_main(alt_eth_emac_instance_t* emac) {

	uint32_t  stat,addr;
    uint32_t  dma_status;
    //uint8_t   test_frame[FRAM_BUF_SIZE]; // a temp array on stack, 384 bytes.
    //ethernet_raw_frame_gen( sizeof(test_frame),MAC_ADDR,test_frame); //generate a raw frame that has incrementing payload value.
    uint32_t rcv_len = 0; //a temp var to hold received frame's length.
    emac->instance = 1;
    alt_eth_emac_dma_init(emac); //emac and phy initialization
    uint32_t last_rxints = emac->rxints; //a temp var to hold previous interrupt numbers that is stored in the struct.
    //uint32_t last_txints = emac->txints;


    while(1) { //poll for the moment until watchdog triggers
        if(last_rxints != emac->rxints) { //callback function increments rxints when rx interrupt triggers
            last_rxints++;
            dma_status = alt_read_word(0xFF703014);
            alt_eth_get_packet(rx_frame_buffer,&rcv_len,emac); //rx_frame_buffer is a location in global region
            //ethCtlLed(rx_frame_buffer); //small function that uses a packet's info to toggle LED
            swap_addr(rx_frame_buffer, sizeof(rx_frame_buffer)); //swaps the dst and src address of the received packet.
            for(int i =0;i!=14;i++) { //tx_frame_buffer is a location in global region
                tx_frame_buffer[i] = rx_frame_buffer[i]; //pops up tx_frame_buffer's raw frame header, src+dst+type = 14byte
            }

            printf("Call back reading dma int status is 0x%08x, received packet len is 0x%08x \n", dma_status, rcv_len );
            ethSinLoop(rx_frame_buffer,tx_frame_buffer+14,rcv_len); //
            for(int i = 14;i!=rcv_len;i++) { //print out the content of rx_frame_buffer for plotting in plotting software
                printf("(%d,%d)",i, (int8_t)rx_frame_buffer[i] ); //
                if(i%10 ==9) {
                    printf("\r\n" );
                } else{
                    printf(",");
                }
            }
            printf("\r\n" );
            for(int i = 14;i!=rcv_len;i++) { //print out the content of tx_frame_buffer for plotting in plotting software
                printf("(%d,%d)",i, (int8_t)tx_frame_buffer[i] ); //
                if(i%10 ==9) {
                    printf("\r\n" );
                } else{
                    printf(",");
                }
            }
            scatter_frame(tx_frame_buffer,rcv_len,emac);
            addr = 0xFF702180; 
            stat = alt_read_word( addr);
            printf("DBG: Number of good frames received is                                0x%08x,0d%08d !\n",addr,stat);

            addr = 0xFF702184; 
            stat = alt_read_word( addr);
            printf("DBG: Number of bytes received is                0x%08x,0d%08d !\n",addr,stat);
            
            addr = 0xFF702188; 
            stat = alt_read_word( addr);
            printf("DBG: Number of bytes in good frames received is 0x%08x,0d%08d !\n",addr,stat);

            addr = 0xFF703014; 
            stat = alt_read_word( addr); 
            printf("DBG: DMA interrupt status is                    0x%08x,0x%08x !\n",addr,stat);
        }
        //dump_frame_buf(emac);

    }

    return 0;

}

ALT_STATUS_CODE socfpga_watchdog_start(ALT_WDOG_TIMER_t tmr_id, ALT_WDOG_RESET_TYPE_t type, uint32_t val) {
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    status = alt_wdog_init();
    if (status != ALT_E_SUCCESS){
        ALT_PRINTF("ERROR: WATCHDOG init failed, %" PRIi32 ".\n", status);
    } 
    status = alt_wdog_response_mode_set(tmr_id,type);
    if (status != ALT_E_SUCCESS){
        ALT_PRINTF("ERROR: WATCHDOG mode set failed, %" PRIi32 ".\n", status);
    }
    status = alt_wdog_counter_set(tmr_id, val);
    if (status != ALT_E_SUCCESS){
        ALT_PRINTF("ERROR: WATCHDOG timeout set failed, %" PRIi32 ".\n", status);
    }
    status = alt_wdog_start(tmr_id);
    if (status != ALT_E_SUCCESS){
        ALT_PRINTF("ERROR: WATCHDOG timeout start failed, %" PRIi32 ".\n", status);
    }
    return status;
}


void fpgaCustomTest(uint8_t* eth_src){
   // dspTest(dsp_arr);
    ethSinTest(eth_src,dsp_arr);
   //
    uint16_t i;
    for(i=0;i<4096;i++){
      //  printf("Dram value check: 0x%02x\r\n", dsp_arr[i] );
        printf("(%d,%d)",i, (int8_t)dsp_arr[i] ); //
        if(i%10 ==9) {
            printf("\r\n" );
        } else{
            printf(",");
        }
    }
    printf("TEST DONE\r\n");
    mysleep( 80000*1000 );
}


ALT_STATUS_CODE socfpga_int_start(void)
{
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    /*
    // Initialize the global and CPU interrupt items
    */

    if (status == ALT_E_SUCCESS)
    {
        status = alt_int_global_init();
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("ERROR: alt_int_global_init() failed, %" PRIi32 ".\n", status);
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_int_cpu_init();
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("ERROR: alt_int_cpu_init() failed, %" PRIi32 ".\n", status);
        }
    }

    /*
    // Enable the CPU and global interrupt
    */

    if (status == ALT_E_SUCCESS)
    {
        status = alt_int_cpu_enable();
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("ERROR: alt_int_cpu_enable() failed, %" PRIi32 ".\n", status);
        }
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_int_global_enable();
        if (status != ALT_E_SUCCESS)
        {
            ALT_PRINTF("ERROR: alt_int_global_enable() failed, %" PRIi32 ".\n", status);
        }
    }

    return status;
}




ALT_STATUS_CODE watchDogInit(void){
    ALT_STATUS_CODE status = ALT_E_SUCCESS;    
    ALT_WDOG_TIMER_t watchdog = ALT_WDOG0_INIT;
    ALT_WDOG_RESET_TYPE_t reset_mode = ALT_WDOG_WARM_RESET;
    uint32_t timer_val = 14;
    uint32_t curr_val;
    status = socfpga_watchdog_start(watchdog,reset_mode,timer_val);
    if (status != ALT_E_SUCCESS){
        ALT_PRINTF("ERROR: socfpga_watchdog_start failed, %" PRIi32 ".\n", status);
    }
    curr_val = alt_wdog_counter_get_current(watchdog);
    ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);
    return status;
}


