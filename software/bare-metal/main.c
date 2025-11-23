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


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define USER_IO_DIR     (0x01000000)
#define BIT_LED         (0x01000000)
#define BUTTON_MASK     (0x02000000)
#define ALT_CPU_WDTGPT_TMR0_BASE (0xFFD02000)
#define ALT_CPU_WDTGPT_TMR1_BASE (0xFFD03000)

//#define ALT_RSTMGR_PERMODRST_ADDR (0xFFD05014)         
#define ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK (1<<25)
#define ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK (1<<26)
#define ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK (1<<27)
//#define ALT_RSTMGR_ADDR (0xFFD05000)
#define ALT_RSTMGR_PERMODRST_OFST (0x14)
#define ALT_GPIO_BITMASK                0x1FFFFFFF

extern UART_INFO_t term0_info;
extern void dspTest(uint8_t* dsp_arr);


void mysleep(uint32_t cycles);
void dbgReg(uint32_t addr);
#ifdef ETH_TEST
static alt_eth_emac_instance_t emac1;
int eth_main(alt_eth_emac_instance_t* emac);
#endif

static uint8_t dsp_arr[32];

void clkMgrTest(void);
void rstMgrTest(void);


void ledTest(void);
void fpgaTest(void);
void fpgaCustomTest(void);

ALT_STATUS_CODE socfpga_bridge_io(void);
ALT_STATUS_CODE socfpga_int_start(void);
ALT_STATUS_CODE socfpga_watchdog_start(ALT_WDOG_TIMER_t tmr_id, ALT_WDOG_RESET_TYPE_t type,  uint32_t val);


int main(void) {
    //set a variable to hold status of each operation
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    //status = alt_clk_clkmgr_init(); 
    //clock manager has been initalized by uboot, leave it.

    //if (status != ALT_E_SUCCESS) {
    //    ALT_PRINTF("ERROR: CLOCK_INIT failed, %" PRIi32 ".\n", status);
    //} else {
    //    ALT_PRINTF("SUCCESS: CLOCK_INIT SUCCESSFUL, %" PRIi32 ".\n", status);
    //}

 
    //initialize uart by passing the term0_info address to init function       
    status = init_uart(&term0_info);
    
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: UART_INIT failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: UART_INIT SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    ALT_BRIDGE_t bridge = ALT_BRIDGE_LWH2F;
    status = alt_bridge_init(bridge, NULL, NULL);

    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: ALT_BRIDGE_INIT LWH2F failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_BRIDGE_INIT LWH2F SUCCESSFUL, %" PRIi32 ".\n", status);
    }

     bridge = ALT_BRIDGE_F2H;
    status = alt_bridge_init(bridge, NULL, NULL);

    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: ALT_BRIDGE_INIT F2H failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_BRIDGE_INIT F2H SUCCESSFUL, %" PRIi32 ".\n", status);
    }

     bridge = ALT_BRIDGE_H2F;
    status = alt_bridge_init(bridge, NULL, NULL);

    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: ALT_BRIDGE_INIT H2F failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_BRIDGE_INIT H2F SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    status = socfpga_bridge_io();




    //dbgReg(0xFFD08568);
	//mysleep(2000*1000);

    //enable watchdog to do a warm reset upon timeout
    ALT_WDOG_TIMER_t watchdog = ALT_WDOG0_INIT;

    ALT_WDOG_RESET_TYPE_t reset_mode = ALT_WDOG_WARM_RESET;

    uint32_t timer_val = 14;
    uint32_t curr_val;
    status = socfpga_watchdog_start(watchdog,reset_mode,timer_val);
    if (status != ALT_E_SUCCESS){
        ALT_PRINTF("ERROR: socfpga_watchdog_start failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: socfpga_watchdog_start success, %" PRIi32 ".\n", status);
    }
    curr_val = alt_wdog_counter_get_current(watchdog);
    ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);
    fpgaCustomTest();      
    clkMgrTest();
    //rstMgrTest();
    curr_val = alt_read_word(0xFFD05000);

    ALT_PRINTF("SUCCESS: RST STATUS REGISTER is %" PRIi32 ".\n", curr_val);
    //fpgaTest();

    //ledTest();
    
    //return 0;

    
    //ethernet module test, including init
#ifdef ETH_TEST
    eth_main(&emac1);
#endif

    if (status == ALT_E_SUCCESS)
    {
        status = socfpga_int_start();
    }

    if (status == ALT_E_SUCCESS) {
        ALT_PRINTF("SUCCESS: socfpga_int_start() finished, interrupt init done, %" PRIi32 ".\n", status);
    }
    
    //dbgReg(0xFFD04060);
    //dbgReg(0xFFD040A0);

    //dbgReg(0xFF703000);
    //dbgReg(0xFF703004);
    //dbgReg(0xFF703008);
    //dbgReg(0xFF703014);
    //dbgReg(0xFF703018);
    //dbgReg(0xFF70301C);
    //dbgReg(0xFF703028);
    //dbgReg(0xFF703058);

    //printf( "System manager emac group\n" );
    //dbgReg( 0xFFD08060);
    //dbgReg( 0xFFD08064);
    //printf( "Clock manager perkph group\n" );
    //for( k = 0; k< 13; k++) {
    //    dbgReg( 0xFFD04080 + 4*k);
    //}
    //printf( "Clock manager makn group\n" );
    //for( k = 0; k< 14; k++) {
    //    dbgReg( 0xFFD04040 + 4*k);
    //}
    //printf( "Clock manager module group\n" );
    //for( k = 0; k< 6; k++) {
    //    dbgReg( 0xFFD04000 + 4*k);
    //}
    //printf( "RST manager module group\n" );
    //for( k = 0; k< 8; k++) {
    //    if(k!=3) {
    //        dbgReg( 0xFFD05000 + 4*k);
    //    }
    //}
    //printf( "Pkn Mux group\n" );
    //for( k = 0; k< 20; k++) {
    //    dbgReg( 0xFFD08400 + 4*k);
    //}


    
	return( 0 );
}

void mysleep(uint32_t cycles) {
    while(cycles !=0) {
        cycles --;
    }
}


void dbgReg(uint32_t addr) {
    uint32_t val;
    val = alt_read_word(addr);
    printf("Addr @ 0x%08x value is 0x%08x.\r\n", (unsigned int)(addr), (unsigned int)(val));
}


void ledTest(void){
    
    //Uninit gpio module:
    alt_replbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK,
                      ALT_GPIO_BITMASK);
    
    dbgReg((uint32_t)ALT_RSTMGR_PERMODRST_ADDR);

    //Init gpio module: rst manager deassert resets
    
    alt_replbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK, 0);

    //dbgReg((uint32_t)ALT_RSTMGR_PERMODRST_ADDR);

    
    
    //uint32_t scan_input;
    int i;
    //dbgReg((uint32_t)ALT_GPIO1_SWPORTA_DDR_ADDR);
	alt_setbits_word( ALT_GPIO1_SWPORTA_DDR_ADDR , USER_IO_DIR );
    //dbgReg((uint32_t)ALT_GPIO1_SWPORTA_DDR_ADDR);

    printf("LED should blink 50 times!\r\n");
	for(i=0;i<50;i++)
	{
        ALT_PRINTF("LED blinked %d times!\r\n",i);
        //ALT_PRINTF("ERROR: alt_int_global_init() failed, %" PRIi32 ".\n", status);
        //dbgReg((uint32_t)ALT_GPIO1_SWPORTA_DDR_ADDR);
		alt_setbits_word( ALT_GPIO1_SWPORTA_DR_ADDR, BIT_LED );
		mysleep(5000*1000);
		alt_clrbits_word( ALT_GPIO1_SWPORTA_DR_ADDR, BIT_LED );
		mysleep(5000*1000);
	}

}



int eth_main(alt_eth_emac_instance_t* emac) {

	//uint32_t  gmac_version;
	uint32_t  stat;
    uint32_t  i,j;
    alt_eth_dma_desc_t *tx_desc;
    //uint32_t *p = (uint32_t *)emac;
    //uint32_t n = sizeof(*emac) / sizeof(uint32_t);

    uint32_t *tx_buf_p =  (uint32_t*) &emac->tx_buf[0];

    tx_desc = &emac->tx_desc_ring[emac->tx_current_desc_number];
    uint32_t* tx_desc_word = (uint32_t*) tx_desc;

    //dbgReg(0xFFD0408C);
    //alt_write_word(0xFFD0408C,0x1); //tentatively


    //pointer and a cast. treates this region of space as if it's the type
    //printf("DBG: emac pointer=%p\r\n",emac);
    //printf("DBG: emac pointer=%p\r\n",p);
    //printf("DBG: emac tx buf pointer=%p\r\n",tx_buf_p);
    //printf("DBG: emac struct size=%u\r\n",n);


    
    //gmac_version = alt_read_word (ALT_EMAC1_GMAC_VER_ADDR);
	//printf( "SUCCESS: gmac eth1 version is 0x%08x\r\n",(unsigned int)gmac_version );
    //gmac_version = alt_read_word (ALT_EMAC0_GMAC_VER_ADDR);
	//printf( "SUCCESS: gmac eth0 version is 0x%08x\r\n",(unsigned int)gmac_version );

    //stat =  alt_read_word (ALT_EMAC1_GMAC_SGMII_RGMII_SMII_CTL_STAT_ADDR);
    //printf( "SUCCESS: gmac eth1 link state is 0x%08x\r\n",(unsigned int)stat );

    //stat =  alt_read_word (ALT_EMAC0_GMAC_SGMII_RGMII_SMII_CTL_STAT_ADDR);
    //printf( "SUCCESS: gmac eth1 link state is 0x%08x\r\n",(unsigned int)stat );


    //stat =  alt_read_word (ALT_SYSMGR_EMAC_CTL_ADDR);
    //printf( "SUCCESS: system manager emac group's ctrl register is 0x %x\r\n",(unsigned int)stat );

    //stat =  alt_read_word (ALT_SYSMGR_EMAC_L3MST_ADDR);
    //printf( "SUCCESS: system manager emac group's L3MST register is 0x %x\r\n",(unsigned int)stat );


    uint8_t test_frame[64] = {
        //ASUSTekCOMPU_:
        0x24,0x4b,0xfe,0xe0,0xef,0x05,
        //Altera_:
         0x00,0x07,0xed,0x42,0x9a,0x48,
        // EtherType = 0x0800 (IPv4)
        0x08,0x00,
        // Payload: 46 bytes filler
        'T','e','s','t',' ','f','r','a','m','e',' ','f','r','o','m',' ',
        'D','E','-','N','a','n','o',' ','E','M','A','C','!',' ','1','2',
        '3','4','5','6','7','8','9','0','!','!','!','!','!','!','!','!'
    };

    emac->instance = 1;
    //alt_eth_emac_hps_init(emac->instance);
    //alt_eth_emac_dma_init(emac->instance);

    alt_eth_dma_mac_config(emac);
    stat =  alt_read_word (ALT_EMAC1_GMAC_SGMII_RGMII_SMII_CTL_STAT_ADDR);
    printf( "SUCCESS: gmac eth1 link state after config is 0x%08x\r\n",(unsigned int)stat );
    //send packet
    printf( "Hufei: get ready to send packet\r\n" );
    for( i=0;i<1;i++) {
        //mysleep(5000*1000);
        //for (j = 0; j < 32; j++) {
        //    printf("tx_buf content: DBG[%d]: 0x%08x\r\n", j, tx_buf_p[j]);
        //}
        //
        //for (i = 0; i < 4; i++) {
        //    printf("tx_desc content: DBG[%d]: 0x%08x\r\n", i, tx_desc_word[i]);
        //}

        mysleep(1000*1000);
        alt_eth_send_packet(test_frame, 64, 1, 1, emac);
        mysleep(5000*1000);


        for (j = 0; j < 4; j++) { //should be 32
            printf("tx_buf content: DBG[%d]: 0x%08x\r\n", j, tx_buf_p[j]);
        }
        //mysleep(50000*1000);
        //for (j = 0; j < n; j++) {
        //    printf("all content: DBG[%d]: 0x%08x\r\n", j, p[j]);
        //}

        for (i = 0; i < 4; i++) {
            printf("tx_desc content: DBG[%d]: 0x%08x\r\n", i, tx_desc_word[i]);
        }


    
    }
    printf( "Hufei: packet sent, check on wireshark\r\n" );

	return( 0 );
}

ALT_STATUS_CODE socfpga_watchdog_start(ALT_WDOG_TIMER_t tmr_id, ALT_WDOG_RESET_TYPE_t type, uint32_t val) {
    
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    //uint32_t curr_val;

    status = alt_wdog_init();

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG init failed, %" PRIi32 ".\n", status);
    } else {
      //  ALT_PRINTF("SUCCESS: WATCHDOG init SUCCESSFUL, %" PRIi32 ".\n", status);
    }

   // curr_val = alt_wdog_counter_get_current(tmr_id);
    //ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);


    status = alt_wdog_response_mode_set(tmr_id,type);

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG mode set failed, %" PRIi32 ".\n", status);
    } else {
     //   ALT_PRINTF("SUCCESS: WATCHDOG mode set SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    status = alt_wdog_counter_set(tmr_id, val);
    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG timeout set failed, %" PRIi32 ".\n", status);
    } else {
     //   ALT_PRINTF("SUCCESS: WATCHDOG timeout set SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    status = alt_wdog_start(tmr_id);
    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG timeout start failed, %" PRIi32 ".\n", status);
    } else {
     //   ALT_PRINTF("SUCCESS: WATCHDOG timeout start SUCCESSFUL, %" PRIi32 ".\n", status);
    }

  //  curr_val = alt_wdog_counter_get_current(tmr_id);
 //   ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);

 //   curr_val=alt_read_word(0xFFD02000);

    //ALT_PRINTF("SUCCESS: WATCHDOG control reg value is , %" PRIi32 ".\n", curr_val);




    return status;

}


void fpgaTest(void){
	int loop_count;
	int led_direction;
	int led_mask;
    //int dip_sw_bits;

    void* h2p_lw_led_addr;

	h2p_lw_led_addr=(uint32_t* )( ALT_LWFPGASLVS_OFST + LED_PIO_BASE  );
    //dip_sw_addr=( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE  );
	//ALT_PRINTF("SUCCESS: LED register address is , %" PRIi32 ".\n", h2p_lw_led_addr);
    printf("LED reg address is : 0x%08x\r\n", h2p_lw_led_addr );

	// toggle the LEDs a bit

	loop_count = 0;
	led_mask = 0x01;
	led_direction = 0; // 0: left to right direction
    //dip_sw_bits = 0;
	while( loop_count < 2 ) {
		printf( "Hufei:You'd be seeing shifting LEDs \n" );
		// control led
		*(uint32_t *)h2p_lw_led_addr = ~led_mask; 
		printf( "Hufei:register stuck test \n" );
		// wait 100ms
		mysleep( 1000*1000 );
		
		// update led mask
		if (led_direction == 0){
			led_mask <<= 1;
			if (led_mask == (0x01 << (LED_PIO_DATA_WIDTH-1)))
				 led_direction = 1;
		}else{
			led_mask >>= 1;
			if (led_mask == 0x01){ 
				led_direction = 0;
				loop_count++;
			}
		}
		
	} // while
    
}

void clkMgrTest(void){

    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    alt_freq_t freq;

    ALT_CLK_t clk = ALT_CLK_EMAC1;
    clk = ALT_CLK_MAIN_PLL;
    ALT_CLK_SAFE_DOMAIN_t clk_domain;
    clk_domain = ALT_CLK_DOMAIN_NORMAL;

    if(alt_clk_is_in_safe_mode(clk_domain)) {
        ALT_PRINTF("Clock manager is in safe mode, PLL settings are not effective\r\n");
    } else{
        ALT_PRINTF("Clock manager is not in safe mode, PLL settings are effective\r\n");
    }

    status=alt_clk_pll_is_bypassed(clk);
    if(status == ALT_E_TRUE) {
        ALT_PRINTF("Main PLL bypassed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("Main PLL not bypassed, %" PRIi32 ".\n", status);

    }

    clk = ALT_CLK_PERIPHERAL_PLL;
    status=alt_clk_pll_is_bypassed(clk);
    if(status == ALT_E_TRUE) {
        ALT_PRINTF("Periph PLL bypassed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("Periph PLL not bypassed, %" PRIi32 ".\n", status);

    }

    clk = ALT_CLK_SDRAM_PLL;
    status=alt_clk_pll_is_bypassed(clk);
    if(status == ALT_E_TRUE) {
        ALT_PRINTF("SDRAM PLL bypassed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SDRAM PLL not bypassed, %" PRIi32 ".\n", status);

    }

    clk = ALT_CLK_EMAC1;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: EMAC1 frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("EMAC1 clk not enabled, enable now %" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("EMAC1 clk is enabled, %" PRIi32 ".\n", freq);
    }
    clk = ALT_CLK_EMAC0;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: EMAC0 frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("EMAC0 clk not enabled, enable now %" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("EMAC0 clk is enabled, %" PRIi32 ".\n", freq);
    }




    clk = ALT_CLK_L4_MP;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: L4_MP frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("L4_MP clk not enabled, enable now%" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("L4_MP clk is enabled, %" PRIi32 ".\n", freq);
    }

    clk = ALT_CLK_L4_SP;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: L4_SP frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("L4_SP clk not enabled, enable now%" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("L4_SP clk is enabled, %" PRIi32 ".\n", freq);
    }



    clk = ALT_CLK_SDMMC;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_CLK_SDMMC frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("ALT_CLK_SDMMC clk not enabled, enable now%" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("ALT_CLK_SDMMC clk is enabled, %" PRIi32 ".\n", freq);
    }

    clk = ALT_CLK_H2F_USER2;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_CLK_H2F_USER2 frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("ALT_CLK_H2F_USER2 clk not enabled, enable now%" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("ALT_CLK_H2F_USER2 clk is enabled, %" PRIi32 ".\n", freq);
    }

    clk = ALT_CLK_H2F_USER1;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_CLK_H2F_USER1 frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("ALT_CLK_H2F_USER1 clk not enabled, enable now%" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("ALT_CLK_H2F_USER1 clk is enabled, %" PRIi32 ".\n", freq);
    }

    clk = ALT_CLK_H2F_USER0;
    status=alt_clk_freq_get(clk,&freq);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get frequency, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: ALT_CLK_H2F_USER0 frequency is, %" PRIi32 ".\n", freq);
    }

    status = alt_clk_is_enabled(clk);
    if (status != ALT_E_TRUE) {
        ALT_PRINTF("ALT_CLK_H2F_USER0 clk not enabled, enable now%" PRIi32 ".\n", status);
        alt_clk_clock_enable(clk);
    } else {
        ALT_PRINTF("ALT_CLK_H2F_USER0 clk is enabled, %" PRIi32 ".\n", freq);
    }
    uint32_t mult, div, subdiv, temp;
    clk = ALT_CLK_PERIPHERAL_PLL;

    status = alt_clk_pll_vco_cfg_get(clk,&mult,&div);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get M and N, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: M and N  is respectively, %" PRIi32 ", %" PRIi32 ".\n", mult,div);
    }
    
    clk = ALT_CLK_EMAC1;
    status = alt_clk_divider_get(clk,&subdiv);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get divider value, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: divider for the clock is, %" PRIi32 ".\n", subdiv);
    }
    temp = 8;
    status = alt_clk_divider_set(clk,subdiv);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't set divider value, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: divider for the clock is set, %" PRIi32 ".\n", temp);
    }

    mysleep( 1000*1000 );

    status = alt_clk_divider_get(clk,&subdiv);
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: can't get divider value, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: divider for the clock is, %" PRIi32 ".\n", subdiv);
    }


    return;
    
}


void rstMgrTest(void){

    int k;
    printf( "Pkn Mux group\n" );
    for( k = 0; k< 9; k++) {
        if(k!=3) {
            dbgReg( 0xFFD05000 + 4*k);
        }
    }


    //THis triggers cold reset
    //alt_write_word(0xFFD05004, 0x1);

    //THis triggers warm reset
    alt_write_word(0xFFD05004, 0x2);



}

ALT_STATUS_CODE socfpga_bridge_io(void)
{
    const uint32_t ALT_LWFPGA_BASE         = 0xFF200000;
    const uint32_t ALT_LWFPGA_SYSID_OFFSET = 0x00010000;
   // const uint32_t ALT_LWFPGA_LED_OFFSET   = 0x00010040;

    /* Attempt to read the system ID peripheral */
    uint32_t sysid = alt_read_word(ALT_LWFPGA_BASE + ALT_LWFPGA_SYSID_OFFSET);

    ALT_PRINTF("INFO: LWFPGA Slave => System ID Peripheral value = 0x%" PRIx32 ".\n", sysid);
    return ALT_E_SUCCESS;

}


void fpgaCustomTest(void){
    uint8_t i;
    dspTest(dsp_arr);
    for(i=0;i<32;i++){
        printf("Dram value check: 0x%02x\r\n", dsp_arr[i] );
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

