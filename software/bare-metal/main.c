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

void mysleep(uint32_t cycles);
void dbgReg(uint32_t addr);
int eth_main(alt_eth_emac_instance_t* emac);
extern UART_INFO_t term0_info;
ALT_STATUS_CODE socfpga_watchdog_start(ALT_WDOG_TIMER_t tmr_id, ALT_WDOG_RESET_TYPE_t type,  uint32_t val);



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

static alt_eth_emac_instance_t emac1;


int main(void) {
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

   // alt_eth_emac_instance_t emac1;

    status = init_uart(&term0_info);

    
    if (status != ALT_E_SUCCESS) {
        ALT_PRINTF("ERROR: UART_INIT failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: UART_INIT SUCCESSFUL, %" PRIi32 ".\n", status);
    }

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

    dbgReg((uint32_t)ALT_RSTMGR_PERMODRST_ADDR);

    
    
    //uint32_t scan_input;
    int i;
    dbgReg((uint32_t)ALT_GPIO1_SWPORTA_DDR_ADDR);
	alt_setbits_word( ALT_GPIO1_SWPORTA_DDR_ADDR , USER_IO_DIR );
    dbgReg((uint32_t)ALT_GPIO1_SWPORTA_DDR_ADDR);

    printf("LED should blink 5 times!\r\n");
	for(i=0;i<1;i++)
	{
        ALT_PRINTF("LED blinked %d times!\r\n",i);
        //ALT_PRINTF("ERROR: alt_int_global_init() failed, %" PRIi32 ".\n", status);
        dbgReg((uint32_t)ALT_GPIO1_SWPORTA_DDR_ADDR);
		alt_setbits_word( ALT_GPIO1_SWPORTA_DR_ADDR, BIT_LED );
		mysleep(5000*1000);
		alt_clrbits_word( ALT_GPIO1_SWPORTA_DR_ADDR, BIT_LED );
		mysleep(5000*1000);
	}

    curr_val = alt_wdog_counter_get_current(watchdog);
    ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);


    //printf("Test the button!\r\n");
    eth_main(&emac1);
    //while(1) {
	//	mysleep(5000*1000);
    //    curr_val = alt_wdog_counter_get_current(watchdog);
    //    ALT_PRINTF("SUCCESS: WATCHDOG current counter value after eth is , %" PRIi32 ".\n", curr_val);
    //}



    //
	//while(1){
    //    //mysleep(5000*1000);
	//	scan_input = alt_read_word( ALT_GPIO1_EXT_PORTA_ADDR );	
    //    dbgReg((uint32_t)ALT_GPIO1_EXT_PORTA_ADDR);
	//	if((~scan_input)&BUTTON_MASK)
	//		alt_setbits_word( ALT_GPIO1_SWPORTA_DR_ADDR , BIT_LED );
	//	else    
    //        alt_clrbits_word( ALT_GPIO1_SWPORTA_DR_ADDR , BIT_LED );
	//}	

    if (status == ALT_E_SUCCESS)
    {
        status = socfpga_int_start();
    }

    if (status == ALT_E_SUCCESS) {
        ALT_PRINTF("SUCCESS: socfpga_int_start() finished, interrupt init done, %" PRIi32 ".\n", status);
    }
    


    
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




int eth_main(alt_eth_emac_instance_t* emac) {

	uint32_t  gmac_version;
	uint32_t  stat;


    //pointer and a cast. treates this region of space as if it's the type


    
    gmac_version = alt_read_word (ALT_EMAC1_GMAC_VER_ADDR);
    
	printf( "SUCCESS: gmac eth1 version is %d\r\n",(unsigned int)gmac_version );
    gmac_version = alt_read_word (ALT_EMAC0_GMAC_VER_ADDR);
	printf( "SUCCESS: gmac eth0 version is %d\r\n",(unsigned int)gmac_version );

    stat =  alt_read_word (ALT_EMAC1_GMAC_SGMII_RGMII_SMII_CTL_STAT_ADDR);
    printf( "SUCCESS: gmac eth1 link state is 0x %x\r\n",(unsigned int)stat );

    stat =  alt_read_word (ALT_SYSMGR_EMAC_CTL_ADDR);
    printf( "SUCCESS: system manager emac group's ctrl register is 0x %x\r\n",(unsigned int)stat );

    stat =  alt_read_word (ALT_SYSMGR_EMAC_L3MST_ADDR);
    printf( "SUCCESS: system manager emac group's L3MST register is 0x %x\r\n",(unsigned int)stat );


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
    alt_eth_dma_mac_config(emac);
    
    //send packet
    printf( "Hufei: get ready to send packet\r\n" );
    for(int i=0;i<20;i++) {
        mysleep(5000*1000);
        alt_eth_send_packet(test_frame, 64, 1, 1, emac);
    }
    printf( "Hufei: packet sent, check on wireshark\r\n" );

    //uint32_t *p = (uint32_t *)&emac1;
    //uint32_t n = sizeof(&emac1) / sizeof(uint32_t);

	return( 0 );
}

ALT_STATUS_CODE socfpga_watchdog_start(ALT_WDOG_TIMER_t tmr_id, ALT_WDOG_RESET_TYPE_t type, uint32_t val) {
    
    ALT_STATUS_CODE status = ALT_E_SUCCESS;

    uint32_t curr_val;

    status = alt_wdog_init();

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG init failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: WATCHDOG init SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    curr_val = alt_wdog_counter_get_current(tmr_id);
    ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);


    status = alt_wdog_response_mode_set(tmr_id,type);

    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG mode set failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: WATCHDOG mode set SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    status = alt_wdog_counter_set(tmr_id, val);
    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG timeout set failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: WATCHDOG timeout set SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    status = alt_wdog_start(tmr_id);
    if (status != ALT_E_SUCCESS)
    {
        ALT_PRINTF("ERROR: WATCHDOG timeout start failed, %" PRIi32 ".\n", status);
    } else {
        ALT_PRINTF("SUCCESS: WATCHDOG timeout start SUCCESSFUL, %" PRIi32 ".\n", status);
    }

    curr_val = alt_wdog_counter_get_current(tmr_id);
    ALT_PRINTF("SUCCESS: WATCHDOG current counter value is , %" PRIi32 ".\n", curr_val);

    curr_val=alt_read_word(0xFFD02000);

    ALT_PRINTF("SUCCESS: WATCHDOG control reg value is , %" PRIi32 ".\n", curr_val);




    return status;

}


