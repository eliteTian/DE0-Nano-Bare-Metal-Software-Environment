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
#include "hps_0.h"
#define ALT_RSTMGR_PERMODRST_OFST (0x14)
#define ALT_GPIO_BITMASK                0x1FFFFFFF
#define ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK (1<<25)
#define ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK (1<<26)
#define ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK (1<<27)
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define USER_IO_DIR     (0x01000000)
#define BIT_LED         (0x01000000)
#define BUTTON_MASK     (0x02000000)

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



