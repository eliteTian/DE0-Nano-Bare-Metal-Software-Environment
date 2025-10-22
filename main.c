#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"

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

int main(void) {
    //Uninit gpio module:
    alt_replbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK,
                      ALT_GPIO_BITMASK);

    //Init gpio module: rst manager deassert resets
    
    alt_replbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK |
                      ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK, 0);
    
    int scan_input;
    int i;
	alt_setbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DDR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), USER_IO_DIR );

	for(i=0;i<2;i++)
	{
        printf("LED blink twice!\r\n");
		alt_setbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
		mysleep(5000*1000);
		alt_clrbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
		mysleep(5000*1000);
	}


	while(1){
        printf("Test the button!\r\n");
        mysleep(5000*1000);
		scan_input = alt_read_word( ( ( ( uint32_t )(  ALT_GPIO1_EXT_PORTA_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ) );		
		if(~scan_input&BUTTON_MASK)
			alt_setbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
		else    
            alt_clrbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
	}	

    
	return( 0 );
}

void mysleep(uint32_t cycles) {
    while(cycles !=0) {
        cycles --;
    }
}




