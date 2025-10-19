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

#define ALT_WDOG_CTRL_REG_OFFSET (0x0)
#define ALT_WDOG_TMR_ENABLE (1<<0)

void mysleep(uint32_t cycles);

int main(void) {
    
	// initialize the pio controller
	// led: set the direction of the HPS GPIO1 bits attached to LEDs to output
    //int i;
    printf("Let's test the LEDs after disable watch dog!\r\n");
    alt_clrbits_word(ALT_CPU_WDTGPT_TMR0_BASE + ALT_WDOG_CTRL_REG_OFFSET, ALT_WDOG_TMR_ENABLE);
    alt_clrbits_word(ALT_CPU_WDTGPT_TMR1_BASE + ALT_WDOG_CTRL_REG_OFFSET, ALT_WDOG_TMR_ENABLE);

    printf("Watch dog disabled!\r\n");
    
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
		scan_input = alt_read_word( ( ( ( uint32_t )(  ALT_GPIO1_EXT_PORTA_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ) );		
		if(~scan_input&BUTTON_MASK)
			alt_setbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
		else    alt_clrbits_word( ( ( ( uint32_t )( ALT_GPIO1_SWPORTA_DR_ADDR ) & ( uint32_t )( HW_REGS_MASK ) ) ), BIT_LED );
	}	

    
	return( 0 );
}

void mysleep(uint32_t cycles) {
    while(cycles !=0) {
        cycles --;
    }
}


