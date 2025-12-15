/******************************************************************************
 *
 * Copyright 2017 Altera Corporation. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/
 
#include <stdio.h>
#include <string.h>
//#include "altx_ethernet.h"
#include "alt_ethernet.h"
#include "alt_cache.h"

#include "socal/alt_rstmgr.h"
#include "socal/alt_sysmgr.h"
#include "socal/alt_clkmgr.h"

#include "socal/socal.h"
#include "socal/hps.h"
#include "alt_interrupt.h"
#include "alt_printf.h"
#include "alt_cache.h"

#define REG_DBG 

#ifdef ALT_DEBUG_ETHERNET
    #define dprintf printf
#else
    #define dprintf null_printf
#endif


//void alt_dbg_reg(void* addr) {
//    uint32_t val;
//    val = alt_read_word(addr);
//    printf("Addr @ 0x%08x value is 0x%08x.\r\n", (unsigned int)(addr), (unsigned int)(val));
//}

//uint8_t MAC_ADDR[6] = { 0x82,0xa3,0xf5,0x17,0x9e,0xc1};

uint8_t MAC_ADDR[6] = { 0x00, 0x07, 0xed, 0x42, 0x9a, 0x48};
static uint8_t frame_buffer[ETH_BUFFER_SIZE];
//static uint8_t frame_buf0[ETH_BUFFER_SIZE];
//static uint8_t frame_buf1[ETH_BUFFER_SIZE];
//static uint8_t frame_buf2[ETH_BUFFER_SIZE];
//static uint8_t frame_buf3[ETH_BUFFER_SIZE];



void alt_dbg_reg(const char *name, void *addr) {
    uint32_t val = alt_read_word(addr);
    printf("Addr @ %s (0x%p) value = 0x%08x\r\n", name, addr, (unsigned int)val);
}
//static volatile uint8_t dumm_arr[786463];

void dump_ddr(void *addr, size_t len)
{
    uint32_t *p = (uint32_t *)addr;
    uint32_t *pa;
    uint32_t i;
    for (i = 0; i < len; i++) {
            pa = p;
            printf("DUMP RAW DDR[%u]: pa=0x%08x, data=0x%08x\n", i, pa, *p++);
    }

}

void dump_frame_buf(alt_eth_emac_instance_t * emac){
    uint32_t i;
    uint32_t *p = (uint32_t *)emac;
    uint32_t n = sizeof(*emac) / sizeof(uint32_t);
    uint32_t pa;
    for (i = 0; i < n; i++) {
        pa = (uint32_t)p;
        printf("POST_DBG[%u]: pa=0x%08x, data=0x%08x\n", i, pa, *p++);
    } 

    //for(i=0;i<NUMBER_OF_RX_DESCRIPTORS;i++) {
    //    for(j=0;j<ETH_BUFFER_SIZE;j++) {
    //        if(i==0) {
    //            printf("DUMP RAW data, data=0x%08x\n", frame_buf0[j]);
    //        }
    //        if(i==1) {
    //            printf("DUMP RAW data, data=0x%08x\n", frame_buf1[j]);
    //        }
    //        if(i==2) {
    //            printf("DUMP RAW data, data=0x%08x\n", frame_buf2[j]);
    //        }
    //        if(i==3) {
    //            printf("DUMP RAW data, data=0x%08x\n", frame_buf3[j]);
    //        }
    //    }
    //}
}

void discard_buff(alt_eth_emac_instance_t * emac) {
    //simply clear RX descriptor
    alt_eth_dma_desc_t * desc = &emac->rx_desc_ring[0];
    uint32_t i;
    //uint32_t j;    
    //uint8_t* buf_ptr;
    for(i=0;i<NUMBER_OF_RX_DESCRIPTORS;i++) {
        desc->status = ETH_DMARXDESC_OWN;
        desc = (alt_eth_dma_desc_t *) desc-> buffer2_next_desc_addr;
        //buf_ptr = (uint8_t*)desc-> buffer1_addr;
        ////store whatever is in rx buf after interrupt gets hit.
        //for(j=0;j<ETH_BUFFER_SIZE;j++) {
        //    //printf("DUMP RAW DESC[%u]: byte_num=%u, data=0x%02x", i, j, *(buf_ptr+j));
        //    //if(!(j%4)) { 
        //    //    printf("\n");
        //    //}
        //    if(i==0) {
        //        *(frame_buf0+j) = *(buf_ptr+j);
        //    }
        //    if(i==1) {
        //        *(frame_buf1+j) = *(buf_ptr+j);
        //    }
        //    if(i==2) {
        //        *(frame_buf2+j) = *(buf_ptr+j);
        //    }
        //    if(i==3) {
        //        *(frame_buf3+j) = *(buf_ptr+j);
        //    }

        //}

    }
}


//#define ALT_EMAC1_DMAGRP_ADDR        ALT_CAST(void *, (ALT_CAST(char *, ALT_EMAC1_ADDR) + ALT_EMAC1_DMAGRP_OFST))
 
/* Lookup tables for the emac registers 
   Cooresponding to Emac0, Emac1, and Emac 2 */
const void * Alt_Emac_Gmac_Grp_Addr[] = {
    ALT_EMAC0_GMACGRP_ADDR,
    ALT_EMAC1_GMACGRP_ADDR
};

const void * Alt_Emac_Dma_Grp_Addr[] = {
    ALT_EMAC0_DMAGRP_ADDR,
    ALT_EMAC1_DMAGRP_ADDR
};

static const uint32_t Alt_Rstmgr_Permodrst_Emac_Set_Msk[] = {
    ALT_RSTMGR_PERMODRST_EMAC0_SET_MSK,
    ALT_RSTMGR_PERMODRST_EMAC1_SET_MSK
};

static const uint32_t Alt_Sysmgr_Fpgaintf_Module_Emac_Set_Msk[] = {
    ALT_SYSMGR_FPGAINTF_MODULE_EMAC_0_SET_MSK,
    ALT_SYSMGR_FPGAINTF_MODULE_EMAC_1_SET_MSK

};


const void *  Alt_Sysmgr_Emac_Ctl_Addr[] = {
        ALT_SYSMGR_EMAC_ADDR
};

static const uint32_t Alt_Sysmgr_Emac_Ctl_Phy_Sel_Set_Msk[] = {
    ALT_SYSMGR_EMAC_CTL_PHYSEL_0_SET_MSK,  //3
    ALT_SYSMGR_EMAC_CTL_PHYSEL_1_SET_MSK   //C
};

/* Delay function used during ethernet setup */
void alt_eth_delay(volatile uint32_t delay)
{
    volatile uint32_t i; 
    
    for(i = delay; i != 0; i--);
}

/*  Reset the EMAC, Disable the FPGA Interface, and set the PHY mode  */
void alt_eth_reset_mac(uint32_t instance)
{
    if (instance > 1) { return; }
     
    /* Reset the EMAC */
    alt_setbits_word(ALT_RSTMGR_PERMODRST_ADDR, Alt_Rstmgr_Permodrst_Emac_Set_Msk[instance]);
    
    /* Program the phy_intf_sel field of the emac* register in the System Manager to select
       RGMII PHY interface. */
    //reg: sysmgr.ctrl @ physel_0/1
    //printf( "Hufei: check ALT_RSTMGR_PERMODRST_ADDR\r\n" );
    //alt_dbg_reg("ALT_RSTMGR_PERMODRST_ADDR",ALT_RSTMGR_PERMODRST_ADDR);

    
            //alt_replbits_word(ALT_SYSMGR_EMAC_CTL_ADDR,
            //          Alt_Sysmgr_Emac_Ctl_Phy_Sel_Set_Msk[instance],  
            //          ALT_SYSMGR_EMAC_CTL_PHYSEL_1_E_RGMII); //bug here it was mistaken as PHYSEL_0



                    
    /* Disable the Ethernet Controller FPGA interface by clearing the emac_* bit in the fpgaintf_en_3
      register of the System Manager. */
    //reg: 
    //  alt_clrbits_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR, Alt_Sysmgr_Fpgaintf_Module_Emac_Set_Msk[instance]); 
    alt_clrbits_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR, Alt_Sysmgr_Fpgaintf_Module_Emac_Set_Msk[0]);           
    alt_clrbits_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR, Alt_Sysmgr_Fpgaintf_Module_Emac_Set_Msk[1]);           
                    
    /* clear the emac* bit in the permodrst register of
       the Reset Manager to bring the EMAC out of reset. */
    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR, Alt_Rstmgr_Permodrst_Emac_Set_Msk[instance]);
    
    //printf( "Hufei: check RST and SYS MGR reg values\r\n" );
    //alt_dbg_reg("ALT_RSTMGR_PERMODRST_ADDR",ALT_RSTMGR_PERMODRST_ADDR);
    //alt_dbg_reg("ALT_SYSMGR_EMAC_CTL_ADDR",ALT_SYSMGR_EMAC_CTL_ADDR);
    //alt_eth_delay(20000*2000);
      
}   

void alt_eth_emac_dma_init(alt_eth_emac_instance_t * emac){

    if (emac->instance > 1) { return; }

    systemConfig(emac->instance); 
    emacHPSIFInit(emac->instance);
    dmaInit(emac);
    emacInit(emac);
    printf( "Hufei: emac and dma all initialized\r\n" );

}

void systemConfig(uint32_t instance) {
    if (instance > 1) { return; }
#ifdef REG_DBG
    uint32_t dbg_addr, dbg_data;
    dbg_addr = (uint32_t) ALT_SYSMGR_EMAC_CTL_ADDR;
    dbg_data = alt_read_word(ALT_SYSMGR_EMAC_CTL_ADDR);
    dprintf("DBG: Addr: 0x%08x,Data=0x%08x\r\n",dbg_addr,dbg_data);

    alt_replbits_word(ALT_SYSMGR_EMAC_CTL_ADDR, Alt_Sysmgr_Emac_Ctl_Phy_Sel_Set_Msk[instance],  ALT_SYSMGR_EMAC_CTL_PHYSEL_1_SET(ALT_SYSMGR_EMAC_CTL_PHYSEL_1_E_RGMII)); 
    dbg_addr = (uint32_t) ALT_SYSMGR_EMAC_CTL_ADDR;
    dbg_data = alt_read_word(ALT_SYSMGR_EMAC_CTL_ADDR);
    dprintf("DBG: Addr: 0x%08x,Data=0x%08x\r\n",dbg_addr,dbg_data);
#else
    alt_replbits_word(ALT_SYSMGR_EMAC_CTL_ADDR, Alt_Sysmgr_Emac_Ctl_Phy_Sel_Set_Msk[instance],  ALT_SYSMGR_EMAC_CTL_PHYSEL_1_SET(ALT_SYSMGR_EMAC_CTL_PHYSEL_1_E_RGMII)); 
#endif
}

void emacHPSIFInit(uint32_t instance){
    if (instance > 1) { return; }
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
#ifdef REG_DBG
    uint32_t dbg_addr, dbg_data;
#endif
    //1. After the HPS is released from cold or warm reset, reset the Ethernet Controller module by setting the appropriate emac bit in the permodrst register in the Reset Manager
    alt_setbits_word(ALT_RSTMGR_PERMODRST_ADDR, Alt_Rstmgr_Permodrst_Emac_Set_Msk[instance]);
    //2. Configure the EMAC Controller clock to 250 MHz by programming the appropriate cnt value in the emac*clk register in the Clock Manager.
    //Done by U-boot/Preloader
    //3. Bring the Ethernet PHY out of reset to verify that there are RX PHY clocks.
    //The description in the doc is not correct here above, emac1 rst bit in the rst manager has to be cleared before MIIM IF can be functional.
    //so this step 3 is commented out. Register content was checked and confirmed MIIM is not functional when emac1 rst bit is asserted.
    //status = alt_eth_phy_reset(instance);
    //if (status != ALT_E_SUCCESS) { 
    //    printf( "ERROR: PHY reset unsuccessful\r\n" );
    //    return ;
    //} else {
    //    printf( "Hufei: PHY reset successful\r\n" );
    //}
    //alt_eth_delay(100000000);
    
    //TODO: Q: why here you can't configure PHY, before EMAC deassert reset? A: It's because rst has to be deasserted.
    //NOTE 2: After the de-assertion of reset, wait a minimum of 100µs before starting programming on the MIIM (MDC/MDIO) interface.
    //alt_eth_delay(100000); //clock speed is 925M, close to 1G, so t cycle is close to 1ns. each cpu cycle is more than 1 clk cycle, so safe.
    //status = alt_eth_phy_config(instance);
    //if (status != ALT_E_SUCCESS) { 
    //    printf( "ERROR: PHY config unsuccessful\r\n" );
    //} else {
    //    printf( "Hufei: PHY config Done\r\n" );
    //}




    //4. When all the clocks are valid, program the following clock settings
        //a.Set the physel_* field in the ctrl register of the System Manager (EMAC Group) to 0x1 to select the RGMII PHY interface.
    //Done in systemConfig
        //b.Disable the Ethernet Controller FPGA interfaces by clearing the emac_* bit in the module register of the System Manager (FPGA Interface group)
    alt_clrbits_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR, Alt_Sysmgr_Fpgaintf_Module_Emac_Set_Msk[instance]);
    //5.Configure all of the EMAC static settings if the user requires a different setting from the default value.These settings include AXI AxCache signal values, 
    //which are programmed in l3 register in the EMAC group of the System Manager.
    //6.Execute a register read back to confirm the clock and static configuration settings are valid
#ifdef REG_DBG   
    dbg_addr = (uint32_t) ALT_SYSMGR_FPGAINTF_MODULE_ADDR;
    dbg_data = alt_read_word(ALT_SYSMGR_FPGAINTF_MODULE_ADDR);
    dprintf("DBG: Addr: 0x%08x,Data=0x%08x\r\n",dbg_addr,dbg_data);
#endif
    //7.After confirming the settings are valid, software can clear the emac bit in the permodrst register of the Reset Manager to bring the EMAC out of reset..
    alt_clrbits_word(ALT_RSTMGR_PERMODRST_ADDR, Alt_Rstmgr_Permodrst_Emac_Set_Msk[instance]);    
    //true reset happening here
    status = alt_eth_phy_reset(instance);
    if (status != ALT_E_SUCCESS) { 
        dprintf( "ERROR: PHY reset unsuccessful\r\n" );
        return ;
    } else {
        dprintf( "Hufei: PHY reset successful\r\n" );
    }


    //Note* PHY can't be configured when EMAC is in reset. 
    status = alt_eth_phy_1g_config(instance);
    if (status != ALT_E_SUCCESS) { 
        dprintf( "ERROR: Hufei: PHY config unsuccessful\r\n" );
        return ;
    } else {
        dprintf( "Hufei: PHY config Done\r\n" );
    }

    
}




void dmaInit(alt_eth_emac_instance_t * emac) {
    if (emac->instance > 1) { return; }
    uint32_t interrupt_mask;
    uint32_t dbg_addr, dbg_data;
    
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    status = alt_eth_software_reset(emac->instance);
    
    //1.Provide a software reset to reset all of the EMAC internal registers and logic. (DMA Register 0 (Bus Mode Register) – bit 0).
    //2. Wait for the completion of the reset process (poll bit 0 of the DMA Register 0 (Bus Mode Register), 
    // which is only cleared after the reset operation is completed).
    if (status != ALT_E_SUCCESS) { 
        dprintf( "ERROR: Ethenet soft reset not successful\r\n" );
        return;
    }  
    //3.Poll the bits of Register 11 (AHB or AXI Status) to confirm that all previously initiated (before
    //software reset) or ongoing transactions are complete      
    if (alt_read_word(ALT_EMAC_DMA_AHB_OR_AXI_STAT_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance])) != 0) {
        return;
    }
    //4.Program the following fields to initialize the Bus Mode Register by setting values in DMA Register 0
    // Set the DMA Bus Mode Register
 
    dbg_addr = (uint32_t) (ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]));
    dbg_data = alt_read_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]));
    dprintf("DBG: Addr: 0x%08x,Data=0x%08x\r\n",dbg_addr,dbg_data);

    alt_write_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]), //need to use replace bits than direct write
        
          ALT_EMAC_DMA_BUS_MOD_USP_SET_MSK          /* Use separate PBL */ 
          /*| ALT_EMAC_DMA_BUS_MOD_AAL_SET_MSK      */     /* Address Aligned Beats */
          /*| ALT_EMAC_DMA_BUS_MOD_EIGHTXPBL_SET(1) */
          | ALT_EMAC_DMA_BUS_MOD_PBL_SET(32)            /* Programmable Burst Length */
          /*| ALT_EMAC_DMA_BUS_MOD_RPBL_SET(8)      */     /* Programmable Burst Length */
         /* |   ALT_EMAC_DMA_BUS_MOD_FB_SET_MSK   */          /* Fixed Burst */
          //  ALT_EMAC_DMA_BUS_MOD_FB_SET(1)
        ); 

    //DBG: Addr: 0xff703000,Data=0x00020100
    //DBG: Addr: 0xff703000,Data=0x00030100
    dbg_addr = (uint32_t) (ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]));
    dbg_data = alt_read_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]));
    dprintf("DBG: Addr: 0x%08x,Data=0x%08x\r\n",dbg_addr,dbg_data);
    
    
    //5.Program the interface options in Register 10 (AXI Bus Mode Register). If fixed burst-length is enabled,
    //then select the maximum burst-length possible on the bus (bits[7:1]).†
    //6.Create a proper descriptor chain for transmit and receive. In addition, ensure that the receive descrip‐
    //tors are owned by DMA (bit 31 of descriptor should be set). When OSF mode is used, at least two
    //descriptors are required.
    //7.8 Initialize receive and transmit descriptor list address with the base address of the transmit and receive
    //descriptor
    alt_eth_setup_rxdesc(emac);
    alt_eth_setup_txdesc(emac);
    //9.Program the following fields to initialize the mode of operation in Register 6
    alt_write_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]), ALT_EMAC_DMA_OP_MOD_TSF_SET_MSK);

    
    //10.Clear the interrupt requests, by writing to those bits of the status register (interrupt bits only) that are set
    interrupt_mask = ALT_EMAC_DMA_INT_EN_NIE_SET_MSK |
                     /*ALT_EMAC_DMA_INT_EN_AIE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_ERE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_FBE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_ETE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_RWE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_RSE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_RUE_SET_MSK |*/
                     ALT_EMAC_DMA_INT_EN_RIE_SET_MSK |
                     /*ALT_EMAC_DMA_INT_EN_TUE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_OVE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_TJE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_UNE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_TSE_SET_MSK |*/
                     ALT_EMAC_DMA_INT_EN_TIE_SET_MSK;              
     emac->interrupt_mask = interrupt_mask;
     /* clear the interrupt requests */
     alt_eth_dma_clear_status_bits(interrupt_mask, emac->instance);
     /* Enable Interrupts */    
     alt_eth_dma_set_irq_reg(interrupt_mask, ALT_ETH_ENABLE, emac->instance);      
     /* confirm that all previous transactions are complete */    
     if (alt_read_word(ALT_EMAC_DMA_AHB_OR_AXI_STAT_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance])) != 0){
        return;
     }

}


void emacInit(alt_eth_emac_instance_t * emac) {
    if (emac->instance > 1) { return; }
    uint32_t alt_mac_config_reg_settings = 0;
    uint32_t phy_duplex_status, phy_speed;
    ALT_STATUS_CODE status = ALT_E_SUCCESS;    

    /* Set the Gmac Configuration register */
    alt_mac_config_reg_settings = (ALT_EMAC_GMAC_MAC_CFG_IPC_SET_MSK           /* Checksum Offload */
                                   | ALT_EMAC_GMAC_MAC_CFG_JD_SET_MSK          /* Jabber Disable */
                                   | ALT_EMAC_GMAC_MAC_CFG_PS_SET_MSK          /* Port Select = MII */
                                   | ALT_EMAC_GMAC_MAC_CFG_BE_SET_MSK          /* Frame Burst Enable */
                                   | ALT_EMAC_GMAC_MAC_CFG_WD_SET_MSK          /* Watchdog Disable */
                                   /*| ALT_EMAC_GMAC_MAC_CFG_DO_SET_MSK  */
                                   );  
    
    /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
    status = alt_eth_phy_get_duplex_and_speed(&phy_duplex_status, &phy_speed, emac->instance);
    if (status != ALT_E_SUCCESS) { return ; }
    
    if (phy_duplex_status != ALT_ETH_RESET)
    {
        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
        alt_mac_config_reg_settings |= ALT_EMAC_GMAC_MAC_CFG_DM_SET_MSK;  
    }
       
    /* Configure the MAC with the speed fixed by the auto-negotiation process */
    if (phy_speed == 100) {
        /* Set Ethernet speed to 100M following the auto-negotiation */ 
        alt_mac_config_reg_settings |= ALT_EMAC_GMAC_MAC_CFG_FES_SET_MSK;  
        dprintf("Auto Negotiation speed = 100\n");       
    }  else if (phy_speed == 1000) {
        /* Set Ethernet speed to 1G following the auto-negotiation */ 
        alt_mac_config_reg_settings &= ALT_EMAC_GMAC_MAC_CFG_PS_CLR_MSK;     
        dprintf("Auto Negotiation speed = 1000\n"); 
    } 
        //set mac addr
    alt_eth_mac_set_mac_addr(MAC_ADDR,emac->instance);

    /* Disable MAC interrupts */
    alt_eth_mac_set_irq_reg(ALT_EMAC_GMAC_INT_STAT_LPIIS_SET_MSK |   /* Disable Low Power IRQ */
                        ALT_EMAC_GMAC_INT_STAT_TSIS_SET_MSK |       /* Disable Timestamp IRQ */
                        ALT_EMAC_GMAC_INT_STAT_RGSMIIIS_SET_MSK,   /*Disable RGMII/SMII IRQ */
                        ALT_ETH_DISABLE, emac->instance);
    
    dprintf( "CRITICAL: GMAC config register to be written is 0x%08x,0x%08x,0x%08x  \r\n",alt_mac_config_reg_settings,phy_speed,phy_duplex_status );
    alt_write_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[emac->instance]), alt_mac_config_reg_settings);
        /* Disable promiscuous mode */
    alt_replbits_word(ALT_EMAC_GMAC_MAC_FRM_FLT_ADDR(Alt_Emac_Gmac_Grp_Addr[emac->instance]),1, 0);  
    dprintf("Initializing irq handler\n");   
    /* Initialize the ethernet irq handler */   
    alt_eth_irq_init(emac, alt_eth_irq_callback);
    printf("Initializing irq handler done\n");   
    /*start DMA for operation*/
    alt_eth_start(emac->instance);
}





void alt_eth_reinit_rxdesc(alt_eth_emac_instance_t * emac)
{
    alt_eth_dma_set_rx_state(ALT_ETH_DISABLE, emac->instance); 
    alt_eth_setup_rxdesc(emac);
    alt_eth_dma_set_rx_state(ALT_ETH_ENABLE, emac->instance); 
}

void alt_eth_setup_rxdesc(alt_eth_emac_instance_t * emac)
{
    int32_t i;
  
    alt_eth_dma_desc_t *rx_desc;

    for (i = 0; i < NUMBER_OF_RX_DESCRIPTORS; i++)
    {
        rx_desc = &emac->rx_desc_ring[i];
        rx_desc->buffer1_addr = (uint32_t) &emac->rx_buf[i * ETH_BUFFER_SIZE];
        rx_desc->control_buffer_size = ETH_DMARXDESC_RCH | ETH_BUFFER_SIZE;       
     
        /*set own bit*/
        rx_desc->status = ETH_DMARXDESC_OWN;     
     
        rx_desc->buffer2_next_desc_addr = (uint32_t)&emac->rx_desc_ring[i+1];
        if (i == (NUMBER_OF_RX_DESCRIPTORS - 1))
        {
            rx_desc->buffer2_next_desc_addr = (uint32_t)&emac->rx_desc_ring[0];
        }           
    }

    emac->rx_current_desc_number = 0;
    emac->rx_processed_desc_number = 0;
    emac->rxints = 0;
  
    /* Set RX Descriptor List Address Register */
    alt_eth_dma_set_rx_desc_addr((uint32_t) &emac->rx_desc_ring[0], emac->instance); 
}

void alt_eth_setup_txdesc(alt_eth_emac_instance_t * emac)
{
    int32_t i;
  
    alt_eth_dma_desc_t *tx_desc;

    for (i = 0; i < NUMBER_OF_TX_DESCRIPTORS; i++)
    {
        tx_desc = &emac->tx_desc_ring[i];
        tx_desc->buffer1_addr = (uint32_t)NULL;
        tx_desc->buffer2_next_desc_addr = (uint32_t)&emac->tx_desc_ring[i+1];
        tx_desc->status = 0;
        tx_desc->control_buffer_size = 0;
    
        if (i == (NUMBER_OF_TX_DESCRIPTORS - 1))
        {
            tx_desc->buffer2_next_desc_addr = (uint32_t)&emac->tx_desc_ring[0];
        }      
    }

    emac->tx_current_desc_number = 0;
    emac->txints = 0;
  
    /* Set TX Descriptor List Address Register */
    alt_eth_dma_set_tx_desc_addr((uint32_t) &emac->tx_desc_ring[0], emac->instance);
    //alt_eth_dma_set_tx_desc_addr(((uint32_t) &emac->tx_desc_ring[0] + 4), emac->instance); //dbg purpose, sabotage

  
}

ALT_STATUS_CODE alt_eth_irq_init(alt_eth_emac_instance_t * emac, alt_int_callback_t callback)
{
    
    ALT_STATUS_CODE status = ALT_E_SUCCESS;    
    uint32_t int_target = 0;
    if (emac->instance==0) { emac->irqnum = ALT_INT_INTERRUPT_EMAC0_IRQ; }
    if (emac->instance==1) { emac->irqnum = ALT_INT_INTERRUPT_EMAC1_IRQ; }
         
    /* Ethernet IRQ Callback */
    status = alt_int_isr_register( emac->irqnum,
                            callback,
                            (void *)emac);
    
    dprintf("IRQ routine registered\n");   

    /* Configure the EMAC as Level. */
    if (status == ALT_E_SUCCESS)
    {
        status = alt_int_dist_trigger_set(emac->irqnum,
                                         ALT_INT_TRIGGER_AUTODETECT);
        dprintf("IRQ routine trigger set registered\n");    
    }
 
    /* Configure the EMAC priority */
    if (status == ALT_E_SUCCESS)
    {  
        status = alt_int_dist_priority_set(emac->irqnum, 16);
        dprintf("IRQ routine priority set registered\n");    
    }
  
    /* Set CPUs 0 and 1 as the target. */
    //if (status == ALT_E_SUCCESS)
    //{                    
    //    status = alt_int_dist_target_set(emac->irqnum, 3); //only cpu0 is there
    //    printf("IRQ target CPU set registered\n");  
    //}

    if (   (status == ALT_E_SUCCESS)
        && (emac->irqnum >= 32)) /* Ignore target_set() for non-SPI interrupts. */
    {
        if (int_target == 0)
        {
            int_target = (1 << alt_int_util_cpu_count()) - 1;
        }
        status = alt_int_dist_target_set(emac->irqnum, int_target);
        dprintf("IRQ target CPU set registered\n");          
    }    
    
    /* Enable the interrupt in the Distributor. */
    if (status == ALT_E_SUCCESS)
    {
        status = alt_int_dist_enable(emac->irqnum);
        dprintf("IRQ enabled in distributor\n");  
    }
 
    return status;
}

void alt_eth_irq_callback(uint32_t icciar, void * context)
{
    printf("IRQ callback called\n");  
    uint32_t status,dma_status;
    alt_eth_emac_instance_t * emac;
    uint32_t rcv_len = 0;
   
    emac = context;
    alt_eth_delay(ETH_RESET_DELAY);
  
    /* Check to make sure this is a valid interrupt. */
    if (icciar != emac->irqnum) { return; }

    //status = alt_read_word(0xFF702038); //check if there is link status change
    ////printf("Call back reading gmac int status is 0x%08x \n",status );
    //if(status) {
    //    status = alt_read_word(0xFF7020D8); //read S/RGMII control status register to clear interrupt
    //}
      
    status = alt_eth_dma_get_status_reg(emac->instance) & emac->interrupt_mask;
    //printf("Call back reading dma int status is 0x%08x \n",status );

    
                                         
    if (status & ALT_EMAC_DMA_INT_EN_NIE_SET_MSK )  
    {
        alt_eth_dma_clear_status_bits(ALT_EMAC_DMA_INT_EN_NIE_SET_MSK, emac->instance);     
        printf( "Hufei: Normal Interrupt request asserted\r\n" );
    }
    
    if (status & ALT_EMAC_DMA_INT_EN_TIE_SET_MSK ) 
    {
        emac->txints++;
        alt_eth_dma_clear_status_bits(ALT_EMAC_DMA_INT_EN_TIE_SET_MSK, emac->instance);    
        printf( "Hufei: TX Interrupt request asserted\r\n" );
        
    }
        
    if (status & ALT_EMAC_DMA_INT_EN_RIE_SET_MSK)
    {
        emac->rxints++;    
        alt_eth_dma_clear_status_bits( ALT_EMAC_DMA_INT_EN_RIE_SET_MSK, emac->instance);   
        printf( "Hufei: Receive Interrupt request asserted\r\n" );
        discard_buff(emac);
        dma_status = alt_read_word(0xFF703014);
        alt_eth_get_packet(frame_buffer,&rcv_len,emac);
        printf("Call back reading dma int status is 0x%08x, received packet len is 0x%08x \n", dma_status, rcv_len );
        
        scatter_frame(frame_buffer,rcv_len,emac);
        
    }

}

/*  Ethernet API Functions  */
ALT_STATUS_CODE alt_eth_dma_mac_config(alt_eth_emac_instance_t * emac)
{
    uint32_t tmpreg = 0, interrupt_mask;
    uint32_t alt_mac_config_reg_settings = 0;
    uint32_t phy_duplex_status, phy_speed;
    ALT_STATUS_CODE status;  
    
    if (emac->instance > 1) { return ALT_E_ERROR; }        
    
    /* Reset the EMAC */
    alt_eth_reset_mac(emac->instance);



     
    /* Reset the PHY  */
    status = alt_eth_phy_reset(emac->instance);
    if (status != ALT_E_SUCCESS) { 
        printf( "ERROR: Hufei: PHY reset unsuccessful\r\n" );
        return status;

    }


    //alt_eth_phy_dump_all(emac->instance);
    //printf( "Hufei: Before PHY config PHY regs dump done\r\n" );  



    
    
    /* Configure the PHY */
    //status = alt_eth_phy_config(emac->instance);
    //if (status != ALT_E_SUCCESS) { return status; }    

    status = alt_eth_phy_1g_config(emac->instance);
    if (status != ALT_E_SUCCESS) { return status; }    

    //printf( "Hufei: PHY config done\r\n" );

    //status = alt_eth_phy_loopback(1,emac->instance);
    //if (status != ALT_E_SUCCESS) { return status; }    

    //printf( "Hufei: PHY loopback done\r\n" );
    //
    //alt_eth_phy_dump_all(emac->instance);
    //printf( "Hufei: After PHY config PHY regs dump done\r\n" );  
    /* Reset the Ethernet */
    status = alt_eth_software_reset(emac->instance);
    if (status != ALT_E_SUCCESS) { 
        printf( "Hufei: Ethenet soft reset not successful\r\n" );
        return status;
    }    

    printf( "Hufei: Ethenet reset done\r\n" );

    
        
    /* note: this does not mean Enhanced Descriptor Format which is always used in A10/S10 */
    #ifdef USE_ALTERNATE_DESCRIPTOR_SIZE          
        tmpreg=ALT_EMAC_DMA_BUS_MOD_ATDS_SET_MSK; /* Alternate Descriptor Size */
    #endif
    
    /* Set the DMA Bus Mode Register */
    alt_write_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]), 
        (tmpreg
          /*| ALT_EMAC_DMA_BUS_MOD_USP_SET_MSK      */     /* Use separate PBL */ 
          /*| ALT_EMAC_DMA_BUS_MOD_AAL_SET_MSK      */     /* Address Aligned Beats */
          /*| ALT_EMAC_DMA_BUS_MOD_EIGHTXPBL_SET(1) */
          /*| ALT_EMAC_DMA_BUS_MOD_PBL_SET(8)       */     /* Programmable Burst Length */
          /*| ALT_EMAC_DMA_BUS_MOD_RPBL_SET(8)      */     /* Programmable Burst Length */
            | ALT_EMAC_DMA_BUS_MOD_FB_SET_MSK              /* Fixed Burst */
        )); 
    
    /* Initialize the tx and rx descriptors */
    alt_eth_setup_rxdesc(emac);
    alt_eth_setup_txdesc(emac);
         
    /* set the DMA OP Mode Regsiter */     
    alt_write_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance]),   
        (0 
         /*| ALT_EMAC_DMA_OP_MOD_OSF_SET_MSK   */   /* Operate on Second Frame */ 
         /*| ALT_EMAC_DMA_OP_MOD_TSF_SET_MSK   */   /* Transmit Store and Forward */
         /*| ALT_EMAC_DMA_OP_MOD_RSF_SET_MSK   */   /* Receive Store and Forward */
         /*| ALT_EMAC_DMA_OP_MOD_FTF_SET_MSK   */   /* Receive Store and Forward */         
        )); 
                                     
    interrupt_mask = ALT_EMAC_DMA_INT_EN_NIE_SET_MSK |
                     /*ALT_EMAC_DMA_INT_EN_AIE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_ERE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_FBE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_ETE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_RWE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_RSE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_RUE_SET_MSK |*/
                     ALT_EMAC_DMA_INT_EN_RIE_SET_MSK |
                     /*ALT_EMAC_DMA_INT_EN_TUE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_OVE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_TJE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_UNE_SET_MSK |
                     ALT_EMAC_DMA_INT_EN_TSE_SET_MSK |*/
                     ALT_EMAC_DMA_INT_EN_TIE_SET_MSK;              
 
     emac->interrupt_mask = interrupt_mask;
      
     /* clear the interrupt requests */
     alt_eth_dma_clear_status_bits(interrupt_mask, emac->instance);
     
     /* Enable Interrupts */    
     alt_eth_dma_set_irq_reg(interrupt_mask, ALT_ETH_ENABLE, emac->instance);                     
                     
     /* confirm that all previous transactions are complete */    
     if (alt_read_word(ALT_EMAC_DMA_AHB_OR_AXI_STAT_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance])) != 0)
     {
        return ALT_E_ERROR;
     }
     
    /* Set the Gmac Configuration register */
    alt_mac_config_reg_settings = (ALT_EMAC_GMAC_MAC_CFG_IPC_SET_MSK           /* Checksum Offload */
                                   | ALT_EMAC_GMAC_MAC_CFG_JD_SET_MSK          /* Jabber Disable */
                                   | ALT_EMAC_GMAC_MAC_CFG_PS_SET_MSK          /* Port Select = MII */
                                   | ALT_EMAC_GMAC_MAC_CFG_BE_SET_MSK          /* Frame Burst Enable */
                                   | ALT_EMAC_GMAC_MAC_CFG_WD_SET_MSK          /* Watchdog Disable */
                                   /*| ALT_EMAC_GMAC_MAC_CFG_DO_SET_MSK  */
                                   );  
    
    /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
    status = alt_eth_phy_get_duplex_and_speed(&phy_duplex_status, &phy_speed, emac->instance);
    if (status != ALT_E_SUCCESS) { return status; }
    
    if (phy_duplex_status != ALT_ETH_RESET)
    {
        /* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
        alt_mac_config_reg_settings |= ALT_EMAC_GMAC_MAC_CFG_DM_SET_MSK;  
    }
       
    /* Configure the MAC with the speed fixed by the auto-negotiation process */
    if (phy_speed == 100)
    {
        /* Set Ethernet speed to 100M following the auto-negotiation */ 
        alt_mac_config_reg_settings |= (ALT_EMAC_GMAC_MAC_CFG_FES_SET_MSK | ALT_EMAC_GMAC_MAC_CFG_PS_SET_MSK);  
        dprintf("Auto Negotiation speed = 100\n");       
    } else if (phy_speed == 1000)
    {
        /* Set Ethernet speed to 1G following the auto-negotiation */ 
        alt_mac_config_reg_settings &= ALT_EMAC_GMAC_MAC_CFG_PS_CLR_MSK;     
        dprintf("Auto Negotiation speed = 1000\n"); 
    } 

    //set mac addr
    alt_eth_mac_set_mac_addr(MAC_ADDR,emac->instance);
    
    /* Read the MII Status Register to clear the changed flag */
    alt_eth_mac_get_mii_link_state(emac->instance);
    
    /* Disable MAC interrupts */
    alt_eth_mac_set_irq_reg(ALT_EMAC_GMAC_INT_STAT_LPIIS_SET_MSK |   /* Disable Low Power IRQ */
                        ALT_EMAC_GMAC_INT_STAT_TSIS_SET_MSK,         /* Disable Timestamp IRQ */
                        ALT_ETH_DISABLE, emac->instance); 
                        
    /* Set the Gmac Configuration Register */

    
    printf( "CRITICAL: GMAC config register to be written is 0x %x\r\n",(unsigned int)alt_mac_config_reg_settings );

    alt_write_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[emac->instance]),
                   alt_mac_config_reg_settings);


    alt_mac_config_reg_settings = alt_read_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[emac->instance]));

    printf( "CRITICAL: GMAC config register readout is 0x %x\r\n",(unsigned int)alt_mac_config_reg_settings );


    
    /* Disable promiscuous mode */
    alt_replbits_word(ALT_EMAC_GMAC_MAC_FRM_FLT_ADDR(Alt_Emac_Gmac_Grp_Addr[emac->instance]),1, 0);  
       
    /* Initialize the ethernet irq handler */   
    alt_eth_irq_init(emac, alt_eth_irq_callback);
    
    /* Start the receive and transmit DMA */
    //alt_eth_start(emac->instance);
    
    /* Return Ethernet configuration success */
    return ALT_E_SUCCESS;
}

void alt_eth_mac_set_tx_state(alt_eth_enable_disable_state_t new_state, uint32_t instance)
{ 
    if (instance > 2) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Enable the MAC transmission */
        alt_setbits_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         ALT_EMAC_GMAC_MAC_CFG_TE_SET_MSK);
    }
    else
    {
        /* Disable the MAC transmission */
        alt_clrbits_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         ALT_EMAC_GMAC_MAC_CFG_TE_SET_MSK);  
    }
}

void alt_eth_mac_set_rx_state(alt_eth_enable_disable_state_t new_state, uint32_t instance)
{ 
    if (instance > 2) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Enable the MAC reception */ 
        alt_setbits_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         ALT_EMAC_GMAC_MAC_CFG_RE_SET_MSK);
    }
    else
    {
        /* Disable the MAC reception */
        alt_clrbits_word(ALT_EMAC_GMAC_MAC_CFG_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         ALT_EMAC_GMAC_MAC_CFG_RE_SET_MSK);
    }
}



void dumpRegs(){
    uint32_t stat,addr;
    //stat=alt_read_word(((ALTX_EMAC_GMACGRP_MAC_CONFIGURATION_ADDR(Alt_Emac_Addr[instance]))));
    //printf( "GMAC_CONFIG status before eth_start:  0x%08x\n", stat );
    //fflush(stderr);



    addr = 0xFF702000;
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702004;
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702010;
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702014;
    stat = alt_read_word( addr);   
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702018;     
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF70201C; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702020; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702024; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);   

    addr = 0xFF702038; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF70203C; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702040; 
    stat = alt_read_word( addr);
    printf("DBG: MAC_ADDR0_high is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702044; 
    stat = alt_read_word( addr);
    printf("DBG: MAC_ADDR0_low is 0x%08x,0x%08x !\n",addr,stat);

// more info.
    addr = 0xFF7020D8; 
    stat = alt_read_word( addr);
    printf("DBG: SGMII_RGMII_SMII_Control Status is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);

    addr = 0xFF702114; 
    stat = alt_read_word( addr);
    printf("DBG: Number of bytes transmitted in good and bad frames is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);

    addr = 0xFF702118; 
    stat = alt_read_word( addr);
    printf("DBG: Number of good and bad Frames transmitted  is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);

    addr = 0xFF70211C; 
    stat = alt_read_word( addr);
    printf("DBG: Number of broadcast frames transmitted  is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);
 
    addr = 0xFF702124; 
    stat = alt_read_word( addr);
    printf("DBG: Number of good and bad frames transmitted with length 64 bytes is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);

    addr = 0xFF702128; 
    stat = alt_read_word( addr);
    printf("DBG: Number of good and bad frames transmitted with length 64-127 bytes is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);
    

    addr = 0xFF702164; 
    stat = alt_read_word( addr);
    printf("DBG: Number of bytes transmitted, exclusive of preamble, in good frames only is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);

    addr = 0xFF702168; 
    stat = alt_read_word( addr);
    printf("DBG: Number of good frames transmitted is 0x%08x,0x%08x !\t\t\t\t\t\n",addr,stat);

    addr = 0xFF702180; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF702184; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
    addr = 0xFF702188; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);


    addr = 0xFF703000; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF70300C; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF703010; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
    addr = 0xFF703014; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
    addr = 0xFF703018; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
    addr = 0xFF70301C; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF703048; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
    addr = 0xFF70304C; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);

    addr = 0xFF703050; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
    addr = 0xFF703054; 
    stat = alt_read_word( addr);
    printf("DBG: DBG_REG is 0x%08x,0x%08x !\n",addr,stat);
    
  



}


void alt_eth_start(uint32_t instance){
    //dumpRegs();
    printf("FIFO not flushed yet and bit not cleared!\r\n");
     /* Enable transmit state machine of the MAC for transmission on the MII */  
    alt_eth_mac_set_tx_state(ALT_ETH_ENABLE, instance);
    
    /* Flush Transmit FIFO */
    alt_eth_dma_flush_tx_fifo(instance);
    
    /* Enable receive state machine of the MAC for reception from the MII */  
    alt_eth_mac_set_rx_state(ALT_ETH_ENABLE, instance);
    
    /* Start DMA transmission */
    alt_eth_dma_set_tx_state(ALT_ETH_ENABLE, instance); 
    
    /* Start DMA reception */
    alt_eth_dma_set_rx_state(ALT_ETH_ENABLE, instance); 
    
    alt_eth_delay(ETH_RESET_DELAY);

    //dumpRegs();
    
    
}

void alt_eth_stop(uint32_t instance)
{
    
    /* Stop DMA transmission */
    alt_eth_dma_set_tx_state(ALT_ETH_DISABLE, instance); 
    
    /* Stop DMA reception */
    alt_eth_dma_set_rx_state(ALT_ETH_DISABLE, instance);
     
    /* Disable transmit state machine of the MAC for transmission on the MII */  
    alt_eth_mac_set_tx_state(ALT_ETH_DISABLE, instance);
    
    /* Flush Transmit FIFO */
    alt_eth_dma_flush_tx_fifo(instance);
    
    /* Disable receive state machine of the MAC for reception from the MII */  
    alt_eth_mac_set_rx_state(ALT_ETH_DISABLE, instance);
          
    alt_eth_delay(ETH_RESET_DELAY);  
}

alt_eth_enable_disable_state_t alt_eth_mac_get_bpa_state(uint32_t instance)
{
    alt_eth_enable_disable_state_t bitstatus = ALT_ETH_DISABLE;
    
    if (instance > 2) { return bitstatus; }
    
    /* The Flow Control register should not be written to until this bit is cleared */
    if (ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_GET(alt_read_word(ALT_EMAC_GMAC_FLOW_CTL_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]))))
    {
        bitstatus = ALT_ETH_ENABLE;
    }
    else
    {
        bitstatus = ALT_ETH_DISABLE;
    }
    
    return bitstatus;
}

void alt_eth_mac_pause_ctrl_frame(uint32_t instance)  
{ 
    if (instance > 2) { return; }
    
    /* When Set In full duplex MAC initiates pause control frame */ 
    alt_setbits_word(ALT_EMAC_GMAC_FLOW_CTL_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                     ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_SET_MSK);

}

alt_eth_set_reset_state_t alt_eth_mac_get_mii_link_state(uint32_t instance)
{
    alt_eth_set_reset_state_t bitstatus = ALT_ETH_RESET;
    
    if (instance > 2) { return bitstatus; }    
    
    if (ALT_EMAC_GMAC_MII_CTL_STAT_LNKSTS_GET(alt_read_word(ALT_EMAC_GMAC_MII_CTL_STAT_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]))))
    {
        bitstatus = ALT_ETH_SET;
    }
    else
    {
        bitstatus = ALT_ETH_RESET;
    }
    
    return bitstatus;
}

void alt_eth_mac_set_bpa_state(alt_eth_enable_disable_state_t new_state, uint32_t instance)   
{  
    if (instance > 2) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Activate the MAC BackPressure operation */
        /* In Half duplex: during backpressure, when the MAC receives a new frame,
        the transmitter starts sending a JAM pattern resulting in a collision */
        alt_setbits_word(ALT_EMAC_GMAC_FLOW_CTL_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_SET_MSK);
    
    }
    else
    {
        /* Deactivate the MAC BackPressure operation */ 
        alt_clrbits_word(ALT_EMAC_GMAC_FLOW_CTL_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         ALT_EMAC_GMAC_FLOW_CTL_FCA_BPA_SET_MSK);
    
    } 
}

alt_eth_set_reset_state_t alt_eth_mac_check_status_reg(uint32_t mac_bit_mask, uint32_t instance)
{
    alt_eth_set_reset_state_t bitstatus = ALT_ETH_RESET;
    
    if (instance > 2) { return bitstatus; }
    
    if (alt_read_word(ALT_EMAC_GMAC_INT_STAT_ADDR(Alt_Emac_Gmac_Grp_Addr[instance])) & mac_bit_mask)
    {
        bitstatus = ALT_ETH_SET;
    }
    else
    {
        bitstatus = ALT_ETH_RESET;
    }
    
    return bitstatus;
}

uint32_t alt_eth_mac_get_irq_status_reg(uint32_t instance)
{
    if (instance > 2) { return 0; }
    
    return alt_read_word(ALT_EMAC_GMAC_INT_STAT_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]));
}

void alt_eth_mac_set_irq_reg(uint32_t mac_irq_mask, alt_eth_enable_disable_state_t new_state, uint32_t instance)
{
    if (instance > 2) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Enable the selected ETHERNET MAC interrupts */
        alt_clrbits_word(ALT_EMAC_GMAC_INT_MSK_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         mac_irq_mask);
    }
    else
    {
        /* Disable the selected ETHERNET MAC interrupts */
        alt_setbits_word(ALT_EMAC_GMAC_INT_MSK_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                         mac_irq_mask);
    }
}
void alt_eth_mac_set_sa_filter(uint8_t *address, uint32_t instance){

}

void alt_eth_mac_set_da_filter(uint8_t *address, uint32_t instance){

}


void alt_eth_mac_set_mac_addr(uint8_t *address, uint32_t instance)
{
    uint32_t tmpreg;
    
    if (instance > 2) { return; }    
      
    /* Calculate the selected MAC address high register */
    tmpreg = ((uint32_t)address[5] << 8) | (uint32_t)address[4];
    
    /* Load the selected MAC address high register */
    alt_write_word(ALT_EMAC_GMAC_MAC_ADDR0_HIGH_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), 
                   tmpreg);
    
    /* Calculate the selected MAC address low register */
    tmpreg = ((uint32_t)address[3] << 24) | ((uint32_t)address[2] << 16) | 
             ((uint32_t)address[1] << 8) | address[0];
                
     /* Load the selected MAC address low register */
    alt_write_word(ALT_EMAC_GMAC_MAC_ADDR0_LOW_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]), tmpreg);
}

void alt_eth_mac_get_mac_addr(uint8_t *address, uint32_t instance)
{
    uint32_t tmpreg;
    
    if (instance > 2) { return; }    
      
    /* Get the selected MAC address high register */
    tmpreg = alt_read_word(ALT_EMAC_GMAC_MAC_ADDR0_HIGH_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]));
    
     /* Calculate the selected MAC address buffer */
    address[5] = ((tmpreg >> 8) & (uint8_t)0xFF);
    address[4] = (tmpreg & (uint8_t)0xFF);
     
    /* Load the selected MAC address low register */
    tmpreg = alt_read_word(ALT_EMAC_GMAC_MAC_ADDR0_LOW_ADDR(Alt_Emac_Gmac_Grp_Addr[instance]));
    
    /* Calculate the selected MAC address buffer */
    address[3] = ((tmpreg >> 24) & (uint8_t)0xFF);
    address[2] = ((tmpreg >> 16) & (uint8_t)0xFF);
    address[1] = ((tmpreg >> 8 ) & (uint8_t)0xFF);
    address[0] = (tmpreg & (uint8_t)0xFF);
}

alt_eth_set_reset_state_t alt_eth_get_software_reset_status(uint32_t instance)
{
    alt_eth_set_reset_state_t bitstatus = ALT_ETH_RESET;
    
    if (instance > 2) { return bitstatus; }    
    
    if(ALT_EMAC_DMA_BUS_MOD_SWR_GET(alt_read_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]))))
    {
        bitstatus = ALT_ETH_SET;
    }
    else
    {
        bitstatus = ALT_ETH_RESET;
    }
    
    return bitstatus;
}

ALT_STATUS_CODE alt_eth_software_reset(uint32_t instance)
{
    unsigned int i;
    uint32_t val;
    uint32_t addr;
    
    if (instance > 1) { return ALT_E_ERROR; }    
    
    /* Set the SWR bit: resets all MAC subsystem internal registers and logic */
    /* After reset all the registers holds their respective reset values */
    addr = (uint32_t)ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]);
    val=alt_read_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]));
    dprintf("DBG: Addr: 0x%08x,DMA_MOD_REG=%u\r\n",addr,val);

    alt_setbits_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                     ALT_EMAC_DMA_BUS_MOD_SWR_SET_MSK);

    val=alt_read_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]));
    dprintf("DBG: Addr: 0x%08x,DMA_MOD_REG=%u\r\n",addr,val);

    
                        
    /* Wait for the software reset to clear */
    for (i = 0; i < 10; i++)
    {
        alt_eth_delay(ETH_RESET_DELAY);
        if (alt_eth_get_software_reset_status(instance) == ALT_ETH_RESET)
        {
            dprintf("Wait for software reset to clear %d times.", i);
            break;
        }
        dprintf("Wait for software reset to clear %d times.\r\n", i);


    }

    val=alt_read_word(ALT_EMAC_DMA_BUS_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]));
    dprintf("DBG: DMA_MOD_REG=%u\r\n",val);
    
    if (i==10) { return ALT_E_ERROR; }
    
    return ALT_E_SUCCESS;
}

/******************************************************************************/                             
/*                           DMA and Descriptors functions                    */
/******************************************************************************/

uint32_t alt_eth_dma_get_rx_desc_frame_len(alt_eth_dma_desc_t *rx_dma_desc)
{
    /* Return the Receive desc frame length */
    return ((rx_dma_desc->status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAME_LENGTHSHIFT);
}

uint32_t alt_eth_dma_get_status_reg(uint32_t instance)
{  

    if (instance > 2) { return 0; }
    
    return alt_read_word(ALT_EMAC_DMA_STAT_ADDR(Alt_Emac_Dma_Grp_Addr[instance]));
}

alt_eth_set_reset_state_t alt_eth_dma_check_status_reg(uint32_t dma_bit_mask, uint32_t instance)
{  
    alt_eth_set_reset_state_t bitstatus = ALT_ETH_RESET;
    
    if (instance > 2) { return bitstatus; }    
    
    if (alt_read_word(ALT_EMAC_DMA_STAT_ADDR(Alt_Emac_Dma_Grp_Addr[instance])) & dma_bit_mask)
    {
        bitstatus = ALT_ETH_SET;
    }
    else
    {
        bitstatus = ALT_ETH_RESET;
    }
    
    return bitstatus;
}

void alt_eth_dma_clear_status_bits(uint32_t dma_bit_mask, uint32_t instance)
{
    if (instance > 1) { return; }
    
    /* Clear the selected ETHERNET DMA bit(s) */
    alt_write_word(ALT_EMAC_DMA_STAT_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), dma_bit_mask);
} 

void alt_eth_dma_set_irq_reg(uint32_t dma_irq_mask, alt_eth_enable_disable_state_t new_state, uint32_t instance)
{
    if (instance > 1) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Enable the selected ETHERNET DMA interrupts */
        alt_setbits_word(ALT_EMAC_DMA_INT_EN_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                         dma_irq_mask);
    }
    else
    {
        /* Disable the selected ETHERNET DMA interrupts */
        alt_clrbits_word(ALT_EMAC_DMA_INT_EN_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                         dma_irq_mask);
    }
}

void alt_eth_dma_flush_tx_fifo(uint32_t instance)
{
    if (instance > 2) { return; }
    
    /* Set the Flush Transmit FIFO bit */ 
    alt_setbits_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), ALT_EMAC_DMA_OP_MOD_FTF_SET_MSK);
}

alt_eth_set_reset_state_t alt_eth_dma_get_tx_fifo_flush_state(uint32_t instance)
{   
    alt_eth_set_reset_state_t bitstatus = ALT_ETH_RESET;
    
    if (instance > 2) { return bitstatus; }    
    
    if (ALT_EMAC_DMA_OP_MOD_FTF_GET(alt_read_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]))))
    {
        bitstatus = ALT_ETH_SET;
    }
    else
    {
        bitstatus = ALT_ETH_RESET;
    }
    
    return bitstatus; 
}

void alt_eth_dma_set_tx_state(alt_eth_enable_disable_state_t new_state, uint32_t instance)
{
    if (instance > 2) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Enable the DMA transmission */
        alt_setbits_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                         ALT_EMAC_DMA_OP_MOD_ST_SET_MSK);
    
    }
    else
    {
        /* Disable the DMA transmission */
        alt_clrbits_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                         ALT_EMAC_DMA_OP_MOD_ST_SET_MSK);
    
    }
}

void alt_eth_dma_set_rx_state(alt_eth_enable_disable_state_t new_state, uint32_t instance)
{ 
    if (instance > 2) { return; }
    
    if (new_state != ALT_ETH_DISABLE)
    {
        /* Enable the DMA reception */
        alt_setbits_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                       ALT_EMAC_DMA_OP_MOD_SR_SET_MSK);
    
    }
    else
    {
        /* Disable the DMA reception */
        alt_clrbits_word(ALT_EMAC_DMA_OP_MOD_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 
                       ALT_EMAC_DMA_OP_MOD_SR_SET_MSK);
    
    }
}

alt_eth_set_reset_state_t alt_eth_dma_check_overflow_counter_reg(uint32_t dma_overflow_mask, uint32_t instance)
{
    alt_eth_set_reset_state_t bitstatus = ALT_ETH_RESET;
    
    if (instance > 2) { return bitstatus; }    
       
    if (alt_read_word(ALT_EMAC_DMA_MFRM_BUF_OVF_CNTR_ADDR(Alt_Emac_Dma_Grp_Addr[instance])) & dma_overflow_mask)
    {
        bitstatus = ALT_ETH_SET;
    }
    else
    {
        bitstatus = ALT_ETH_RESET;
    }
    
    return bitstatus;
}

uint32_t alt_eth_dma_get_curr_tx_desc_addr(uint32_t instance)
{
    if (instance > 2) { return 0; }
    
    return ((uint32_t)alt_read_word(ALT_EMAC_DMA_CUR_HOST_TX_DESC_ADDR(Alt_Emac_Dma_Grp_Addr[instance])));
}

uint32_t alt_eth_dma_get_curr_rx_desc_addr(uint32_t instance)
{
    if (instance > 2) { return 0; }
    
    return ((uint32_t)alt_read_word(ALT_EMAC_DMA_CUR_HOST_RX_DESC_ADDR(Alt_Emac_Dma_Grp_Addr[instance])));
}

uint32_t alt_eth_dma_get_curr_tx_buff_addr(uint32_t instance)
{
    if (instance > 2) { return 0; }
    
    return ((uint32_t)alt_read_word(ALT_EMAC_DMA_CUR_HOST_TX_BUF_ADDR_ADDR(Alt_Emac_Dma_Grp_Addr[instance])));
}

uint32_t alt_eth_dma_get_curr_rx_buff_addr(uint32_t instance)
{
    if (instance > 2) { return 0; }
    
    return ((uint32_t)alt_read_word( ALT_EMAC_DMA_CUR_HOST_RX_BUF_ADDR_ADDR(Alt_Emac_Dma_Grp_Addr[instance])));
}

void alt_eth_dma_set_tx_desc_addr(uint32_t tx_desc_list_addr, uint32_t instance)
{
    if (instance > 2) { return; }
    
    alt_write_word(ALT_EMAC_DMA_TX_DESC_LIST_ADDR_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), tx_desc_list_addr);
}

void alt_eth_dma_set_rx_desc_addr(uint32_t rx_desc_list_addr, uint32_t instance)
{
    if (instance > 2) { return; }
    
    alt_write_word(ALT_EMAC_DMA_RX_DESC_LIST_ADDR_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), rx_desc_list_addr);
    uint32_t dbg_addr, dbg_data;

    dbg_addr = (uint32_t) ((ALT_EMAC_DMA_RX_DESC_LIST_ADDR_ADDR(Alt_Emac_Dma_Grp_Addr[instance])));
    dbg_data = alt_read_word(ALT_EMAC_DMA_RX_DESC_LIST_ADDR_ADDR(Alt_Emac_Dma_Grp_Addr[instance]));
    dprintf("DBG: Addr: 0x%08x,Data=0x%08x\r\n",dbg_addr,dbg_data);
}

void alt_eth_dma_resume_dma_tx(uint32_t instance)
{
    if (instance > 2) { return; }
    
    alt_write_word(ALT_EMAC_DMA_TX_POLL_DEMAND_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 0);
}

void alt_eth_dma_resume_dma_rx(uint32_t instance)
{
    if (instance > 2) { return; }
    
    alt_write_word(ALT_EMAC_DMA_RX_POLL_DEMAND_ADDR(Alt_Emac_Dma_Grp_Addr[instance]), 0);
}

ALT_STATUS_CODE alt_eth_send_packet(uint8_t * pkt, uint32_t len, uint32_t first, uint32_t last, alt_eth_emac_instance_t * emac)
{
    
    alt_eth_dma_desc_t *tx_desc;
    dprintf("DBG: emac addr=%p\n",emac);
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    
    //uint32_t * txbuf;
    int32_t index=0;
    unsigned int i;
    int32_t paranoid=NUMBER_OF_TX_DESCRIPTORS+1;
    //printf("DBG: Start to execute send_packet!\r\n");
    
    tx_desc = &emac->tx_desc_ring[emac->tx_current_desc_number];
    dprintf("DBG: tx_desc=%p\n",tx_desc);
    uint32_t* tx_desc_word = (uint32_t*) tx_desc;

    uint32_t *p = (uint32_t *)emac;
    uint32_t n = sizeof(*emac) / sizeof(uint32_t);
    uint32_t pa;


    //printf("DBG: pointer checked,introduce some delay to avoid hang!\r\n"); //adding this printf between tx_desc and if(tx_desc->status) can resolve hang issue.
    dprintf("DBG: pointer checked! check addresses \r\n");
    //alt_eth_delay(128); //adding this delay between tx_desc and if(tx_desc->status) can resolve hang issue.
    /* Check if it is a free descriptor.  */
    if (tx_desc->status & ETH_DMATXDESC_OWN) 
    {
        /* Buffer is still owned by device.  */
        dprintf("No free tx descriptors!\n");
        return(ALT_E_ERROR);
    } 

    
    /* check if len is too large */
    if (len > ETH_BUFFER_SIZE)
    {
        return(ALT_E_ERROR);
    }
    
    /* Copy data to local buffer   */
    for (i = 0; i < len; i++)
    {
        *(uint8_t *)(emac->tx_buf + (emac->tx_current_desc_number * ETH_BUFFER_SIZE) + i) = *(pkt + i);
    }
    alt_eth_delay(128);
    //printf("DBG: Data copied over!\r\n");

    //txbuf = (uint32_t*)&emac->tx_buf;
    //for (i = 0; i < 32; i++) {
    //    printf("tx_buf content: DBG[%d]: 0x%08x\r\n", i, txbuf[i]);
    //}


   
       // void * vaddr  = (void *)((uintptr_t)(pgm->program + pgm->buffer_start) & ~(ALT_CACHE_LINE_SIZE - 1));
       // void * vend   = (void *)(((uintptr_t)(pgm->program + pgm->buffer_start + pgm->code_size) + (ALT_CACHE_LINE_SIZE - 1)) & ~(ALT_CACHE_LINE_SIZE - 1));
       // size_t length = (uintptr_t)vend - (uintptr_t)vaddr;

      //status = alt_cache_system_clean(vaddr, length);



    
    /* set the buffer pointer */
    tx_desc->buffer1_addr = (uint32_t)&emac->tx_buf[emac->tx_current_desc_number * ETH_BUFFER_SIZE];
    dprintf("DBG: tx_desc->buffer1_addr=%p\n",tx_desc->buffer1_addr);
    
    /* Set the buffer size.  */
    tx_desc->control_buffer_size = (len & ETH_DMATXDESC_TBS1);
    
#ifdef USE_ENHANCED_DMA_DESCRIPTORS
    tx_desc->status =   ETH_DMATXDESC_TCH;       
     
    /* Set the Descriptor's FS bit.  */
    if (first) { tx_desc->status |=  (ETH_DMATXDESC_FS | ETH_DMATXDESC_CIC_TCPUDPICMP_FULL); }
    
    /* set the Descriptor's LS and IC bit.  */
    if (last)  { tx_desc->status |=  ( ETH_DMATXDESC_LS | ETH_DMATXDESC_IC); }
#else    
    tx_desc->control_buffer_size |=   ETH_DMATXDESC_TCH;     
    
    /* Set the Descriptor's FS bit.  */
    if (first) { tx_desc->control_buffer_size |=  (ETH_DMATXDESC_FS | ETH_DMATXDESC_CIC_TCPUDPICMP_FULL); }
    
    /* set the Descriptor's LS and IC bit.  */
    if (last)  { tx_desc->control_buffer_size |=  ( ETH_DMATXDESC_LS | ETH_DMATXDESC_IC); }
#endif 

    if (last)  { index = emac->tx_current_desc_number; }

    /* Set the current index to the next descriptor.  */
    emac->tx_current_desc_number = (emac->tx_current_desc_number + 1);
    if (emac->tx_current_desc_number >= NUMBER_OF_TX_DESCRIPTORS) { emac->tx_current_desc_number=0; }  

    for (i = 0; i < 4; i++) {
        dprintf("tx_desc content: DBG[%d]: 0x%08x\r\n", i, tx_desc_word[i]);
    }

    
    /* if this is the last descriptor, set the chain's owned bits to owned by hardware */
    if (last)
    {
       // printf("DBG: Last situation over!\r\n");
        /* paranoid will never get to 0.  Its just here for non human paranoid error checkers */
        while (paranoid--)  
        {
            dprintf("DBG: Within the loop!\r\n");
            tx_desc = &emac->tx_desc_ring[index];
            for (i = 0; i < 4; i++) {
                dprintf("tx_desc content before set OWN: DBG[%d]: 0x%08x\r\n", i, tx_desc_word[i]);
            }

            
            if (tx_desc->status & ETH_DMATXDESC_OWN)
            {
                /* tx buffer error, re-initialize and return error*/
                dprintf("Send packet error!\n");                
                alt_eth_dma_set_tx_state(ALT_ETH_DISABLE, emac->instance); 
                alt_eth_dma_flush_tx_fifo(emac->instance);
                alt_eth_setup_txdesc(emac);
                alt_eth_dma_set_tx_state(ALT_ETH_ENABLE, emac->instance); 
                return ALT_E_ERROR;
            }
         
            /* Set OWN bit.  */
            tx_desc->status |= ETH_DMATXDESC_OWN;
            
            for (i = 0; i < 4; i++) {
                dprintf("tx_desc content after set OWN: DBG[%d]: 0x%08x\r\n", i, tx_desc_word[i]);
            }
            

#ifdef USE_ENHANCED_DMA_DESCRIPTORS            
            if (tx_desc->status & ETH_DMATXDESC_FS) { break; }
#else
            if (tx_desc->control_buffer_size & ETH_DMATXDESC_FS) { break; }
#endif            
            index--;
            if (index < 0) { index=NUMBER_OF_TX_DESCRIPTORS-1; }
        } 
       
        /* If the DMA transmission is suspended, resume transmission.  */
        if (alt_eth_dma_check_status_reg(ALT_EMAC_DMA_STAT_TS_SET_MSK,emac->instance))
        {         
            printf("Are we trnsmitting!\n");     
            /* Clear TBUS ETHERNET DMA flag */
            alt_eth_dma_clear_status_bits(ALT_EMAC_DMA_STAT_TS_SET_MSK,emac->instance);
            
            /* Resume DMA transmission */
            alt_eth_dma_resume_dma_tx(emac->instance);
        }
        
       // for (i = 0; i < n; i++) {
       //     pa = (uint32_t)p;
       //     printf("PRE_DBG[%u]: pa=0x%08x, data=0x%08x\n", i, pa, *p++);
       // }

        void* dma_buf = (void*) ((uintptr_t)emac & ~(ALT_CACHE_LINE_SIZE - 1));// emac->tx_buf + (emac->tx_current_desc_number * ETH_BUFFER_SIZE);
        size_t alen = (n + ALT_CACHE_LINE_SIZE - 1) & ~(ALT_CACHE_LINE_SIZE - 1);


        ////dump_ddr(dma_buf,alen);

        status = alt_cache_system_clean(dma_buf, alen);

        ////dump_ddr(dma_buf,alen);


        if (status != ALT_E_SUCCESS) {
            ALT_PRINTF("ERROR: CACHE SYNC failed, %" PRIi32 ".\n", status);
        } 
        
        alt_eth_start(emac->instance);
        
        p = (uint32_t *)emac;
        for (i = 0; i < n/n; i++) {
            pa = (uint32_t)p;
            printf("POST_DBG[%u]: pa=0x%08x, data=0x%08x\n", i, pa, *p++);
        }        

    }    

    return ALT_E_SUCCESS;
}
                
ALT_STATUS_CODE alt_eth_get_packet(uint8_t * pkt, uint32_t * len, alt_eth_emac_instance_t * emac)
{
    printf("Get packet dbg: buffer=0x%08x, len_buf=0x%08x\n", pkt, len);
    static int numrxpackets=0;
    alt_eth_dma_desc_t * desc;
    uint32_t size=0,rx_search_desc_number,packet_end=0,packet_start=0,wrap,i;
    
    if (emac->instance > 1) { return ALT_E_ERROR; }    
    
    /* If the DMA reception is suspended, resume transmission.  */
#ifdef ALT_RX_RESUME    
    if (alt_eth_dma_check_status_reg(ALT_EMAC_DMA_STAT_RS_SET_MSK,emac->instance))
    {
        /* Clear TBUS ETHERNET DMA flag */
        alt_eth_dma_clear_status_bits(ALT_EMAC_DMA_STAT_RS_SET_MSK,emac->instance);

        /* Resume DMA transmission */
        alt_eth_dma_resume_dma_rx(emac->instance);
    }
#endif    
 
    rx_search_desc_number=emac->rx_processed_desc_number;
    wrap=rx_search_desc_number;
    desc=&emac->rx_desc_ring[rx_search_desc_number];
    while ((desc->status & ETH_DMARXDESC_OWN)==0)
    {
        if (desc->status & ETH_DMARXDESC_FS)  {  packet_start++; }
        
        if (desc->status & ETH_DMARXDESC_LS)
        { 
            packet_end=1;
            break;
        };
        
        rx_search_desc_number++;
        if (rx_search_desc_number == NUMBER_OF_RX_DESCRIPTORS) { rx_search_desc_number = 0; }
        desc=&emac->rx_desc_ring[rx_search_desc_number];
        if (rx_search_desc_number==wrap) {
           printf("Error: Rx buffer wrap, End of Packet not found.\n");
           alt_eth_reinit_rxdesc(emac);
           return ALT_E_ERROR;
        }
    }
    if (packet_end==0) { 
        /* no packet available */
        return ALT_E_ERROR; 
    }  
    
    if (packet_start != 1)
    {
        printf("Rx Packet Error, %d Packet Starts found\n",packet_start);
        alt_eth_reinit_rxdesc(emac);
        return ALT_E_ERROR;
    }
    
    if (alt_eth_dma_check_status_reg(ALT_EMAC_DMA_STAT_RU_SET_MSK, emac->instance))
    {      
        printf("Rx Unavailable Error\n");
        alt_eth_dma_clear_status_bits(ALT_EMAC_DMA_STAT_RU_SET_MSK, emac->instance); 
        alt_eth_reinit_rxdesc(emac);
        return ALT_E_ERROR;
    } 
    
    if (alt_eth_dma_check_status_reg(ALT_EMAC_DMA_STAT_OVF_SET_MSK, emac->instance))
    {
        printf("Rx OverFlow Error\n");
        alt_eth_reinit_rxdesc(emac);
        alt_eth_dma_clear_status_bits(ALT_EMAC_DMA_STAT_OVF_SET_MSK, emac->instance);
        return ALT_E_ERROR;
    }     
        
    /* A complete packet is available, process it */   
    desc=&emac->rx_desc_ring[emac->rx_processed_desc_number];   
    while ((desc->status & ETH_DMARXDESC_OWN)==0)
    {
        emac->rx_processed_desc_number++;
        if (emac->rx_processed_desc_number == NUMBER_OF_RX_DESCRIPTORS) { emac->rx_processed_desc_number = 0; }
        
        if (desc->status & ETH_DMARXDESC_LS)
        { 
            size=((desc->status >> ETH_DMARXDESC_FRAME_LENGTHSHIFT) & ETH_DMARXDESC_RBS1);  
            for (i = 0; i < size; i++)
            {
                *(pkt + i) = *(uint8_t *)((uintptr_t)desc->buffer1_addr + i);
            }
            desc->status = ETH_DMARXDESC_OWN;            
            break;
        };
        
        desc->status = ETH_DMARXDESC_OWN;        
        desc=&emac->rx_desc_ring[emac->rx_processed_desc_number];
    }
    
    *len = size;
    
    numrxpackets++;
    if ((numrxpackets & 0x1fff)==0) 
    {
#ifdef ALT_DEBUG_ETHERNET
        //dprintf("numrxpkts=%d numrxints=%d numtxints=%d missed=%d ovmiss=%d\n",
        //  numrxpackets,emac->rxints,emac->txints,
          //(uint32_t)alt_read_word(ALTX_EMAC_DMAGRP_MISSED_FRAME_AND_BUFFER_OVERFLOW_COUNTER_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance])) & 0xffff,
          //((uint32_t)alt_read_word(ALTX_EMAC_DMAGRP_MISSED_FRAME_AND_BUFFER_OVERFLOW_COUNTER_ADDR(Alt_Emac_Dma_Grp_Addr[emac->instance])) >> 17) & 0x7ff);
#endif          
    }
              
    alt_eth_mac_check_mii_link_status(emac->instance);
       
    return ALT_E_SUCCESS;   
}

void alt_eth_mac_check_mii_link_status(uint32_t instance)
{
    uint32_t status;
    
    /* Read the MAC Status to determine if the link changed */
    status = alt_eth_mac_get_irq_status_reg(instance);
    
    /* Interrupt on PHY status change.  */
    if(status & ALT_EMAC_GMAC_INT_STAT_RGSMIIIS_SET_MSK)
    {
        dprintf("PHY STATUS CHANGE\n");
        
        /* If link is up, we need to restart the negotiation process */
        if (alt_eth_mac_get_mii_link_state(instance))
        {
            /* may need to implement this later */
            dprintf("Need to regonotiate\n");
        }
    }
}

void ethernet_raw_frame_gen(uint32_t len, uint8_t* dst_addr_arr, uint8_t* arr) {
    uint32_t i;
    uint32_t pay_load_cnt = len - 14;
    //pass dst addr
    uint8_t src_addr[6] = {0x00, 0x07, 0xed, 0x42, 0x9a, 0x48};
    uint8_t* p = src_addr;
    for(i=0;i<6;i++) {
        *arr++ = *dst_addr_arr++;
    }
    //pass src addr
    for(i=0;i<6;i++) {
        *arr++ = *p++;
    }
    //pass type
    *arr++ = 0x88;
    *arr++ = 0xB5;
    for(i=0;i<pay_load_cnt;i++) {
        *arr++ = i;
    }
}






ALT_STATUS_CODE scatter_frame(uint8_t * pkt, uint32_t len, alt_eth_emac_instance_t * emac) {
//this function calls alt_eth_send_packet, based upon the size, assign arguments correctly.
//for instance, if buffer size is 128, then a frame of 200 size has to be sent calling this function
//twice, first half sets first and second half sets last.
    if(len==0) {
        return ALT_E_ERROR;
    }
    ALT_STATUS_CODE status = ALT_E_SUCCESS;
    uint32_t num=len/ETH_BUFFER_SIZE;
    uint32_t rem=len%ETH_BUFFER_SIZE;
    uint32_t i, ind_begin;
    if(rem) { //test frame is not a multiple of individual buffer
        num++; //at least 2
        for(i=0;i<num;i++) {
            ind_begin = i*ETH_BUFFER_SIZE;
            if(num==1) {
                alt_eth_send_packet(pkt+ind_begin,rem,1,1,emac);
            } else {
                if(i==0) { //first 
                    alt_eth_send_packet(pkt+ind_begin,ETH_BUFFER_SIZE,1,0,emac);
                } else if(i==num-1) { //last
                    alt_eth_send_packet(pkt+ind_begin,rem,0,1,emac);
                } else {
                    alt_eth_send_packet(pkt+ind_begin,ETH_BUFFER_SIZE,0,0,emac);
                } 
            }
        }
    } else { // test frame is a multiple of individual buffer
        for(i=0;i<num;i++) {
            ind_begin = i*ETH_BUFFER_SIZE;
            if(num==1) {
                alt_eth_send_packet(pkt+ind_begin,ETH_BUFFER_SIZE,1,1,emac);
            } else {
                if(i==0) { //first 
                    alt_eth_send_packet(pkt+ind_begin,ETH_BUFFER_SIZE,1,0,emac);
                } else if(i==num-1) { //last
                    alt_eth_send_packet(pkt+ind_begin,ETH_BUFFER_SIZE,0,1,emac);
                } else {
                    alt_eth_send_packet(pkt+ind_begin,ETH_BUFFER_SIZE,0,0,emac);
                }
            }
        }
    }

    return status;

}




