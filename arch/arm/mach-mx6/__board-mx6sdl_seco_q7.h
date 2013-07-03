
//#ifndef _BOARD_MX6_SECO_Q7_H
//#define _BOARD_MX6_SECO_Q7_H

#include <mach/iomux-mx6dl.h>


static iomux_v3_cfg_t mx6sdl_seco_q7_pads[] = {
	
	MX6DL_PAD_SD2_CMD__GPIO_1_11,	 			//	for audio reset

        /* SD2 */
	//MX6DL_PAD_SD2_CLK__USDHC2_CLK,	 			//	for SD signal to logic
	//MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,  			//  for SD signal to logic
	//MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,  			//  for SD signal to logic
	//MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,  			//  for SD signal to logic
	//MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,  			//  for SD signal to logic
        
        /* AUDMUX */
	MX6DL_PAD_DI0_PIN2__AUDMUX_AUD6_TXD,  		//  for AUDIO AUDMUX port 6
	MX6DL_PAD_DI0_PIN3__AUDMUX_AUD6_TXFS, 		//  for AUDIO AUDMUX port 6
	MX6DL_PAD_DI0_PIN4__AUDMUX_AUD6_RXD,  		//  for AUDIO AUDMUX port 6
	MX6DL_PAD_DI0_PIN15__AUDMUX_AUD6_TXC, 		//  for AUDIO AUDMUX port 6

        /* CAN1  */
	MX6DL_PAD_KEY_ROW2__CAN1_RXCAN,				//  for can bus
	MX6DL_PAD_KEY_COL2__CAN1_TXCAN,				//  for can bus
        
		/* UART1 */
	MX6DL_PAD_SD3_DAT6__UART1_RXD,				//	OPTIONAL DUART_RXD - ATTENTION configurable as MX6DL_PAD_SD3_DAT6__USDHC3_DAT6_50MHZ
	MX6DL_PAD_SD3_DAT7__UART1_TXD,				//	OPTIONAL DUART_TXD - ATTENTION configurable as MX6DL_PAD_SD3_DAT7__USDHC3_DAT7_50MHZ
	
        /* UART2 for debug */
	MX6DL_PAD_EIM_D26__UART2_TXD,				//	DUART_TXD - for UART2
	MX6DL_PAD_EIM_D27__UART2_RXD,				//	DUART_RXD - for UART2

		/* UART4 */
	MX6DL_PAD_KEY_ROW0__UART4_RXD,				//  for UART4
	MX6DL_PAD_KEY_COL0__UART4_TXD,				//  for UART4
	
		/* UART5 */
	MX6DL_PAD_KEY_COL1__UART5_TXD,   			//  for UART5
	MX6DL_PAD_KEY_ROW1__UART5_RXD,				//  for UART5
	MX6DL_PAD_KEY_COL4__UART5_RTS,   			//  for UART5
	MX6DL_PAD_KEY_ROW4__UART5_CTS,				//  for UART5
        
        
	MX6DL_PAD_EIM_D20__GPIO_3_20,

	/*SDIO INTERFACE*/

		/* USDHC1 */
	MX6DL_PAD_SD1_CMD__USDHC1_CMD,				//	SD1_CMD  - for SD on board
	MX6DL_PAD_SD1_CLK__USDHC1_CLK,				//	SD1_CLK  - for SD on board
	MX6DL_PAD_SD1_DAT0__USDHC1_DAT0,				//	SD1_DAT0 - for SD on board
	MX6DL_PAD_SD1_DAT1__USDHC1_DAT1,				//	SD1_DAT1 - for SD on board
	MX6DL_PAD_SD1_DAT2__USDHC1_DAT2,				//	SD1_DAT2 - for SD on board
	MX6DL_PAD_SD1_DAT3__USDHC1_DAT3,				//	SD1_DAT3 - for SD on board
	MX6DL_PAD_NANDF_CS2__GPIO_6_15,				//	SD1_CD   - for SD on board
	//MX6DL_PAD_GPIO_19__GPIO_4_5, 				//	SD1_WP   - for SD on board
	MX6DL_PAD_CSI0_DATA_EN__GPIO_5_20, 				//	SD1_WP   - for SD on board
#ifdef CONFIG_MMC1_8bit
	MX6DL_PAD_NANDF_D0__USDHC1_DAT4,				//	SD1_DAT4 - for SD on board
	MX6DL_PAD_NANDF_D1__USDHC1_DAT5,				//	SD1_DAT5 - for SD on board
	MX6DL_PAD_NANDF_D2__USDHC1_DAT6,				//	SD1_DAT6 - for SD on board
	MX6DL_PAD_NANDF_D3__USDHC1_DAT7,				//	SD1_DAT7 - for SD on board
#endif


		/* USDHC3 -> eMMC ONBOARD */
	MX6DL_PAD_SD3_CLK__USDHC3_CLK_50MHZ,			//	eMMC_CMD   
	MX6DL_PAD_SD3_CMD__USDHC3_CMD_50MHZ,			//	eMMC_CLK
	MX6DL_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,		//	eMMC_DAT0
	MX6DL_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,		//	eMMC_DAT1
	MX6DL_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,		//	eMMC_DAT2
	MX6DL_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,		//	eMMC_DAT3
	MX6DL_PAD_SD3_RST__USDHC3_RST,				//	eMMC_RESETN
	
		/* USDHC4 */
	MX6DL_PAD_SD4_CLK__USDHC4_CLK_50MHZ,			//	uSD_CLK
	MX6DL_PAD_SD4_CMD__USDHC4_CMD_50MHZ,			//	uSD_CMD
	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,		//	uSD_DAT0
	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,		//	uSD_DAT1
	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,		//	uSD_DAT2
	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,		//	uSD_DAT3
	MX6DL_PAD_NANDF_D6__GPIO_2_6,				//	uSD_CD
	MX6DL_PAD_NANDF_D7__GPIO_2_7,				//	uSD_WP

	MX6DL_PAD_NANDF_D4__GPIO_2_4,				//  gpio generic
	MX6DL_PAD_NANDF_D5__GPIO_2_5,				//  gpio generic
	MX6DL_PAD_SD4_DAT5__GPIO_2_13,				//  gpio generic
	MX6DL_PAD_SD4_DAT6__GPIO_2_14,				//  gpio generic
	MX6DL_PAD_NANDF_WP_B__GPIO_6_9,				//  gpio generic
	MX6DL_PAD_GPIO_18__GPIO_7_13,				//  gpio generic
	MX6DL_PAD_GPIO_2__GPIO_1_2,					//  gpio generic
	MX6DL_PAD_GPIO_4__GPIO_1_4,					//  gpio generic
	MX6DL_PAD_GPIO_6__GPIO_1_6,					//  gpio generic
	MX6DL_PAD_GPIO_7__GPIO_1_7,					//  gpio generic
	MX6DL_PAD_GPIO_8__GPIO_1_8,					//  gpio generic

        /* CPLD CLOCK  */
	MX6DL_PAD_GPIO_0__CCM_CLKO,					// CPLD_GPIO_0_CLKO
        
        /* ECSPI1 */
	MX6DL_PAD_EIM_D17__ECSPI1_MISO,      		//	to SPI
	MX6DL_PAD_EIM_D18__ECSPI1_MOSI,      		//	to SPI
	MX6DL_PAD_EIM_D16__ECSPI1_SCLK,      		//	to SPI
	MX6DL_PAD_EIM_D19__GPIO_3_19,				//	to SPI  CS1
	MX6DL_PAD_EIM_D24__GPIO_3_24,				//  to SPI  CS2
	MX6DL_PAD_EIM_D25__GPIO_3_25,				//  to SPI  CS3
        
        /* ENET */
	MX6DL_PAD_ENET_MDIO__ENET_MDIO,				//  to ENET
	MX6DL_PAD_ENET_MDC__ENET_MDC,				//  to ENET
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC,			//  to ENET
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0,			//  to ENET
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1,			//  to ENET
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2,			//  to ENET
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3,			//  to ENET
	MX6DL_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,	//  to ENET
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK,			// 	to ENET
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC,			// 	to ENET
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0,			//  to ENET
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1,			//  to ENET
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2,			//  to ENET
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3,			//  to ENET
	MX6DL_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,	//  to ENET
	MX6DL_PAD_ENET_TX_EN__GPIO_1_28,				//  Micrel RGMII Phy Interrupt - to ENET
	MX6DL_PAD_EIM_D23__GPIO_3_23,				//  RGMII reset
        
		/* SATA */
	//MX6DL_PAD_NANDF_CS0__GPIO_6_11,				//  for SATA led activity

        /* WEIM BUS */      
	MX6DL_PAD_EIM_CS0__WEIM_WEIM_CS_0,			//  for CPLD logic
	MX6DL_PAD_EIM_CS1__WEIM_WEIM_CS_1,			//  for CPLD logic

    MX6DL_PAD_EIM_EB0__WEIM_WEIM_EB_0,			//  for CPLD logic
	MX6DL_PAD_EIM_EB1__WEIM_WEIM_EB_1,			//  for CPLD logic
	MX6DL_PAD_EIM_EB3__WEIM_WEIM_EB_3,			//  for CPLD logic
      
    MX6DL_PAD_EIM_RW__WEIM_WEIM_RW,				//  for CPLD logic
	MX6DL_PAD_EIM_OE__WEIM_WEIM_OE,				//  for CPLD logic
	MX6DL_PAD_EIM_LBA__WEIM_WEIM_LBA,			//  for CPLD logic
       
    MX6DL_PAD_EIM_DA0__WEIM_WEIM_DA_A_0,			//  for CPLD logic
	MX6DL_PAD_EIM_DA1__WEIM_WEIM_DA_A_1,			//  for CPLD logic
	MX6DL_PAD_EIM_DA2__WEIM_WEIM_DA_A_2,			//  for CPLD logic
	MX6DL_PAD_EIM_DA3__WEIM_WEIM_DA_A_3,			//  for CPLD logic
	MX6DL_PAD_EIM_DA4__WEIM_WEIM_DA_A_4,			//  for CPLD logic
	MX6DL_PAD_EIM_DA5__WEIM_WEIM_DA_A_5,			//  for CPLD logic
	MX6DL_PAD_EIM_DA6__WEIM_WEIM_DA_A_6,			//  for CPLD logic
	MX6DL_PAD_EIM_DA7__WEIM_WEIM_DA_A_7,			//  for CPLD logic
	MX6DL_PAD_EIM_DA8__WEIM_WEIM_DA_A_8,			//  for CPLD logic
	MX6DL_PAD_EIM_DA9__WEIM_WEIM_DA_A_9,			//  for CPLD logic
	MX6DL_PAD_EIM_DA10__WEIM_WEIM_DA_A_10,		//  for CPLD logic
	MX6DL_PAD_EIM_DA11__WEIM_WEIM_DA_A_11,		//  for CPLD logic
	MX6DL_PAD_EIM_DA12__WEIM_WEIM_DA_A_12,		//  for CPLD logic
	MX6DL_PAD_EIM_DA13__WEIM_WEIM_DA_A_13,		//  for CPLD logic
	MX6DL_PAD_EIM_DA14__WEIM_WEIM_DA_A_14,		//  for CPLD logic
	MX6DL_PAD_EIM_DA15__WEIM_WEIM_DA_A_15,		//  for CPLD logic

	MX6DL_PAD_NANDF_CS1__GPIO_6_14,				//  for CPLD logic LPC_IRQ0
	MX6DL_PAD_NANDF_CS3__GPIO_6_16,				//  for CPLD logic LPC_IRQ1
	MX6DL_PAD_NANDF_ALE__GPIO_6_8,				//  for CPLD logic LPC_IRQ2
        
        /* GPIO2 */ 
	MX6DL_PAD_EIM_A22__GPIO_2_16,				//	for CPLD logic
	MX6DL_PAD_EIM_A21__GPIO_2_17,				//	for CPLD logic
	MX6DL_PAD_EIM_A20__GPIO_2_18,				//	for CPLD logic
	MX6DL_PAD_EIM_A19__GPIO_2_19,				//  for CPLD logic
	MX6DL_PAD_EIM_A18__GPIO_2_20,				//  for CPLD logic
	MX6DL_PAD_EIM_A17__GPIO_2_21,				//  for CPLD logic
	MX6DL_PAD_EIM_A16__GPIO_2_22,				//  for CPLD logic
	MX6DL_PAD_EIM_A23__GPIO_6_6,					//  for CPLD logic
	MX6DL_PAD_EIM_WAIT__GPIO_5_0,				//  for CPLD logic
	MX6DL_PAD_EIM_A24__GPIO_5_4,					//  for CPLD logic

	MX6DL_PAD_EIM_A25__GPIO_5_2, 				//  HDMI - check
        
        /* USBOTG ID pin */
	MX6DL_PAD_GPIO_1__USBOTG_ID,			   
        MX6DL_PAD_GPIO_17__GPIO_7_12,			//  USB Hub Reset
	MX6DL_PAD_EIM_D22__GPIO_3_22,				//  for OTG power enable

		/* USB OC pin */
	MX6DL_PAD_EIM_D30__USBOH3_USBH1_OC,			//  to USBH1_OC Over Current
        
		/* PWM1 */
    MX6DL_PAD_GPIO_9__PWM1_PWMO,       			// LVDS CTRL 
                
    /************ I2C BUS ************/
		/* I2C1 */
	MX6DL_PAD_EIM_D21__I2C1_SCL,				//	I2C1_SCL - SMB
	MX6DL_PAD_EIM_D28__I2C1_SDA,				//	I2C1_SDA - SMB
		/* I2C3 */
	MX6DL_PAD_GPIO_5__I2C3_SCL,					//	I2C3_SCL
	MX6DL_PAD_GPIO_16__I2C3_SDA,				// 	I2C3_SDA
};


static iomux_v3_cfg_t mx6dl_seco_q7_hdmi_ddc_pads[] = {
	MX6DL_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6DL_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6dl_seco_q7_i2c2_pads[] = {
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

#define MX6DL_USDHC_PAD_SETTING(id, speed)	\
mx6dl_sd##id##_##speed##mhz[] = {		\
	MX6DL_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6DL_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}


//#endif
