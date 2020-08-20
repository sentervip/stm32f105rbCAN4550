/*
 * main.c
 * Author: Texas Instruments
 * Date: 4/25/2019
 *
 * Description: A basic version of code to set up and receive a packet.
 *  - This is designed to work with the EVM the BOOSTXL-CANFD-LIN Rev 1.0 Boosterpack
 *  - It assumes TCAN4550 Oscillator of 40 MHz
 *  - Sets CAN arbitration rate at 500 kBaud
 *  - Sets CAN FD data phase for 2 MBaud
 *  - The interrupt pin is used for signal a received message, rather than polling
 *
 *   Pressing S1 will transmit a CAN message. S1 is on the MSP430FR6989 launchpad to the left.
 *
 *  Pinout
 *   - P1.4 SPI Clock / SCLK
 *   - P1.6 MOSI / SDI
 *   - P1.7 MISO / SDO
 *   - P2.5 SPI Chip Select / nCS
 *
 *   - P2.3 MCAN Interrupt 1 / nINT
 *   - Ground wire is important
 */

#include "stm32f10x.h"
#include "TCAN4550.h"

#define RCC_S1_PIN     RCC_APB2Periph_GPIOB 
#define PORT_S1_PIN     GPIOB
#define S1_PIN 					GPIO_Pin_12   
#define APP_S1_READ() GPIO_ReadInputDataBit(PORT_S1_PIN,S1_PIN)

#define RCC_RST_PIN     RCC_APB2Periph_GPIOC
#define PORT_RST_PIN     GPIOC
#define RST_PIN 					GPIO_Pin_2  

#define SET_RST_HIGH() GPIO_SetBits(PORT_RST_PIN,RST_PIN)
#define SET_RST_LOW() GPIO_ResetBits(PORT_RST_PIN,RST_PIN)



#define RCC_LED0_PIN      RCC_APB2Periph_GPIOA
#define PORT_LED0_PIN     GPIOA
#define LED0_PIN 					GPIO_Pin_15                 
#define LED_Off(LED0) 		GPIO_SetBits(PORT_LED0_PIN,LED0##_PIN);
#define LED_On(LED0) 			GPIO_ResetBits(PORT_LED0_PIN,LED0##_PIN);



#define RCC_INT_IN      	 RCC_APB2Periph_GPIOC
#define PORT_INT_IN      	 GPIOC

#define RCC_INT_TX_IN      RCC_APB2Periph_GPIOC
#define PORT_INT_TX_IN     GPIOC

#define RCC_INT_RX_IN      RCC_APB2Periph_GPIOC
#define PORT_INT_RX_IN		 GPIOC
#define CANIF_MAX_CAN_DATA   64
#define CANIF_MAX_CANFD_DATA   64
#define CANIF_MAX_CANSTD_DATA     8


// Interrupts
#define INT_IN			GPIO_Pin_4
#define INT_TX_IN		GPIO_Pin_1 
#define INT_RX_IN		GPIO_Pin_0 

#define APP_INT()		 (!GPIO_ReadInputDataBit(PORT_INT_IN,INT_IN))
#define APP_TX_INT()	(!GPIO_ReadInputDataBit(PORT_INT_TX_IN,INT_TX_IN))
#define APP_RX_INT()	(!GPIO_ReadInputDataBit(PORT_INT_RX_IN,INT_RX_IN))


void Init_GPIO(void);
void Init_Clock(void);
void Init_SPI(void);
void Init_CAN(void);

volatile uint8_t TCAN_Int_Cnt = 0;					// A variable used to keep track of interrupts the MCAN Interrupt pin
uint8_t tx_led_blink_flag = 0;
uint16_t g_RxCount =0;
uint8_t  g_RxFifo[CANIF_MAX_CANFD_DATA];

//LED1 处理程序
void Blink_Led_Process(void)
{
	static uint32_t time_out_count = 0;
	if(tx_led_blink_flag == 1)
	{
		time_out_count++;
		LED_On(LED0) ;
		
	}
	if(time_out_count >= 100000)
	{
		time_out_count = 0;
		tx_led_blink_flag = 0;
			LED_Off(LED0) ;
	}
}


			
uint32_t id_test=0;
int main(void)
{
	uint32_t times=10000;

	
	SET_RST_HIGH();
	while(times--);//上电等待一会
	SET_RST_LOW();

	Init_GPIO();									// Set up GPIOs for SPI and TCAN4550 connections
	Init_SPI();										// Initialize the SPI hardware module for 2 MHz SPI
	

				//硬件复位tcan4550芯片
	times=100000;
	SET_RST_HIGH();
	while(times--);//
	SET_RST_LOW();
	times=100000;
	while(times--);//

	id_test = AHB_READ_32(REG_SPI_DEVICE_ID0);
	if(id_test != 0x4E414354)
	{
		while(1);//SPI通信失败
	}
	/*********************************************
	 * Everything at this point is for TCAN4550  *
	 *********************************************/
	Init_CAN();										// Run the main MCAN configuration sequence. The bulk of the configuration is in this!
	/* Define the CAN message we want to send*/
	TCAN4x5x_MCAN_TX_Header header = {0};			// Remember to initialize to 0, or you'll get random garbage!
	uint8_t data[8] = {0x55, 0x66, 0x77, 0x88};		// Define the data payload
	header.DLC = MCAN_DLC_8B;						// Set the DLC to be equal to or less than the data payload (it is ok to pass a 64 byte data array into the WriteTXFIFO function if your DLC is 8 bytes, only the first 8 bytes will be read)
	header.ID = 0x144;								// Set the ID
	header.FDF = 1;									// CAN FD frame enabled
	header.BRS = 1;									// Bit rate switch enabled
	header.EFC = 0;
	header.MM  = 0;
	header.RTR = 0;
	header.XTD = 0;									// We are not using an extended ID in this example
	header.ESI = 0;									// Error state indicator


	TCAN4x5x_MCAN_WriteTXBuffer(0, &header, data);	// This function actually writes the header and data payload to the TCAN's MRAM in the specified TX queue number. It returns the bit necessary to write to TXBAR,
													// but does not necessarily require you to use it. In this example, we won't, so that we can send the data queued up at a later point.

	/* Let's make a different CAN message */
	data[0] = 0x11;
	data[1] = 0x22;
	data[2] = 0x33;
	data[3] = 0x44;									// Define the data payload

	header.DLC = MCAN_DLC_8B;						// Set the DLC to be equal to or less than the data payload (it is ok to pass a 64 byte data array into the WriteTXFIFO function if your DLC is 8 bytes, only the first 8 bytes will be read)
	header.ID = 0x123;								// Set the ID
	header.FDF = 1;									// CAN FD frame enabled
	header.BRS = 1;									// Bit rate switch enabled
	header.EFC = 0;
	header.MM  = 0;
	header.RTR = 0;
	header.XTD = 0;									// We are not using an extended ID in this example
	header.ESI = 0;									// Error state indicator

	TCAN4x5x_MCAN_WriteTXBuffer(1, &header, data);	// This line writes the data and header to TX FIFO 1
	TCAN4x5x_MCAN_TransmitBufferContents(1);		// Request that TX Buffer 1 be transmitted


	TCAN4x5x_MCAN_TransmitBufferContents(0);		// Now we can send the TX FIFO element 0 data that we had queued up earlier but didn't send.

	while (1)
	{
		Blink_Led_Process();
		
		if (TCAN_Int_Cnt > 0 )
		{
			TCAN_Int_Cnt--;
			TCAN4x5x_Device_Interrupts dev_ir = {0};            // Define a new Device IR object for device (non-CAN) interrupt checking
			TCAN4x5x_MCAN_Interrupts mcan_ir = {0};				// Setup a new MCAN IR object for easy interrupt checking
			TCAN4x5x_Device_ReadInterrupts(&dev_ir);            // Read the device interrupt register
			TCAN4x5x_MCAN_ReadInterrupts(&mcan_ir);		        // Read the interrupt register

			if (dev_ir.SPIERR)                                  // If the SPIERR flag is set
			    TCAN4x5x_Device_ClearSPIERR();                  // Clear the SPIERR flag
			if (mcan_ir.RF0W || mcan_ir.RF0F){//RF0N	if (mcan_ir.RF0N)		// If a new message in RX FIFO 0
				TCAN4x5x_MCAN_RX_Header MsgHeader = {0};		// Initialize to 0 or you'll get garbage
				uint8_t ret = CANIF_MAX_CANSTD_DATA;                           // Used since the ReadNextFIFO function will return how many bytes of data were read
				uint8_t dataPayload[64] = {0};                  // Used to store the received data

				while(ret >= CANIF_MAX_CANSTD_DATA){
					TCAN4x5x_MCAN_ClearInterrupts(&mcan_ir);	    // Clear any of the interrupt bits that are set.
					ret = TCAN4x5x_MCAN_ReadNextFIFO( RXFIFO0, &MsgHeader, dataPayload);	// This will read the next element in the RX FIFO 0

				// numBytes will have the number of bytes it transfered in it. Or you can decode the DLC value in MsgHeader.DLC
				// The data is now in dataPayload[], and message specific information is in the MsgHeader struct.
				if (MsgHeader.ID == 0x00258003)		// 0x00258003 Example of how you can do an action based off a received address
				{
					if(dataPayload[0]==0x01)	//点亮LED
					{
						LED_On(LED0);
					}
					else
					{
						LED_Off(LED0);
					}
					// Do something
				}//end ID
				
			}//while
		}
		}
	}
}

TCAN4x5x_MCAN_Nominal_Timing_Simple TCANNomTiming = {0};	// 500k arbitration with a 40 MHz crystal ((40E6 / 2) / (32 + 8) = 500E3)
TCAN4x5x_MCAN_Data_Timing_Simple TCANDataTiming = {0};		// 2 Mbps CAN FD with a 40 MHz crystal (40E6 / (15 + 5) = 2E6)
void TCAN4x5x_Set_Baudrate(uint16_t NomTiming,uint16_t DataTiming)
{
	switch(NomTiming)
	{
		case 50:
			TCANNomTiming.NominalBitRatePrescaler = 20;
				TCANNomTiming.NominalTqBeforeSamplePoint = 32;
				TCANNomTiming.NominalTqAfterSamplePoint = 8;
			break;
	case 100:
				TCANNomTiming.NominalBitRatePrescaler = 10;
				TCANNomTiming.NominalTqBeforeSamplePoint = 32;
				TCANNomTiming.NominalTqAfterSamplePoint = 8;
			break;
	case 125:
		TCANNomTiming.NominalBitRatePrescaler = 8;
				TCANNomTiming.NominalTqBeforeSamplePoint = 32;
				TCANNomTiming.NominalTqAfterSamplePoint = 8;
			break;
	case 250:
				TCANNomTiming.NominalBitRatePrescaler = 4;
				TCANNomTiming.NominalTqBeforeSamplePoint = 32;
				TCANNomTiming.NominalTqAfterSamplePoint = 8;
			break;
		case 500:
				TCANNomTiming.NominalBitRatePrescaler = 2;
				TCANNomTiming.NominalTqBeforeSamplePoint = 32;
				TCANNomTiming.NominalTqAfterSamplePoint = 8;
			break;
		case 800:
				TCANNomTiming.NominalBitRatePrescaler = 1;
				TCANNomTiming.NominalTqBeforeSamplePoint = 37;
				TCANNomTiming.NominalTqAfterSamplePoint = 13;
			break;
		case 1000:
				TCANNomTiming.NominalBitRatePrescaler = 1;
				TCANNomTiming.NominalTqBeforeSamplePoint = 30;
				TCANNomTiming.NominalTqAfterSamplePoint = 10;
			break;
	}
	
	
	switch(DataTiming)
	{
		case 1000:
			TCANDataTiming.DataBitRatePrescaler = 1;
			TCANDataTiming.DataTqBeforeSamplePoint = 30;
			TCANDataTiming.DataTqAfterSamplePoint = 10;
			break;
	case 2000:
				TCANDataTiming.DataBitRatePrescaler = 1;
			TCANDataTiming.DataTqBeforeSamplePoint = 15;
			TCANDataTiming.DataTqAfterSamplePoint = 5;
			break;
	case 3000:
				TCANDataTiming.DataBitRatePrescaler = 1;
			TCANDataTiming.DataTqBeforeSamplePoint = 9;
			TCANDataTiming.DataTqAfterSamplePoint = 4;
			break;
	case 4000:
					TCANDataTiming.DataBitRatePrescaler = 1;
			TCANDataTiming.DataTqBeforeSamplePoint = 7;
			TCANDataTiming.DataTqAfterSamplePoint = 3;
			break;
		case 5000:
				TCANDataTiming.DataBitRatePrescaler = 1;
			TCANDataTiming.DataTqBeforeSamplePoint = 6;
			TCANDataTiming.DataTqAfterSamplePoint = 2;
			break;
		case 8000:
				TCANDataTiming.DataBitRatePrescaler = 1;
			TCANDataTiming.DataTqBeforeSamplePoint = 3;
			TCANDataTiming.DataTqAfterSamplePoint = 2;
			break;
	}

	
}
	

/*
 * Configure the TCAN4550
 */
void
Init_CAN(void)
{
	
	
  TCAN4x5x_Device_ClearSPIERR();                              // Clear any SPI ERR flags that might be set as a result of our pin mux changing during MCU startup

    /* Step one attempt to clear all interrupts */
	TCAN4x5x_Device_Interrupt_Enable dev_ie = {0};				// Initialize to 0 to all bits are set to 0.
	TCAN4x5x_Device_ConfigureInterruptEnable(&dev_ie);	        // Disable all non-MCAN related interrupts for simplicity

	TCAN4x5x_Device_Interrupts dev_ir = {0};					// Setup a new MCAN IR object for easy interrupt checking
	TCAN4x5x_Device_ReadInterrupts(&dev_ir);					// Request that the struct be updated with current DEVICE (not MCAN) interrupt values

	if (dev_ir.PWRON)                                           // If the Power On interrupt flag is set
		TCAN4x5x_Device_ClearInterrupts(&dev_ir);               // Clear it because if it's not cleared within ~4 minutes, it goes to sleep

//设置波特率500KB classic CAN not CAN FD
	TCAN4x5x_Set_Baudrate(500,500);

	/* Configure the MCAN core settings */
	TCAN4x5x_MCAN_CCCR_Config cccrConfig = {0};					// Remember to initialize to 0, or you'll get random garbage!
	cccrConfig.FDOE = 0;//1										// CAN FD mode enable
	cccrConfig.BRSE = 0;//1										// CAN FD Bit rate switch enable

	/* Configure the default CAN packet filtering settings */
	TCAN4x5x_MCAN_Global_Filter_Configuration gfc = {0};
	gfc.RRFE = 0;      //1                                         // Reject remote frames (TCAN4x5x doesn't support this)
	gfc.RRFS = 0;      //1                                         // Reject remote frames (TCAN4x5x doesn't support this)
	
	gfc.ANFE = TCAN4x5x_GFC_REJECT;                // Default behavior if incoming message doesn't match a filter is to reject for extended ID messages (29 bit IDs)
	gfc.ANFS = TCAN4x5x_GFC_REJECT;                // Default behavior if incoming message doesn't match a filter is to reject for standard ID messages (11 bit IDs)
//	gfc.ANFE = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for extended ID messages (29 bit IDs)
//	gfc.ANFS = TCAN4x5x_GFC_ACCEPT_INTO_RXFIFO0;                // Default behavior if incoming message doesn't match a filter is to accept into RXFIO0 for standard ID messages (11 bit IDs)

	/* ************************************************************************
	 * In the next configuration block, we will set the MCAN core up to have:
	 *   - 1 SID filter element
	 *   - 1 XID Filter element
	 *   - 5 RX FIFO 0 elements
	 *   - RX FIFO 0 supports data payloads up to 64 bytes
	 *   - RX FIFO 1 and RX Buffer will not have any elements, but we still set their data payload sizes, even though it's not required
	 *   - No TX Event FIFOs
	 *   - 2 Transmit buffers supporting up to 64 bytes of data payload
	 */
	TCAN4x5x_MRAM_Config MRAMConfiguration = {0};
	MRAMConfiguration.SIDNumElements = 0;						// Standard ID number of elements, you MUST have a filter written to MRAM for each element defined
	MRAMConfiguration.XIDNumElements = 8; //1 aizj						// Extended ID number of elements, you MUST have a filter written to MRAM for each element defined
	MRAMConfiguration.Rx0NumElements = 32; //2 aizj		// RX0 Number of elements RemoteFrame <32;STD 64
	MRAMConfiguration.Rx0ElementSize = MRAM_64_Byte_Data;		// RX0 data payload size
	MRAMConfiguration.Rx1NumElements = 0;						// RX1 number of elements
	MRAMConfiguration.Rx1ElementSize = MRAM_64_Byte_Data;		// RX1 data payload size
	MRAMConfiguration.RxBufNumElements = 64;						// RX buffer number of elements
	MRAMConfiguration.RxBufElementSize = MRAM_64_Byte_Data;		// RX buffer data payload size
	MRAMConfiguration.TxEventFIFONumElements = 0;				// TX Event FIFO number of elements
	MRAMConfiguration.TxBufferNumElements = 8;//2;					// TX buffer number of elements
	MRAMConfiguration.TxBufferElementSize = MRAM_64_Byte_Data;	// TX buffer data payload size


	/* Configure the MCAN core with the settings above, the changes in this block are write protected registers,      *
	 * so it makes the most sense to do them all at once, so we only unlock and lock once                             */

	TCAN4x5x_MCAN_EnableProtectedRegisters();					// Start by making protected registers accessible
	TCAN4x5x_MCAN_ConfigureCCCRRegister(&cccrConfig);			// Enable FD mode and Bit rate switching
	TCAN4x5x_MCAN_ConfigureGlobalFilter(&gfc);                  // Configure the global filter configuration (Default CAN message behavior)
	TCAN4x5x_MCAN_ConfigureNominalTiming_Simple(&TCANNomTiming);// Setup nominal/arbitration bit timing
	TCAN4x5x_MCAN_ConfigureDataTiming_Simple(&TCANDataTiming);	// Setup CAN FD timing
	TCAN4x5x_MRAM_Clear();										// Clear all of MRAM (Writes 0's to all of it)
	TCAN4x5x_MRAM_Configure(&MRAMConfiguration);				// Set up the applicable registers related to MRAM configuration
	TCAN4x5x_MCAN_DisableProtectedRegisters();					// Disable protected write and take device out of INIT mode


	/* Set the interrupts we want to enable for MCAN */
	TCAN4x5x_MCAN_Interrupt_Enable mcan_ie = {0};				// Remember to initialize to 0, or you'll get random garbage!
	//mcan_ie.RF0NE = 0;											// RX FIFO 0 new message interrupt enable
	mcan_ie.RF0WE = 1;
	mcan_ie.RF0FE =1;
	TCAN4x5x_MCAN_ConfigureInterruptEnable(&mcan_ie);			// Enable the appropriate registers


	/* Setup filters, this filter will mark any message with ID 0x055 as a priority message */
	TCAN4x5x_MCAN_SID_Filter SID_ID = {0};
	
	SID_ID.SFT = TCAN4x5x_SID_SFT_CLASSIC;//TCAN4x5x_SID_SFT_RANGE;//TCAN4x5x_SID_SFT_DUALID;//TCAN4x5x_SID_SFT_CLASSIC;						// SFT: Standard filter type. Configured as a classic filter
	SID_ID.SFEC = TCAN4x5x_SID_SFEC_PRIORITYSTORERX0;			// Standard filter element configuration, store it in RX fifo 0 as a priority message
	SID_ID.SFID1 = 0x55;										// SFID1 (Classic mode Filter)
	SID_ID.SFID2 = 0x7FF;										// SFID2 (Classic mode Mask)
	TCAN4x5x_MCAN_WriteSIDFilter(0, &SID_ID);					// Write to the MRAM
	


	/* Store ID 0x12345678 as a priority message */
/* {0x00258001,0x00258002, 0x00258003, 0x00258004, 0x00198001, 0x00198002, 0x00198003, 0x00198004}*/
    uint32_t ExFilter[] = {0x00258001,0x00258002, 0x00258003, 0x00258004, 0x00198001, 0x00198002, 0x00198003, 0x00198004};
	TCAN4x5x_MCAN_XID_Filter XID_ID = {0};
	XID_ID.EFT = TCAN4x5x_XID_EFT_CLASSIC;                      // EFT
	XID_ID.EFEC = TCAN4x5x_XID_EFEC_PRIORITYSTORERX0;           // EFEC
	XID_ID.EFID2 = 0x1FFFFFFF;                                  // EFID2 (Classic mode mask)
    for(int i=0; i< sizeof(ExFilter)/sizeof(ExFilter[0]);i++){
	    XID_ID.EFID1 = ExFilter[i];                                  // EFID1 (Classic mode filter)
        TCAN4x5x_MCAN_WriteXIDFilter(i, &XID_ID);                   // Write to the MRAM    
	}
	

	/* Configure the TCAN4550 Non-CAN-related functions */
	TCAN4x5x_DEV_CONFIG devConfig = {0};                        // Remember to initialize to 0, or you'll get random garbage!
	devConfig.SWE_DIS = 0;                                      // Keep Sleep Wake Error Enabled (it's a disable bit, not an enable)
	devConfig.DEVICE_RESET = 0;                                 // Not requesting a software reset
	devConfig.WD_EN = 0;                                        // Watchdog disabled
	devConfig.nWKRQ_CONFIG = 0;                                 // Mirror INH function (default)
	devConfig.INH_DIS = 0;                                      // INH enabled (default)
	devConfig.GPIO1_GPO_CONFIG = TCAN4x5x_DEV_CONFIG_GPO1_MCAN_INT1;    // MCAN nINT 1 (default)
	devConfig.FAIL_SAFE_EN = 0;                                 // Failsafe disabled (default)
	devConfig.GPIO1_CONFIG = TCAN4x5x_DEV_CONFIG_GPIO1_CONFIG_GPO;      // GPIO set as GPO (Default)
	devConfig.WD_ACTION = TCAN4x5x_DEV_CONFIG_WDT_ACTION_nINT;  // Watchdog set an interrupt (default)
	devConfig.WD_BIT_RESET = 0;                                 // Don't reset the watchdog
	devConfig.nWKRQ_VOLTAGE = 0;                                // Set nWKRQ to internal voltage rail (default)
	devConfig.GPO2_CONFIG = TCAN4x5x_DEV_CONFIG_GPO2_NO_ACTION; // GPO2 has no behavior (default)
	devConfig.CLK_REF = 1;                                      // Input crystal is a 40 MHz crystal (default)
	devConfig.WAKE_CONFIG = TCAN4x5x_DEV_CONFIG_WAKE_BOTH_EDGES;// Wake pin can be triggered by either edge (default)
	TCAN4x5x_Device_Configure(&devConfig);                      // Configure the device with the above configuration

	TCAN4x5x_Device_SetMode(TCAN4x5x_DEVICE_MODE_NORMAL);       // Set to normal mode, since configuration is done. This line turns on the transceiver

	TCAN4x5x_MCAN_ClearInterruptsAll();                         // Resets all MCAN interrupts (does NOT include any SPIERR interrupts)
	
	
	
}








/*******************************************************************************
* Function Name  : EXTI_Configuration
* Description    : Configures the different EXTI lines.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);

  EXTI_ClearITPendingBit(EXTI_Line12); //S1
	EXTI_ClearITPendingBit(EXTI_Line4);	//INT


  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	


  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/*
 * GPIO Initialization
 */
void
Init_GPIO()
{

		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_APB2PeriphClockCmd( RCC_RST_PIN|RCC_INT_IN|RCC_INT_TX_IN|RCC_INT_RX_IN|RCC_LED0_PIN|RCC_S1_PIN , ENABLE); 	
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE); 	
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
		/**
		*	INT-> PB6 PB8 PB9
		*/					 
		GPIO_InitStructure.GPIO_Pin = INT_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_IN, &GPIO_InitStructure);	
	
	/**
		*	INT-> PC4 PC0 PC1
		*/					 
		GPIO_InitStructure.GPIO_Pin = INT_IN;    //PC4  to TCAN4550 /INT 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_IN, &GPIO_InitStructure);	
	
		GPIO_InitStructure.GPIO_Pin = INT_TX_IN;  //no used
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_TX_IN, &GPIO_InitStructure);	
	
		GPIO_InitStructure.GPIO_Pin = INT_RX_IN;  //no used
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_RX_IN, &GPIO_InitStructure);	
		
		/**
		*	LED
		*/					 
		GPIO_InitStructure.GPIO_Pin =LED0_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(PORT_LED0_PIN, &GPIO_InitStructure);
		
		
				/**
		*	RST
		*/					 
		GPIO_InitStructure.GPIO_Pin =RST_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(PORT_RST_PIN, &GPIO_InitStructure);
		
	

			/**
		*	S1
		*/	
		GPIO_InitStructure.GPIO_Pin = S1_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_S1_PIN, &GPIO_InitStructure);
		
		EXTI_Configuration();
		

}




/*
 * Initialize the EUSCI B SPI
 */
void Init_SPI()
{	
	SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_SPI_FLASH_CS, ENABLE); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	

								  
  GPIO_InitStructure.GPIO_Pin = SPI_FALSH_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_FALSH_CS_PORT, &GPIO_InitStructure);
	
	
//	 GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	
  SPI_FLASH_CS_HIGH();
  /* SPI2 Config -------------------------------------------------------------*/ 								  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  /* Enable SPI1 */ 
  SPI_Cmd(SPI1, ENABLE); 
	
}


/*
 * S1按键中断处理程序
 */
void EXTI15_10_IRQHandler(void)
{
	
		if ( EXTI_GetITStatus(EXTI_Line12) != RESET ) 
		{
				
				tx_led_blink_flag = 1;
			
				EXTI_ClearITPendingBit(EXTI_Line12);
				TCAN4x5x_MCAN_TransmitBufferContents(0); 
			
			
		}
}



/*
 * CAN FD接收中断处理函数
 */
void EXTI4_IRQHandler(void)
{
	
		if ( EXTI_GetITStatus(EXTI_Line4) != RESET ) 
		{
				EXTI_ClearITPendingBit(EXTI_Line4);
				TCAN_Int_Cnt++;
		}
}
