// A simple program for STM32F103C8 depending on module tc35661-spp-mcu-driver
// which sends data received from Blutooth SPP back to the remote device.

#include "string.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_systick.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_pwr.h"

#include "tc35661.h"

#define Flash_Link_Storage_Address (0x8020000 - 2 - TC_BT_Link_Storage_Size) //stores at the end, but last two bytes are reserved

uint8_t tc_uart_buff[TC_UART_Buff_Max_Size] = {0x00};
volatile uint16_t tc_uart_rec_count = 0;

uint8_t spp_received = 0, flash_link_key = 0; //bool tags (not extern)

/*********************************************************************************************************
      Delay
*********************************************************************************************************/
extern void delay_ms(uint16_t ms)
{
	uint16_t delay_fac_ms = 0;  // us delay factor
	FlagStatus Status;
	
	RCC_ClocksTypeDef RCC_ClocksStatus;
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	delay_fac_ms = RCC_ClocksStatus.HCLK_Frequency / 8000;
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);  //choose external clock HCLK/8
	SysTick_ITConfig(DISABLE);
	
	SysTick_SetReload(delay_fac_ms);
	SysTick_CounterCmd(SysTick_Counter_Clear); //clear counter

	SysTick_CounterCmd(SysTick_Counter_Enable);	
	do
	{		
		//wait until elapsed
		do
		{
			Status = SysTick_GetFlagStatus(SysTick_FLAG_COUNT);
		} while (Status != SET);
		//After SET value is read, FlagStatus should RESET
	} while (ms--);
	
	SysTick_CounterCmd(SysTick_Counter_Disable);
}

/*********************************************************************************************************
      Flash
*********************************************************************************************************/
uint8_t flash_read_link_storage(TC_BT_Link_Storage* link) //bool, return 0 if the flash region is empty
{
	uint16_t* p = (uint16_t*) link;
	uint8_t i, not_empty = 0;
	for (i = 0; i < TC_BT_Link_Storage_Size / 2; i++, p++) {
		*p = *(uint16_t*)(Flash_Link_Storage_Address + 2 * i);
		if (*p != 0xFFFF)
			not_empty = 1;
	}
	return not_empty;
}

void flash_write_link_storage(TC_BT_Link_Storage* link)
{
	uint16_t* p = (uint16_t*) link;
	uint8_t i;
	
	uint32_t page_addr;
	page_addr = Flash_Link_Storage_Address - FLASH_BASE;
	page_addr = FLASH_BASE + page_addr - page_addr % 2048;
	
	FLASH_Unlock();
	FLASH_ErasePage(page_addr);
	
	for (i = 0; i < TC_BT_Link_Storage_Size / 2; i++, p++)
		FLASH_ProgramHalfWord(Flash_Link_Storage_Address + 2 * i, *p);
	
	FLASH_Lock();
}

/*********************************************************************************************************
      USART
*********************************************************************************************************/
void tc_uart_init(uint32_t baud_rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	//Initialize the two GPIO pins used by USART peripheral
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	
	//Reset USART peripheral through APB2 bus
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	
	//Initialize USART
	USART_InitStructure.USART_BaudRate = baud_rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//Enable USART1 IRQ Channel of NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//Enable USART Interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE); 
	USART_ClearFlag(USART1,USART_FLAG_TC);
	
}

void USART1_IRQHandler(void)
{
	uint8_t tmp;
		
	USART_ClearITPendingBit(USART1,USART_IT_RXNE); //for the next possible interrupt
	
	tmp = USART_ReceiveData(USART1);//read one byte. it involves read operation of USART1->DR.

	if(tc_uart_rec_count > TC_UART_Buff_Max_Size)
		tc_uart_rec_count = 0;
	
	tc_uart_buff[tc_uart_rec_count++] = tmp;
	tc_uart_receive_handler();
}

extern void tc_uart_send_bytes(uint8_t* bytes, uint16_t count)
{
	int i; uint8_t* b;
	b = bytes;
	for (i = 0; i < count; i++)
	{
		USART_SendData(USART1, *b);
		b++;
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

/*********************************************************************************************************
      LED
*********************************************************************************************************/
void LED1_Init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
}

void LED1_ON(void) {GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);}
void LED1_OFF(void) {GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);}

/*********************************************************************************************************
      Main
*********************************************************************************************************/
extern void tc_bt_event_handler(TC_BT_Event_Type event)
{
	switch (event)
	{
		case TC_BT_Event_Initialized:
			LED1_ON(); delay_ms(150); LED1_OFF(); break;
		case TC_BT_Event_Paired:
			LED1_ON(); delay_ms(150); LED1_OFF();
			flash_link_key = 1; break;//let main() store the link key in flash memory
		case TC_BT_Event_Connect_Failed:
			LED1_ON(); delay_ms(750); LED1_OFF();
			LED1_OFF(); break;
		case TC_BT_Event_Connect:
			LED1_ON(); break;
		case TC_BT_Event_Disconnected:
			LED1_OFF(); break;
		case TC_BT_Event_SPP_Receive:
			spp_received = 1; break; //let main() send back the SPP data received
		case TC_BT_Event_SPP_Sent:
			LED1_ON(); break;
		default: break;
	}
}

int main()
{
	uint8_t addr[6] = {0x12, 0x34, 0x56, 0x33, 0x22, 0x11};
	memcpy(tc_bt_conf.local_addr, addr, 6); //definition of tc_bt_conf is in tc35661 module
	tc_bt_conf.device_name_length = 8;
	strcpy(tc_bt_conf.device_name, "35661SPP");
	tc_bt_conf.link_stored = flash_read_link_storage(& tc_bt_conf.valid_link);
	
	delay_ms(5000); //wait for possible debugging session which will restart the program
	
	LED1_Init();
	tc_uart_init(115200);

	delay_ms(300);
	if (tc_uart_rec_count > 0) tc_uart_rec_count = 0;
	tc_bt_init();
	
	while (1)
	{
		if (spp_received) {
			spp_received = 0; //clear the tag
			
			if (tc_bt_spp_rec_count > TC_BT_SPP_Single_Block_Max_Size)
				USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //might not needed
			
			tc_bt_spp_send(tc_bt_spp_buff, tc_bt_spp_rec_count);
			tc_bt_spp_rec_count = 0; //clear buffer
			
			if (tc_bt_spp_rec_count > TC_BT_SPP_Single_Block_Max_Size)
				USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			
			LED1_OFF();
		}
		if (flash_link_key) {
			flash_write_link_storage(& tc_bt_conf.valid_link);
			flash_link_key = 0; //!!! IMPORTANT
		}

	}
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

