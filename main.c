// A simple program for STM32F103C8T6 which sends data received from Blutooth SPP
// back to the remote device. It depends on module tc35661-spp-mcu-driver.

#include "string.h"

#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_dbgmcu.h"

#include "tc35661.h"

// #define DEBUG

// Uncomment the next line if connections from only one remote device should be allowed.
// #define Bluetooth_Fixed_Link
#ifdef Bluetooth_Fixed_Link
	//stores at the end, but last two bytes are reserved
	#define Flash_Link_Storage_Address (0x8020000 - 2 - TC_BT_Link_Storage_Size)
#endif

#define Bluetooth_State_Backup_Reg BKP_DR4

//bluetooth address is set in reversed sequence
const uint8_t Bluetooth_Address[6] = {0x51, 0x9a, 0xd0, 0x3c, 0x28, 0x3f};
const char Bluetooth_Device_Name[] = "35661";

//extern
uint8_t tc_uart_buff[TC_UART_Buff_Max_Size] = {0x00};
volatile uint16_t tc_uart_rec_count = 0;

//not extern
volatile bool tag_tc_uart_received = false, tag_spp_received = false, tag_flash_link_key = false;
bool tag_disconnect_before_sleep = false;
bool tag_systick_init = false;
volatile uint16_t delay_remain = 0, bt_delay_remain = 0;
volatile uint32_t sleep_tick_count = 0;

void delay_init(void);
void delay_ms(uint16_t ms);
extern void bt_delay_ms(uint16_t ms);

void sleep_deep(void);
void state_backup(void); //it's useful after reset, avoiding failure of reinitializing the bluetooth module
void state_recover(void);

#ifdef Bluetooth_Fixed_Link
bool flash_read_link_storage(TC_BT_Link_Storage* link);
void flash_write_link_storage(TC_BT_Link_Storage* link);
#endif

void exti10_enable(void);
void exti10_disable(void);
void EXTI15_10_IRQHandler(void);
void tc_uart_init(void); //PA9 (USART1_Tx) is connected to Rx of TC35661, PA10 (Rx) is connected to Tx of TC35661
void tc_uart_disable(void);
extern void tc_uart_send_bytes(uint8_t* bytes, uint16_t count);
void USART1_IRQHandler(void);

void led1_init(void); //GPIO B12 is connected to the LED's cathode
void led1_on(void);
void led1_off(void);

extern void tc_bt_event_handler(TC_BT_Event_Type event);
int main();

/*********************************************************************************************************
      Delay
*********************************************************************************************************/
void delay_init(void)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	if (tag_systick_init) return;
	
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //set to highest priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SystemCoreClockUpdate(); //get the current system clock frequency (Hz)

	// interrupt per 1 ms. modified from `SysTick_Config` in `core_cm3.h`
	SysTick->LOAD  = ((SystemCoreClock / 1000) & SysTick_LOAD_RELOAD_Msk) - 1;      /* set reload register */
	SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | 
					 SysTick_CTRL_TICKINT_Msk   | 
	SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
	
	tag_systick_init = true;
}

void SysTick_Handler(void)
{
	sleep_tick_count++;
	if (delay_remain > 0) delay_remain--;
	if (bt_delay_remain > 0) bt_delay_remain--;
}

void delay_ms(uint16_t ms)
{
	delay_remain = ms;
	while (delay_remain > 0);
}

extern void bt_delay_ms(uint16_t ms)
{
	bt_delay_remain = ms;
	while (bt_delay_remain > 0);
}

/*********************************************************************************************************
      Power
*********************************************************************************************************/
void sleep_deep(void)
{
	exti10_enable();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	#ifdef DEBUG
		DBGMCU_Config(DBGMCU_STOP, ENABLE);
	#endif
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
	
	//after waken up by the EXTI line
	SystemInit(); //Set system clock to HSE, then SysTick wouldn't need reset and reconfiguration
	tc_uart_init();
	sleep_tick_count = 0;
}

void state_backup(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	BKP_WriteBackupRegister(Bluetooth_State_Backup_Reg, (uint16_t)tc_driver_state & 0xFF);
	PWR_BackupAccessCmd(DISABLE);
}

void state_recover(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	tc_driver_state = (TC_BT_Driver_State)(uint8_t)(BKP_ReadBackupRegister(Bluetooth_State_Backup_Reg) & 0xFF);
	PWR_BackupAccessCmd(DISABLE);
}

/*********************************************************************************************************
      Flash
*********************************************************************************************************/
#ifdef Bluetooth_Fixed_Link

bool flash_read_link_storage(TC_BT_Link_Storage* link) //return false if the flash region is empty
{
	uint16_t* p = (uint16_t*) link;
	uint8_t i; bool not_empty = false;
	for (i = 0; i < TC_BT_Link_Storage_Size / 2; i++, p++) {
		*p = *(uint16_t*)(Flash_Link_Storage_Address + 2 * i);
		if (*p != 0xFFFF)
			not_empty = true;
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
#endif
/*********************************************************************************************************
      USART
*********************************************************************************************************/
void exti10_enable(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	tc_uart_disable();
	
	//Enable external interrupt that wakes up the MCU from stop mode
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void exti10_disable(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line10))
		EXTI_ClearITPendingBit(EXTI_Line10);
	//then execute the program behind the command of entering stop mode in function `sleep_deep()`
}

void tc_uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	exti10_disable();
	tc_uart_disable();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);
	
	//Initialize the two GPIO pins used by USART peripheral
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //USART1_RX	 PA.10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	//Enable USART1 IRQ Channel of NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//Enable USART Interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE); 
	USART_ClearFlag(USART1,USART_FLAG_TC);
	
	//Clear data that might be invalid
	delay_ms(300);
	if (tc_uart_rec_count > 0) tc_uart_rec_count = 0;
}

void tc_uart_disable(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_Cmd(USART1, DISABLE);
	USART_DeInit(USART1); //Reset USART peripheral through APB2 bus
}

extern void tc_uart_send_bytes(uint8_t* bytes, uint16_t count)
{
	int i; uint8_t* b = bytes;
	
	for (i = 0; i < count; i++)
	{
		USART_SendData(USART1, *b);
		b++;
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	}
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void USART1_IRQHandler(void)
{
	USART_ClearITPendingBit(USART1,USART_IT_RXNE); //for the next possible interrupt
	
	if(tc_uart_rec_count >= TC_UART_Buff_Max_Size)
		tc_uart_rec_count = 0;
	
	tc_uart_buff[tc_uart_rec_count++] = USART_ReceiveData(USART1); //read one byte
	tag_tc_uart_received = true;
}

/*********************************************************************************************************
      LED
*********************************************************************************************************/
void led1_init(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);
}

void led1_on(void) {GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);}
void led1_off(void) {GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);}

/*********************************************************************************************************
      Main
*********************************************************************************************************/
extern void tc_bt_event_handler(TC_BT_Event_Type event)
{
	switch (event)
	{
		case TC_BT_Event_Initialized:
			led1_on(); delay_ms(150); led1_off(); break;
		case TC_BT_Event_Paired:
			led1_on(); delay_ms(150); led1_off();
			#ifdef Bluetooth_Fixed_Link
				tag_flash_link_key = true;//let main() store the link key in flash memory
			#endif
			break;
		case TC_BT_Event_Connect_Failed:
			led1_on(); delay_ms(750); led1_off();
			led1_off(); break;
		case TC_BT_Event_Connect:
			led1_on(); break;
		case TC_BT_Event_Disconnected:
			led1_off(); break;
		case TC_BT_Event_SPP_Receive:
			tag_spp_received = true; break; //let main() send back the SPP data received
		case TC_BT_Event_SPP_Sent:
			led1_on(); break;
		default: break;
	}
	
	state_backup();
	if (! tag_disconnect_before_sleep)
		sleep_tick_count = 0;
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	delay_init();
	#ifdef DEBUG
		delay_ms(5000); //Note: after the MCU is powered on, start debug session as soon as possible.
	#else
		delay_ms(150);
	#endif
	state_recover(); //recover the bluetooth module state if the system has been reset
	memcpy(tc_bt_conf.local_addr, Bluetooth_Address, 6); //definition of tc_bt_conf is in tc35661 module
	strcpy(tc_bt_conf.device_name, Bluetooth_Device_Name);
	tc_bt_conf.device_name_length = strlen(Bluetooth_Device_Name);
	#ifdef Bluetooth_Fixed_Link
		tc_bt_conf.link_fixed = true;
		tc_bt_conf.link_stored = flash_read_link_storage(& tc_bt_conf.valid_link);
	#endif
	tc_uart_init();
	tc_bt_init();
	
	led1_init();
	if (tc_driver_state == TC_BT_Connected)
		led1_on();
	
	while (true)
	{
		if (tag_tc_uart_received) {
			tag_tc_uart_received = false;
			tc_uart_receive_handler();
			if (! tag_disconnect_before_sleep)
				sleep_tick_count = 0;
		}
		if (tag_spp_received) {
			tag_spp_received = false; //clear the tag
			
			if (tc_bt_spp_buff[0] == 0xff && tc_bt_spp_buff[1] == 0x00) {
				tc_bt_spp_rec_count = 0; //important for the next acception
				tc_bt_spp_disconnect();
				sleep_tick_count = 15 * 1000;
				tag_disconnect_before_sleep = true;
				continue;
			}
			
			tc_bt_spp_send(tc_bt_spp_buff, tc_bt_spp_rec_count);
			tc_bt_spp_rec_count = 0; //clear buffer
			
			led1_off();
		}
		
		if (sleep_tick_count > 15 * 1000) { //15 s
			if (tc_driver_state == TC_BT_Connected) {
				if (! tag_disconnect_before_sleep) {
					tc_bt_spp_disconnect();
					tag_disconnect_before_sleep = true;
				}
			} else if (tc_driver_state == TC_BT_Idle) {
				tag_disconnect_before_sleep = false;
				sleep_deep(); //enter sleep mode
			}
		}
		
		#ifdef Bluetooth_Fixed_Link
			if (tag_flash_link_key) {
				flash_write_link_storage(& tc_bt_conf.valid_link);
				tag_flash_link_key = false; //!!! IMPORTANT
			}
		#endif
	}
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
