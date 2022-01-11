// tc35661-spp-mcu-driver
// by wuwbobo2021 <wuwbobo@outlook.com> <https://github.com/wuwbobo2021>
// My father and I designed this program based on information given in
// <https://github.com/yuht/TC35661_Bluetooth>.

#include "stdint.h"
#include "stdbool.h"

#define TC_BT_Link_Storage_Size 0x22

#define TC_UART_Buff_Max_Size 0x1000
#define TC_BT_SPP_Buff_Max_Size 0x1000

#define TC_BT_SPP_Single_Block_Max_Size 0x21e

typedef struct {
	uint8_t remote_addr[6];
	uint8_t link_key[16];
} TC_BT_Link_Storage;

typedef struct {
	uint8_t local_addr[16];
	uint8_t device_name_length; //max length is 32
	char device_name[32];
	bool link_fixed: 1; //set to true if requests from only one remote device should be accepted 
	bool link_stored: 1; //set to true if a link key is stored
	TC_BT_Link_Storage valid_link;
} TC_BT_Config;

typedef enum {
	TC_BT_Before_Init = 0,
	TC_BT_Init,
	TC_BT_Idle,
	TC_BT_Pairing,
	TC_BT_Connecting,
	TC_BT_Connected,
	TC_BT_Disconnecting,
	TC_BT_Init_Failed
} TC_BT_Driver_State;

typedef enum {
	TC_BT_Event_Initialized = 0,
	TC_BT_Event_Init_Failed,
	TC_BT_Event_Paired,
	TC_BT_Event_Pair_Failed,
	TC_BT_Event_Rejected,
	TC_BT_Event_Connect,
	TC_BT_Event_Connect_Failed,
	TC_BT_Event_SPP_Receive,
	TC_BT_Event_SPP_Sent,
	TC_BT_Event_Disconnected
} TC_BT_Event_Type;

extern volatile TC_BT_Config tc_bt_conf;

extern uint8_t tc_uart_buff[]; //imported, UART receive buffer
extern volatile uint16_t tc_uart_rec_count; //imported, amount of bytes received from UART

extern volatile TC_BT_Driver_State tc_driver_state; //extern, indicates current state
extern volatile uint8_t rejected_remote_addr[]; //extern, it's valid only when TC_BT_Event_Rejected is raised

extern uint8_t tc_bt_spp_buff[]; //extern, SPP receive buffer
extern volatile uint16_t tc_bt_spp_rec_count; //extern, amount of bytes received from UART, it needs to be cleared manually

extern void bt_delay_ms(uint16_t ms); //imported. it depends on SysTick

// `void tc_uart_receive_handler()`: imported interrupt, it should be called in the main function's infinite loop,
// it would be better having a tag set in the UART IRQ Handler: call `tc_uart_receive_handler` in main function if the tag
// is set, then clear the tag. Don't call this function in UART IRQ Handler after being waken up from STOP mode.
extern void tc_uart_receive_handler(void);
extern void tc_uart_send_bytes(uint8_t* bytes, uint16_t count); //imported, sends bytes to bluetooth controller interface

extern void tc_bt_init(void); //extern, start this driver
extern void tc_bt_spp_send(uint8_t* bytes, uint16_t count); //extern, sends data to remote device after connected
extern void tc_bt_spp_disconnect(void); //extern, cancel current SPP connection

extern void tc_bt_event_handler(TC_BT_Event_Type event); //extern event, reports the event that happened
