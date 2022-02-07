// tc35661-spp-mcu-driver
// by wuwbobo2021 <wuwbobo@outlook.com> <https://github.com/wuwbobo2021>
// My father and I designed this program based on information given in
// <https://github.com/yuht/TC35661_Bluetooth>.

#include "tc35661.h"

typedef struct {
	uint8_t length; //I can't define `unsigned int length:24` or `uint16_t length`, because they cause problems
	uint8_t length2;
	uint8_t length3;
	uint8_t service_id;
	uint8_t op_code;
	uint8_t param_length;
	uint8_t param_length2;
	uint8_t param[0xffff];
} TCU_Command; //responses and events are the same


typedef enum {
	Connection_Status_Successful = 0x00,
	Connection_Status_Page_Timer_Out = 0x80,
	Connection_Status_Local_Device_Reject = 0x81,
	Connection_Status_Link_Loss = 0x82,
	Connection_Status_Link_Key_Failure = 0x87
} TCU_MNG_Connection_Status1;

typedef enum {
	Connection_Status_Connected = 0x00,
	Connection_Status_Disconnected,
	Connection_Status_Connection_Failure,
	Connection_Status_Link_Key,
	Connection_Status_Mode_Change_Active,
	Connection_Status_Mode_Change_Hold,
	Connection_Status_Mode_Change_Sniff,
	Connection_Status_Mode_Change_Park
} TCU_MNG_Connection_Status2;

typedef struct {
	TCU_MNG_Connection_Status1 status;
	uint8_t remote_address[6];
	TCU_MNG_Connection_Status2 connection_status;
	uint8_t link_key[16];
	uint8_t link_key_type; //undefined
} TCU_MNG_Connection_Status_Event;

typedef struct {
	uint8_t remote_address[6];
	uint8_t remote_class_uuid[3];
} TCU_MNG_Connection_Request_Event;

typedef enum {
	Seq_HCI_Reset = 0,
	Seq_HCI_Write_Address,
	Seq_HCI_RTS_CTS_Disable, //HCI Extension
	Seq_HCI_Set_Mode, //HCI Extention, enter TCU mode
	
	//TCU_MNG
	Seq_MNG_Init,
	Seq_MNG_SSP_Set1,
	Seq_MNG_SPP_Setup,
	Seq_MNG_SSP_Set2,
	Seq_MNG_SSP_Set3,
	Seq_MNG_Set_Scan
} Init_Seq;

uint8_t Init_HCI_Length[4] = {0x04, 0x0a, 0x07, 0x07};

//10 + 32 = 0x2a, hence the max length of device name is 32
uint8_t Init_Req[10][0x2a] = {{0x01, 0x03, 0x0c, 0x00},
                              {0x01, 0x13, 0x10, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //00s should be replaced with local address
                              {0x01, 0x08, 0xfc, 0x03, 0x00, 0x93, 0x00},
                              {0x01, 0x08, 0xfc, 0x03, 0x00, 0x99, 0x01},
							  
							  {0x14, 0x00, 0x00, 0xe1, 0x01, 0x0d, 0x00, 0x04, 0x02}, //1B local device name length and device name will be added at right
                              {0x0d, 0x00, 0x00, 0xe1, 0x3d, 0x06, 0x00, 0x24, 0x0c, 0x03, 0x04, 0x04, 0x24},
                              {0x07, 0x00, 0x00, 0xe5, 0x01, 0x00, 0x00},
                              {0x0e, 0x00, 0x00, 0xe1, 0x3d, 0x07, 0x00, 0x1c, 0x0c, 0x04, 0x00, 0x04, 0x12, 0x00},
                              {0x0e, 0x00, 0x00, 0xe1, 0x3d, 0x07, 0x00, 0x1e, 0x0c, 0x04, 0x00, 0x02, 0x12, 0x00},
                              {0x08, 0x00, 0x00, 0xe1, 0x0c, 0x01, 0x00, 0x03}};

typedef enum {
	Seq_MNG_Connection_Accept1 = 0,
	Seq_MNG_HCI_Set1,
	Seq_MNG_HCI_Set2,
	Seq_MNG_Connection_Accept2,
	
	Seq_Reject,
	Seq_Connect_Cancel
} Connect_Seq;

uint8_t Connect_Req[6][0x1f] = {{0x0f, 0x00, 0x00, 0xe1, 0x13, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //6B remote address and 1B use of link key at last
                                {0x13, 0x00, 0x00, 0xe1, 0x3d, 0x0c, 0x00, 0x2b, 0x04, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04},
                                {0x10, 0x00, 0x00, 0xe1, 0x3d, 0x09, 0x00, 0x2c, 0x04, 0x06}, //6B remote address at last, initialized with 0x00 (auto)
                                {0x1f, 0x00, 0x00, 0xe1, 0x13, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}, //16B link key at last
                                {0x0f, 0x00, 0x00, 0xe1, 0x13, 0x08, 0x00, 0x01}, //6B remote address at last
                                {0x07, 0x00, 0x00, 0xe1, 0x15, 0x00, 0x00}};

uint8_t SPP_Disconnect_Req[7] = {0x07, 0x00, 0x00, 0xe5, 0x04, 0x00, 0x00};

volatile TC_BT_Config tc_bt_conf; //extern

volatile TC_BT_Driver_State tc_driver_state = TC_BT_Before_Init;
volatile uint8_t rejected_remote_addr[6];

volatile Init_Seq init_seq_current = Seq_HCI_Reset;
volatile Connect_Seq connect_seq_current = Seq_MNG_Connection_Accept1;

volatile uint8_t* pbuff;

uint8_t tc_bt_spp_buff[TC_BT_SPP_Buff_Max_Size]; //extern, SPP receive buffer
volatile uint16_t tc_bt_spp_rec_count = 0; //extern

void memcpy(volatile uint8_t* p1, const volatile uint8_t* p2, uint16_t length)
{
	uint16_t i;
	for (i = 0; i < length; i++)
		p1[i] = p2[i];
}

bool array_equal(const volatile uint8_t* p1, const volatile uint8_t* p2, uint16_t length)
{
	uint16_t i;
	for (i = 0; i < length; i++)
		if (p1[i] != p2[i]) return false;
	
	return true;
}

void modify_commands(void)
{
	memcpy(Init_Req[Seq_HCI_Write_Address] + 4, tc_bt_conf.local_addr, 6);
	
	if (tc_bt_conf.device_name_length > 32) tc_bt_conf.device_name_length = 32;
	memcpy(Init_Req[Seq_MNG_Init] + 10, tc_bt_conf.device_name, tc_bt_conf.device_name_length);
	Init_Req[Seq_MNG_Init][0] = tc_bt_conf.device_name_length + 10;
	Init_Req[Seq_MNG_Init][5] = tc_bt_conf.device_name_length + 3;
	Init_Req[Seq_MNG_Init][9] = tc_bt_conf.device_name_length;
	
	memcpy(Connect_Req[Seq_MNG_Connection_Accept1] +  8, tc_bt_conf.valid_link.remote_addr, 6);
	memcpy(Connect_Req[Seq_MNG_HCI_Set1]           + 10, tc_bt_conf.valid_link.remote_addr, 6);
	memcpy(Connect_Req[Seq_MNG_HCI_Set2]           + 10, tc_bt_conf.valid_link.remote_addr, 6);
	memcpy(Connect_Req[Seq_MNG_Connection_Accept2] +  8, tc_bt_conf.valid_link.remote_addr, 6);
	
	if (tc_bt_conf.link_stored)
		memcpy(Connect_Req[Seq_MNG_Connection_Accept2] + 15, tc_bt_conf.valid_link.link_key, 16);
}

void step_init(void)
{
	uint8_t lsend;
	
	if (tc_driver_state == TC_BT_Before_Init) {
		tc_driver_state++;
		init_seq_current = Seq_HCI_Reset;
	} else if (tc_driver_state == TC_BT_Init) {
		if (init_seq_current < 4) { //HCI Response
			if (pbuff[0] != 0x04) return; //not HCI event
			if (pbuff[1] == 0x0f) { //HCI_Command_Status_Event
				if (array_equal(pbuff + 1 + 4, Init_Req[init_seq_current] + 1, 2)) //response of last command
					if (pbuff[1 + 2] != 0x00) //not successful
						{tc_driver_state = TC_BT_Init_Failed; tc_bt_event_handler(TC_BT_Event_Init_Failed); return;}
			} else //it should be a response of the specific type corresponding with that of the last command
			if (pbuff[1 + 5] != 0x00) { //not successful
				tc_driver_state = TC_BT_Init_Failed;
				tc_bt_event_handler(TC_BT_Event_Init_Failed);
				return;
			}
		} else {
			TCU_Command* tcu_resp = (TCU_Command*) pbuff;
			switch (init_seq_current)
			{
				case Seq_MNG_Init:
					if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x81) { //TCU_MNG_Init_Resp
						if (tcu_resp->param[0] != 0x00)
							{tc_driver_state = TC_BT_Init_Failed; tc_bt_event_handler(TC_BT_Event_Init_Failed); return;}
					} else return;
					break;
				
				case Seq_MNG_SSP_Set1: case Seq_MNG_SSP_Set2: case Seq_MNG_SSP_Set3:
					if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0xBD) { //TCU_MNG_SSP_Set_Resp
						if (tcu_resp->param[0] != 0x00)
							{tc_driver_state = TC_BT_Init_Failed; tc_bt_event_handler(TC_BT_Event_Init_Failed); return;}
					} else return;
					break;
				
				case Seq_MNG_SPP_Setup:
					if (tcu_resp->service_id == 0xE5 && tcu_resp->op_code == 0x81) { //TCU_SPP_Setup_Resp
						if (tcu_resp->param[0] == 0x00);
						else if (tcu_resp->param[0] == 0x40) return; //setuping SPP
						else {tc_driver_state = TC_BT_Init_Failed; tc_bt_event_handler(TC_BT_Event_Init_Failed); return;}
					} else return;
					break;
				
				case Seq_MNG_Set_Scan:
					if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x8C) { //MNG_Set_Scan_Resp
						if (tcu_resp->param[0] != 0x00)
							{tc_driver_state = TC_BT_Init_Failed; tc_bt_event_handler(TC_BT_Event_Init_Failed); return;}
					} else return;
					break;
				
				default: return; //it should be impossible
			}
		}
		init_seq_current++;
		if (init_seq_current > Seq_MNG_Set_Scan) { //last command sent
			tc_driver_state = TC_BT_Idle;
			tc_bt_event_handler(TC_BT_Event_Initialized);
			return;
		}
	} else
		return;
	
	bt_delay_ms(150);
	
	//send next command
	lsend = (init_seq_current < 4) ? Init_HCI_Length[init_seq_current]
	                               : Init_Req[init_seq_current][0];
	tc_uart_send_bytes(Init_Req[init_seq_current], lsend);
}

void step_connect(void)
{
	TCU_Command* tcu_resp = (TCU_Command*) pbuff; uint8_t lsend;
	
	if (tc_driver_state == TC_BT_Idle) {
		if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x55) { //TCU_MNG_Connection_Request_Event			
			bool pair = false;
			TCU_MNG_Connection_Request_Event* creq = (TCU_MNG_Connection_Request_Event*)(&(tcu_resp->param));
		
			if (tc_bt_conf.link_stored) { //stored link exists
				if (array_equal(creq->remote_address, tc_bt_conf.valid_link.remote_addr, 6)) {
					tc_driver_state = TC_BT_Connecting;
					connect_seq_current = Seq_MNG_Connection_Accept2;
					modify_commands();
				} else {
					if (tc_bt_conf.link_fixed) {
						connect_seq_current = Seq_Reject;
						memcpy(Connect_Req[Seq_Reject] + 8, creq->remote_address, 6);
						memcpy(rejected_remote_addr, creq->remote_address, 6);
					} else
						pair = true;
				}
			} else
				pair = true;
			
			if (pair) {
				tc_driver_state = TC_BT_Pairing;
				connect_seq_current = Seq_MNG_Connection_Accept1;
				memcpy(tc_bt_conf.valid_link.remote_addr, creq->remote_address, 6);
				modify_commands();
			}
		} else return;
	} else if (tc_driver_state == TC_BT_Pairing) {
		if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x7D //TCU_MNG_SSP_Info_Event
		&&  tcu_resp->param[0] == 0x31) //HCI_IO_Capability_Request_Event
			connect_seq_current = Seq_MNG_HCI_Set1;
		else if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x7D //TCU_MNG_SSP_Info_Event
		     &&  tcu_resp->param[0] == 0x33) //HCI_User_Confirmation_Request_Event
			connect_seq_current = Seq_MNG_HCI_Set2;
		else if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x47) { //TCU_MNG_Connection_Status_Event
			TCU_MNG_Connection_Status_Event* st = (TCU_MNG_Connection_Status_Event*)(&(tcu_resp->param));
			if (st->connection_status == Connection_Status_Link_Key) {
				memcpy(tc_bt_conf.valid_link.link_key, st->link_key, 16);
				tc_bt_conf.link_stored = true;
				tc_driver_state = TC_BT_Idle;
				tc_bt_event_handler(TC_BT_Event_Paired);
				return;
			} else if (st->status != Connection_Status_Successful
			        || st->connection_status == Connection_Status_Disconnected
			        || st->connection_status == Connection_Status_Connection_Failure) {
				tc_driver_state = TC_BT_Idle;
				tc_bt_event_handler(TC_BT_Event_Pair_Failed);
			} else return;
		} else return;
	} else if (tc_driver_state == TC_BT_Connecting) {
		if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x55) { //TCU_MNG_Connection_Accept_Resp
			if (tcu_resp->param[0] != 0x00) //failed
				{tc_driver_state = TC_BT_Idle; tc_bt_event_handler(TC_BT_Event_Connect_Failed); return;}
			else return; //wait for TCU_MNG_Connection_Status_Event
		} else if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x47) { //TCU_MNG_Connection_Status_Event
			TCU_MNG_Connection_Status_Event* st = (TCU_MNG_Connection_Status_Event*)(&(tcu_resp->param));
			if (st->status == Connection_Status_Successful && st->connection_status == Connection_Status_Connected) {
				tc_driver_state = TC_BT_Connected;
				tc_bt_event_handler(TC_BT_Event_Connect);
			} else {
				if (! tc_bt_conf.link_fixed)
					tc_bt_conf.link_stored = false; //then the remote device should delete current link and pair again
				tc_driver_state = TC_BT_Idle; tc_bt_event_handler(TC_BT_Event_Connect_Failed); return;
			}
			return;
		} else return;
	} else return;
	
	//send next command
	bt_delay_ms(300);
	lsend = Connect_Req[connect_seq_current][0];
	tc_uart_send_bytes(Connect_Req[connect_seq_current], lsend);
	
	if (connect_seq_current == Seq_Reject) {
		tc_driver_state = TC_BT_Idle; //reject command has been sent (done)
		tc_bt_event_handler(TC_BT_Event_Rejected);
	}
}

void spp_handler(void)
{
	TCU_Command* tcu_resp = (TCU_Command*) pbuff;
	
	if (tc_driver_state != TC_BT_Connected && tc_driver_state != TC_BT_Disconnecting) return;
	
	if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x7D
	&&  tcu_resp->param[0] == 0x32) { //TCU_MNG_SSP_Info_Event, HCI_IO_Capability_Response_Event. it's false connection.
		//it should be the same remote device which has discarded its link key, deny this pairing request at first.
		if (! tc_bt_conf.link_fixed)
			tc_bt_conf.link_stored = false; //then the remote device can pair again
		
		bt_delay_ms(150);
		tc_uart_send_bytes(Connect_Req[Seq_Connect_Cancel], Connect_Req[Seq_Connect_Cancel][0]);
		tc_driver_state = TC_BT_Idle;
		tc_bt_event_handler(TC_BT_Event_Connect_Failed);
	}
	
	else if (tcu_resp->service_id == 0xE5 && tcu_resp->op_code == 0x48) { //TCU_SPP_Data_Receive_Event
		uint16_t count = *(uint16_t*)(tcu_resp->param);
		memcpy(tc_bt_spp_buff + tc_bt_spp_rec_count, tcu_resp->param + 2, count);
		tc_bt_spp_rec_count += count;
		tc_bt_event_handler(TC_BT_Event_SPP_Receive);
	}
	else if (tcu_resp->service_id == 0xE5 && tcu_resp->op_code == 0xF1) { //TCU_SPP_Data_Send_Event
		tc_bt_event_handler(TC_BT_Event_SPP_Sent);
	}
	else if (tcu_resp->service_id == 0xE1 && tcu_resp->op_code == 0x47) { //TCU_MNG_Connection_Status_Event
		TCU_MNG_Connection_Status_Event* st = (TCU_MNG_Connection_Status_Event*)(&(tcu_resp->param));
		if (st->status == Connection_Status_Link_Loss
		||  st->connection_status == Connection_Status_Disconnected
		||  st->connection_status == Connection_Status_Connection_Failure) { //the last is possible when cancelling false connection
			tc_driver_state = TC_BT_Idle;
			tc_bt_event_handler(TC_BT_Event_Disconnected);
		}
	} else if (tcu_resp->service_id == 0xE5 && tcu_resp->op_code == 0x44) { //TCU_SPP_Disconnect_Event
		tc_driver_state = TC_BT_Idle;
		tc_bt_event_handler(TC_BT_Event_Disconnected);
	}
}

extern void tc_bt_init(void)
{
	modify_commands();
	
	if (tc_driver_state > TC_BT_Init || init_seq_current >= Seq_HCI_Set_Mode) return;
	
	tc_driver_state = TC_BT_Before_Init;
	init_seq_current = Seq_HCI_Reset;
	step_init();
}

extern void tc_bt_spp_send(uint8_t* bytes, uint16_t count)
{
	uint8_t send[TC_BT_SPP_Single_Block_Max_Size + 3 + 6] = {0x00, 0x00, 0x00, 0xe5, 0x08}; //TCU_SPP_Data_Transfer_Req
	uint8_t* p = bytes; uint16_t count_remain = count;
	
	uint16_t* pl1 = (uint16_t*) send;
	uint16_t* pl2 = (uint16_t*)(send + 3 + 2);
	uint16_t* pl3 = (uint16_t*)(send + 3 + 4);
	uint8_t* pd = (uint8_t*)(send + 3 + 6);
	
	uint8_t i; uint8_t times = count / TC_BT_SPP_Single_Block_Max_Size;
	if (count % TC_BT_SPP_Single_Block_Max_Size != 0) times++;
	
	for (i = 0; i < times; i++) {
		bt_delay_ms(150);
		tc_uart_rec_count = 0;
		
		*pl3 = (count_remain > TC_BT_SPP_Single_Block_Max_Size)? TC_BT_SPP_Single_Block_Max_Size
		                                                       : count_remain;
		*pl2 = 2 + *pl3;
		*pl1 = 3 + 4 + *pl2;
		memcpy(pd, p, *pl3);
		
		tc_uart_send_bytes(send, *pl1);
		
		count_remain -= *pl3;
		p += *pl3;
	}
}

extern void tc_bt_spp_disconnect(void)
{
	if (tc_driver_state != TC_BT_Connected) return;
	tc_driver_state = TC_BT_Disconnecting;
	
	bt_delay_ms(150);
	tc_uart_send_bytes(SPP_Disconnect_Req, SPP_Disconnect_Req[0]);
}

extern void tc_uart_receive_handler(void)
{
	uint16_t lparsed = 0; //length of already parsed data
	uint16_t lrec = 0; //total length of the currently parsing response
	pbuff = tc_uart_buff; //beginning position of the current response
	
	while (true) {
		if (tc_driver_state == TC_BT_Init && init_seq_current < 4) { //HCI
			if (tc_uart_rec_count >= 1 + 2) {
				lrec = (pbuff[1 + 2 - 1] & 0xff) + 1 + 2;
				*((uint8_t*)(&lrec) + 1) = 0x00; //to get rid of a bug
			} else break;
			
			if (tc_uart_rec_count >= lrec)
				step_init();
			else break;
		} else {
			if (tc_uart_rec_count >= 3) {
				if ((uint8_t) pbuff[1] > 2) {
					pbuff++; lparsed++; //it's able to ignore one invalid byte
					continue;
				}
				lrec = (((uint16_t)(pbuff[1])) << 8) + pbuff[0];
			} else break;
			
			if (tc_uart_rec_count >= lrec) { //the whole response is received				
				switch (tc_driver_state) { //these step_xxx are time-costing, new interrupts might come while delay_ms()
					case TC_BT_Init:
						step_init(); break;
					case TC_BT_Idle: case TC_BT_Pairing: case TC_BT_Connecting:
						step_connect(); break;
					case TC_BT_Connected: case TC_BT_Disconnecting:
						spp_handler(); break;
					default: break;
				}
			} else break;
		}
		
		lparsed += lrec; pbuff += lrec;
		if (lparsed >= tc_uart_rec_count) {
			tc_uart_rec_count = 0; return; //the buffer is clean
		}
	}
	
	if (lparsed < tc_uart_rec_count) {
		//make sure that `tc_uart_buff` begins at the next response (currently incomplete) for the next time
		uint16_t i;
		for (i = 0; i < tc_uart_rec_count - lparsed; i++)
			tc_uart_buff[i] = pbuff[i];
	}
	
	tc_uart_rec_count -= lparsed;
}
