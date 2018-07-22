#include "Slave1.h"

//variables
USERS user_type;
uint8_t ID;
PCK_CONV Send_pck;
PCK_CONV Received_pck;
uint8_t PCK_RCV;
PCK_STATE pck_state;
LED_HANDLER led_green;
LED_HANDLER led_red;
uint8_t buff;
list missed_list;
FLAG called_flag;
FLAG calling_flag;
FLAG wait_flag;
uint8_t counter_1s;
FLAG registered_flag;

// functions

void Registery(void){
	HAL_TIM_Base_Start_IT(&htim1);
	
	registered_flag=FLAG_DISABLE;
	HAL_UART_Receive_IT(&Slave_Uart,&buff,1);
	
	led_green.state=LED_TOGGLE_FAST;
	led_red.state=LED_TOGGLE_FAST;
	
	user_type=SLAVE1_DOOR;
	pck_state=PCK_WAIT;
	Init_PCK(SLAVE1_REGISTER,0,0,0);
	
	while(registered_flag==FLAG_DISABLE)HAL_Delay(1);
	HAL_Delay(500);
}
	
void Slave_Init(uint8_t room,uint8_t usr_room){
	
	if(usr_room==7)user_type=SLAVE1_WC;
	else if(usr_room==0)user_type=SLAVE1_DOOR;
	
	registered_flag=FLAG_ENABLE;
	missed_list.index=0;
	called_flag=FLAG_DISABLE;
	calling_flag=FLAG_DISABLE;
	wait_flag=FLAG_DISABLE;
	counter_1s=0;
	
	ID=(room<<4)+usr_room;
	
	set_call(Send_pck.DIST_PCK.State,0);
	set_req(Send_pck.DIST_PCK.State,0);
	set_slamp(Send_pck.DIST_PCK.State,0);
	set_lamp(Send_pck.DIST_PCK.State,0);
	set_slave1(Send_pck.DIST_PCK.State,0);
	set_user(Send_pck.DIST_PCK.State,user_type);
	
	
	pck_state=PCK_WAIT;
	Init_PCK(SLAVE1_HELLO,0,0,0);
	PCK_RCV=0;
	
	led_red.counter=0;
	led_red.state=LED_OFF;
	
	led_green.counter=0;
	led_green.state=LED_OFF;
	
	HAL_TIM_Base_Start_IT(&htim1);
	
	HAL_UART_Abort(&Slave_Uart);
	HAL_UART_Receive_IT(&Slave_Uart,&buff,1);
}

PCK_STATE GetNewData(uint8_t data){
	
	switch(PCK_RCV)
	{
	
	case 0:
		if (data == START_BYTE0)
		{
			PCK_RCV++;
			Received_pck.ASS_PCK[PCK_RCV]=data;
		}
		else {
			PCK_RCV=0;
			return PCK_Unknown;
		}
		break;
	
	case 1:
		if (data == START_BYTE1)
		{
			PCK_RCV++;
			Received_pck.ASS_PCK[PCK_RCV]=data;
		}
		else
		{
			PCK_RCV = 0;
			return PCK_Unknown;
		}
		break;
	
	case 2:
		Received_pck.ASS_PCK[PCK_RCV] = data;
		Received_pck.DIST_PCK.cksum = data;
		PCK_RCV++;
		break;
	
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
		Received_pck.ASS_PCK[PCK_RCV] = data;
		Received_pck.DIST_PCK.cksum+= data;
		PCK_RCV++;
		break;
		
	case 8:
		if((Received_pck.DIST_PCK.cksum-data) == 0) PCK_RCV++;
		else {
			PCK_RCV = 0;
			return PCK_Unknown;
		}
		break;
	
	case 9:
		PCK_RCV=0;
		if (data == STOP_BYTE) return Check_PCK();
		return PCK_Unknown;
	}
	return PCK_WAIT;
}

//change state by order of master
PCK_STATE Check_PCK(void){
	if(Received_pck.DIST_PCK.func==MASTER_HELLO)htim1.Instance->CNT=Received_pck.DIST_PCK.Sensor1;

	if(Received_pck.DIST_PCK.func==MASTER_REGISTER && Received_pck.DIST_PCK.addr==0xFF && registered_flag==FLAG_DISABLE) return PCK_REGISTERY;
	else if(Received_pck.DIST_PCK.addr==ID){
		return PCK_With_Me;
	}
	else if(GET_ROOM(Received_pck.DIST_PCK.addr)==GET_ROOM(ID)){
		if(user_type==SLAVE1_DOOR){
			if(read_req(Received_pck.DIST_PCK.State)==1 && read_call(Received_pck.DIST_PCK.State)==0){
				wait_flag=FLAG_ENABLE;
				add2list(&missed_list,Received_pck.DIST_PCK.addr);
			}
			else if(read_req(Received_pck.DIST_PCK.State)==0 && read_call(Received_pck.DIST_PCK.State)==1){
				calling_flag=FLAG_ENABLE;
				called_flag=FLAG_ENABLE;
				removefromlist(&missed_list,Received_pck.DIST_PCK.addr);
			}
		}
		else if(user_type==SLAVE1_WC){
			if(read_slave1(Received_pck.DIST_PCK.State)==1 && (GET_USR_ROOM(Received_pck.DIST_PCK.addr))==0){
				set_slave1(Send_pck.DIST_PCK.State,0);
				led_red.state=LED_OFF;
			}
		}
	}
	return PCK_Without_Me;
}

void Init_PCK(FUNCTION f,uint8_t data1,uint8_t data2,uint8_t data3){
	Send_pck.DIST_PCK.addr=ID;
	Send_pck.DIST_PCK.func=f;
	Send_pck.DIST_PCK.Sensor1=0;
	Send_pck.DIST_PCK.Sensor2=0;
	Send_pck.DIST_PCK.Sensor3=0;
	Send_pck.DIST_PCK.ST0=START_BYTE0;
	Send_pck.DIST_PCK.ST1=START_BYTE1;
	Send_pck.DIST_PCK.stp=STOP_BYTE;
	
	int a=f+ID+data1+data2+data3+Send_pck.DIST_PCK.State;
	
	Send_pck.DIST_PCK.cksum=a%256;
}

HAL_StatusTypeDef Send_PCK(FUNCTION f,uint8_t data1,uint8_t data2,uint8_t data3){
	Init_PCK(f,data1,data2,data3);

	HAL_UART_Abort(&huart1);
	
	Enable_RS485_Line;
	uint8_t out= HAL_UART_Transmit_DMA(&Slave_Uart,Send_pck.ASS_PCK,Packet_Length);

	return out;
}

int add2list(list * mylist,uint8_t id){
		
	int n=findfromlist(mylist,id);
	
	if(n==-1)
	{
		*(mylist->client+mylist->index)=id;	
		mylist->index++;
	}
	
	return n;
}

int removefromlist(list * mylist,uint8_t id){
	
	int n=findfromlist(mylist,id);
		
	if(n!=-1)
	{
		for(int i=n;i<((mylist->index)-1);i++)
		{
			mylist->client[i]=mylist->client[i+1];
		}
		mylist->index--;
	}
	
	return n;
}

// if there is not this id , -1 returned
int findfromlist(list * mylist,uint8_t id){
	int out=-1;
	
	for(int i=0; i < mylist->index ; i++)
		if(mylist->client[i] == id)out=i;
	
	return out;
}

void flushlist(list * mylist){
	mylist->index=0;
}


