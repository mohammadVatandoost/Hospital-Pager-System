#ifndef SLAVE1_H
#define SLAVE1_H

//includes
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

//defines

#define INDEXMAX	5

#define Packet_Length 10
#define START_BYTE0 0xA5
#define START_BYTE1 0x5A
#define START_BYTE2 0x00
#define START_BYTE3 0xFF
#define STOP_BYTE 0x80
#define Buffer_Size Packet_Length

#define Date_Per_100ms		800

// specific for this slave
#define Slave_Uart			huart1
#define Timeout_Timer		htim2

#define RS485_GPIO_PORT	RS485_ENABLE_GPIO_Port
#define RS485_GPIO_PIN	RS485_ENABLE_GPIO_Pin

#define Enable_RS485_Line			HAL_GPIO_WritePin(RS485_ENABLE_GPIO_Port,RS485_ENABLE_Pin,GPIO_PIN_SET)
#define Disable_RS485_Line		HAL_GPIO_WritePin(RS485_ENABLE_GPIO_Port,RS485_ENABLE_Pin,GPIO_PIN_RESET)

#define GET_ID(A,B)				(A<<4)+B
#define GET_ROOM(A)				A>>4
#define GET_USR_ROOM(A)			A&0x0F

// state decode
#define read_call(A)		(A&0x80)==0x80
#define read_req(A)			(A&0x40)==0x40
#define read_lamp(A)		(A&0x20)==0x20
#define read_slamp(A)		(A&0x10)==0x10
#define read_slave1(A)	(A&0x08)==0x08
#define read_user(A)		(A&0x03)

#define set_call(A,B)			A=((A&0x7F)|((uint8_t)(B<<7)))
#define set_req(A,B)			A=((A&0xBF)|((uint8_t)(B<<6)))
#define set_lamp(A,B)			A=((A&0xDF)|((uint8_t)(B<<5)))
#define set_slamp(A,B)		A=((A&0xEF)|((uint8_t)(B<<4)))
#define set_slave1(A,B)		A=((A&0xF7)|((uint8_t)(B<<3)))
#define set_user(A,B)			A=((A&0xFC)|((uint8_t)(B)))


#define	LED_NORMAL_PERIOD		20
#define	LED_FAST_PERIOD		3

//Types

// state of packet enumeration
typedef enum {
	VIP=0x3,
	SLAVE2=0x0,
	SLAVE1_WC=0x1,
	SLAVE1_DOOR=0x2
}USERS;

typedef enum{
	MASTER_HELLO,
	MASTER_REQ_AUDIO,
	MASTER_AUDIO_ALL,
	MASTER_REGISTER,
	
	SLAVE1_HELLO,
	SLAVE1_REGISTER,
	
	SLAVE2_HELLO,
	SLAVE2_ACK_AUDIO,
	SLAVE2_REGISTER
}FUNCTION;

typedef enum {
	PCK_WAIT=0,
	PCK_Unknown =1,
	PCK_Without_Me=2 ,
	PCK_With_Me=3 ,
	PCK_REGISTERY
}PCK_STATE;

//conversation packet struct and union
typedef union
{
	struct DATA_CONV
  {
    uint8_t ST0;
    uint8_t ST1;
    uint8_t addr;
    uint8_t func;
		uint8_t State;
		uint8_t Sensor1;
		uint8_t Sensor2;
		uint8_t Sensor3;
		uint8_t cksum;
		uint8_t stp;
  }DIST_PCK;															// Distributed packet
	
  uint8_t ASS_PCK[Packet_Length];					// Associated  packet
}PCK_CONV;

typedef enum{
	FLAG_ENABLE=1,
	FLAG_DISABLE=0,
}FLAG;

typedef enum{
	LED_ON,
	LED_OFF,
	LED_TOGGLE_NORM,
	LED_TOGGLE_FAST
}LED_STATES;

typedef struct{
	uint8_t counter;
	LED_STATES state;
}LED_HANDLER;

typedef struct{
	uint8_t client[INDEXMAX];
	uint8_t index;	
}list;


// Variables
extern USERS user_type;
extern uint8_t ID;
extern PCK_CONV Send_pck;
extern PCK_CONV Received_pck;
extern PCK_STATE pck_state;
extern LED_HANDLER led_green;
extern LED_HANDLER led_red;
extern uint8_t buff;
extern list missed_list;
extern FLAG called_flag;
extern FLAG calling_flag;
extern FLAG wait_flag;
extern uint8_t counter_1s;
extern FLAG registered_flag;
	
// Functions
void Registery(void);
void Slave_Init(uint8_t room,uint8_t usr_room);
PCK_STATE Check_PCK(void);
PCK_STATE GetNewData(uint8_t data);
void Init_PCK(FUNCTION f,uint8_t data1,uint8_t data2,uint8_t data3);
HAL_StatusTypeDef Send_PCK(FUNCTION f, uint8_t data1,uint8_t data2,uint8_t data3);

int add2list(list * mylist,uint8_t id);
int removefromlist(list * mylist,uint8_t id);
int findfromlist(list * mylist,uint8_t id);
void flushlist(list * mylist);

#endif

