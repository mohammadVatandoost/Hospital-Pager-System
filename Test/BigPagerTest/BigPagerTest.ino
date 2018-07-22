extern "C" {
  #include "user_interface.h"
}

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Ticker.h>
#include "SSD1306.h"

// Shift Register
#define GLED 0
#define RLED 1
#define BLED 2
#define A4 3
#define G 4
#define C 5
#define D 6
#define E 7
#define Buzzer 8
#define Vibre 9
#define A1 10
#define B 11
#define A3 12
#define A2 13
#define F 14
#define A 15
// Keys
#define nextTimeKey digitalRead(5)
#define silentPowerKey digitalRead(4)
//PowerControl
#define PowerControl_On digitalWrite(2, HIGH)
#define PowerControl_Off digitalWrite(2, LOW)
//RCLKN
#define RCLKN_On digitalWrite(14, HIGH)
#define RCLKN_Off digitalWrite(14, LOW)
//SRCLKN
#define SRCLKN_On digitalWrite(12, HIGH)
#define SRCLKN_Off digitalWrite(12, LOW) 
//DataIN
#define DataIN_On digitalWrite(13, HIGH)
#define DataIN_Off digitalWrite(13, LOW)  
// Array of ShiftRegister
bool ShiftRegister[16] = { 0 } ;
// counter 7Segment
int counter7Seg = 0 ;
// seven Segment number{1,1,1,1,G,C,D,E,1,1,1,B,1,1,F,A}
bool number[10][16] = {
                       {1,1,1,1,1,0,0,0,1,1,1,0,1,1,0,0}, // 0
                       {1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1}, // 1
                       {1,1,1,1,0,1,0,0,1,1,1,0,1,1,1,0}, // 2
                       {1,1,1,1,0,0,0,1,1,1,1,0,1,1,1,0}, // 3
                       {1,1,1,1,0,0,1,1,1,1,1,0,1,1,0,1}, // 4
                       {1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0}, // 5
                       {1,1,1,1,0,0,0,0,1,1,1,1,1,1,0,0}, // 6
                       {1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0}, // 7
                       {1,1,1,1,0,0,0,0,1,1,1,0,1,1,0,0}, // 8
                       {1,1,1,1,0,0,0,1,1,1,1,0,1,1,0,0} // 9
                      };
bool SevenSegment4[4][16];                      
//number[0] = {{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}} ;

// timer 
Ticker blinker;

const char* ssid = "Tenda_266360";  //MAAD    
const char* password = "Raiwan1234"; //SalamSalam   
uint8_t Time[4];
uint8_t stackCounter = 0;
uint8_t tempIndex = 0 ;
uint8_t roomBedNum[10][4];
uint8_t bufRoomBedNum[4];
uint16_t buttonCounter1 = 0 ;
uint16_t buttonCounter2 = 0 ;
WiFiUDP Udp;
unsigned int localUdpPort = 4211;  // local port to listen on Reciever transmitter
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer for incoming packets
IPAddress IP_Remote(192, 168, 1, 17);
 uint8_t timeCounter = 0;
 uint8_t flagTime = 0 ;
 uint8_t flag7Segment = 0 ;
//=======================================================================
void ICACHE_RAM_ATTR onTimerISR(){
    timeCounter++;
    timer1_write(8000000);//12us
    // 60S
    if(timeCounter == 30) {
      sleepDevice();
    }
    // 20S
    if( (timeCounter == 10) && (flagTime == 1)) {
     sleepDevice();
    }
}
void setup()
{
  initialize();
  shiftRegisterProcess();
  Serial.begin(9600);
  delay(500);
  Serial.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  delay(1000);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
 // Udp.beginMulticast(WiFi.localIP(), IP_Remote, localUdpPort);
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  // Timer
    timer1_attachInterrupt(onTimerISR);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    timer1_write(8000000); //120000 us
  //  Enable light sleep
//  wifi_set_sleep_type(LIGHT_SLEEP_T);
//  wifi_set_sleep_type(MODEM_SLEEP_T);
//    system_deep_sleep(0);
  wifi_set_sleep_type(LIGHT_SLEEP_T);
  delay(1000);
}

void loop()
{
  // for test
  ShiftRegister[Vibre] = 1;
    // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++)
    {
      Serial.print(remote[i], DEC);
      if (i < 3)
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    int c=0;
    for (int i = 0; i < packetSize; i++)
    {
     if(packetBuffer[i] == '-') {
      c++;
     } else {
      if(c == 0)
      {
       Time[0] = charToNum(packetBuffer[i]);
       i++;
       Time[1] = charToNum(packetBuffer[i]);
       i++;
       i++;
       Time[2] = charToNum(packetBuffer[i]);
       i++;
       Time[3] = charToNum(packetBuffer[i]);
      }
      if(c == 1)
      {
       bufRoomBedNum[0] = charToNum(packetBuffer[i]);
       i++;
       bufRoomBedNum[1] = charToNum(packetBuffer[i]);
      }
      if(c == 2)
      {
       bufRoomBedNum[2] = charToNum(packetBuffer[i]);
       i++;
       bufRoomBedNum[3] = charToNum(packetBuffer[i]);
      }
      if(c == 3)
      {
//        Serial.println("Time");
//        Serial.print(Time[0]);Serial.print(Time[1]);Serial.print(Time[2]);Serial.print(Time[3]);
//        Serial.println("roomNum");
//        Serial.print(bufRoomBedNum[0]);Serial.println(bufRoomBedNum[1]);
//        Serial.println("bedNum");
//        Serial.print(bufRoomBedNum[2]);Serial.println(bufRoomBedNum[3]);
        if(packetBuffer[i] == 'T') {
          if(checkNew(bufRoomBedNum[0],bufRoomBedNum[1],bufRoomBedNum[2],bufRoomBedNum[3])) {
               roomBedNum[stackCounter][0] = bufRoomBedNum[0];
               roomBedNum[stackCounter][1] = bufRoomBedNum[1];
               roomBedNum[stackCounter][2] = bufRoomBedNum[2];
               roomBedNum[stackCounter][3] = bufRoomBedNum[3];
               stackCounter++;
               tempIndex = stackCounter ;
               ShiftRegister[Buzzer] = 1 ; 
               ShiftRegister[Vibre] = 1 ; 
               timeCounter = 0 ;
               flag7Segment = 1 ;
               switchToRoomBedNum();
          }
        } 
        if(packetBuffer[i] == 'F') {
          removeAlarmList(bufRoomBedNum[0],bufRoomBedNum[1],bufRoomBedNum[2],bufRoomBedNum[3]);
          sleepDevice();
        }
      }
     }
    }
  }
  // buttons 
  if (!nextTimeKey) {
    buttonCounter1++;
  } else {
    if(buttonCounter1 > 1000) {
      switchToTime(); flagTime = 1 ;timeCounter = 0 ; ShiftRegister[Buzzer] = 0; ShiftRegister[Vibre] = 0;tempIndex = stackCounter;flag7Segment = 1 ;
    } else if(buttonCounter1 > 20) {
        if( stackCounter > 0 ) { 
           if(tempIndex == 0) {tempIndex = stackCounter;}
           switchToRoomBedNum();tempIndex--;timeCounter = 0 ; ShiftRegister[Buzzer] = 0; ShiftRegister[Vibre] = 0;flagTime = 0 ;flag7Segment = 1 ;
        } else {
           flag7Segment = 0;timeCounter = 0;ShiftRegister[Buzzer] = 0; ShiftRegister[Vibre] = 0;flagTime = 1 ;// ledOn
        }
    } 
    buttonCounter1 = 0 ;
  }
  
  if (silentPowerKey) {
    buttonCounter2++;
  } else {
    if(buttonCounter2 > 1000) {
      // turn off the system
    } else if(buttonCounter2 > 20) {
        sleepDevice();
    } 
    buttonCounter2 = 0 ;
  }
   
  // proccess
  process7Segment() ;
  shiftRegisterProcess();
}

void removeAlarmList(uint8_t roomNum1,uint8_t roomNum2,uint8_t bedNum1,uint8_t bedNum2) {
  for(uint8_t i=0;i<stackCounter;i++)
  {
    if( (roomBedNum[i][0] == roomNum1) && (roomBedNum[i][1] == roomNum2) && (roomBedNum[i][2] == bedNum1) && (roomBedNum[i][3] == bedNum2) ) {
      for(uint8_t j=i;j<stackCounter;j++) {
        if( (j+1) ==  stackCounter ) {
          stackCounter--;
          tempIndex = stackCounter ;
        } else {
          roomBedNum[j][0] = roomBedNum[j+1][0];
          roomBedNum[j][1] = roomBedNum[j+1][1];
          roomBedNum[j][2] = roomBedNum[j+1][2];
          roomBedNum[j][3] = roomBedNum[j+1][3];
        }
      }
    }
  }
}

void initialize()
{
  //PowerControl
  pinMode(2, OUTPUT);
  //PowerKey
  pinMode(4, INPUT);
  //nextSilentKey
  pinMode(5, INPUT_PULLUP);
  //RCLKN
  pinMode(14, OUTPUT);
  //SRCLKN
  pinMode(12, OUTPUT);
  //DataIN
  pinMode(13, OUTPUT);

  SRCLKN_Off;
  RCLKN_Off;
  DataIN_Off;
}

uint8_t checkNew(uint8_t roomNum1,uint8_t roomNum2,uint8_t bedNum1,uint8_t bedNum2) {
  for(uint8_t i=0;i<stackCounter;i++)
  {
    if( (roomBedNum[i][0] == roomNum1) && (roomBedNum[i][1] == roomNum2) && (roomBedNum[i][2] == bedNum1) && (roomBedNum[i][3] == bedNum2) ) {
      return 0;
    }
  }
  return 1 ;
}

void switchToTime() {
  for(uint8_t i=0;i<4;i++) {
    set7Segment(Time[i],i);  
  }
}

void switchToRoomBedNum() {
  for(uint8_t i=0;i<4;i++) {
    set7Segment((roomBedNum[tempIndex-1][i]),i);  
  }
}
void set7Segment(uint8_t num,uint8_t which) {
  for(int i=0;i<16;i++) {
    SevenSegment4[which][i] = number[num][i];  
  }
}

void process7Segment() {
  // CA 7Segment
  ShiftRegister[A1] = 1;
  ShiftRegister[A2] = 1;
  ShiftRegister[A3] = 1;
  ShiftRegister[A4] = 1;
  if(counter7Seg > 3) {
    counter7Seg = 0 ;
  }
 if(flag7Segment == 1) {  
   // seven Segment number{1,1,1,1,G,C,D,E,1,1,1,B,1,1,F,A}
   ShiftRegister[4]=  SevenSegment4[counter7Seg][4];
   ShiftRegister[5]=  SevenSegment4[counter7Seg][5];
   ShiftRegister[6]=  SevenSegment4[counter7Seg][6];
   ShiftRegister[7]=  SevenSegment4[counter7Seg][7];
   ShiftRegister[11]=  SevenSegment4[counter7Seg][11];
   ShiftRegister[14]=  SevenSegment4[counter7Seg][14];
   ShiftRegister[15]=  SevenSegment4[counter7Seg][15];
 
  if(counter7Seg == 0 ) {ShiftRegister[A1] = 0;}
  if(counter7Seg == 1 ) {ShiftRegister[A2] = 0;}
  if(counter7Seg == 2 ) {ShiftRegister[A3] = 0;}
  if(counter7Seg == 3 ) {ShiftRegister[A4] = 0;}
   counter7Seg++;
 }
}

void shiftRegisterProcess() {
  for(int i=0;i<16;i++) {
    if(ShiftRegister[i] == 0) {
      DataIN_Off;
    } else {
      DataIN_On;
    }
    SRCLKN_On;
    delayMicroseconds(100);
    SRCLKN_Off;
    delayMicroseconds(100);
  }
   RCLKN_Off;
   delayMicroseconds(100);
   RCLKN_On;
   delayMicroseconds(100);
   RCLKN_Off;
}

uint8_t charToNum(char temp) {
   return temp-'0';
}

void sleepDevice() {
 ShiftRegister[GLED] = 0;ShiftRegister[RLED] = 0;ShiftRegister[BLED] = 0;ShiftRegister[Buzzer] = 0; ShiftRegister[Vibre] = 0;flagTime = 0 ;flag7Segment = 0;
 tempIndex = stackCounter;wifi_set_sleep_type(LIGHT_SLEEP_T);//delay(100);
}

