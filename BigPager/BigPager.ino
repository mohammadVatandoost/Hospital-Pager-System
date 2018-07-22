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
#define RLED 1
#define GLED 0
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

#define minBatteryVoltageAccebtable 920  //   3.5V  868
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
// timer 
Ticker blinker;

const char* ssid = "Tenda_266360";  //MAAD    
const char* password = "Raiwan1234"; //SalamSalam   
uint8_t Time[4];
uint8_t stackCounter = 0;
uint8_t tempIndex = 0 ;
uint8_t roomBedNum[10][4];
uint8_t bufRoomBedNum[4];
int batteryVoltage = 0;
uint16_t buttonCounter1 = 0 ;
uint16_t buttonCounter2 = 0 ;
WiFiUDP Udp;
unsigned int localUdpPort = 4211;  // local port to listen on Reciever transmitter
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer for incoming packets
IPAddress IP_Remote(192, 168, 1, 17);
 uint8_t timeCounter = 0;
 uint8_t flagTime = 0 ;
 uint8_t flag7Segment = 0 ;
 uint8_t flag7Buzzer = 0 ;
 uint16_t flagRedLed = 0 ;  uint16_t flagGreenLed = 0 ;
 uint8_t flagBatteryVoltage = 0 ;
 uint8_t flagVibre = 0 ;
 uint16_t vibreCount = 0 ;
 uint8_t connectCount = 0 ;
  uint8_t flagWifi = 0 ;uint16_t counterWifiTest = 0 ;
//=======================================================================
void ICACHE_RAM_ATTR onTimerISR(){
    timeCounter++;
    timer1_write(8000000);    
    checkBatteryVoltage();
    buzzerProcess();
}

void setup()
{
  initialize(); turnOnShowStart();shiftRegisterProcess();delay(1000);turnOnShowStop();shiftRegisterProcess(); delay(1000);
   while(flagWifi == 0) {
    WiFi.mode(WIFI_AP);delay(100);WiFi.begin(ssid, password); wifiConnecting();delay(100);connectCount= 0;
    while (WiFi.status() != WL_CONNECTED){ 
      if(connectCount > 25) {  flagWifi = 0 ;WiFi.mode(WIFI_OFF);break; }  // ShiftRegister[RLED]=0;shiftRegisterProcess();delay(1500);ESP.reset();
      else {wifiConnecting(); connectCount++; flagWifi = 1 ; } 
    } 
    if(flagWifi == 0) { for(counterWifiTest=0; counterWifiTest<2000; counterWifiTest++) { delay(250);buttonCheck();delay(250);buttonCheck(); } }
  }

  Udp.begin(localUdpPort);
//  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  // Timer
    timer1_attachInterrupt(onTimerISR);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    timer1_write(8000000); 
  sleepDevice();
  delay(1000);
}

void loop()
{
    // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
//    Serial.print("Received packet of size ");
//    Serial.println(packetSize);
//    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++)
    {
//      Serial.print(remote[i], DEC);
      if (i < 3)
      {
//        Serial.print(".");
      }
    }
//    Serial.print(", port ");
//    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
//    Serial.println("Contents:");
//    Serial.println(packetBuffer);
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
       if( (bufRoomBedNum[0] != 9) && (bufRoomBedNum[1] != 9) && (bufRoomBedNum[2] != 9) && (bufRoomBedNum[3] != 9) ) { 
        if(packetBuffer[i] == 'T') {
          if(checkNew(bufRoomBedNum[0],bufRoomBedNum[1],bufRoomBedNum[2],bufRoomBedNum[3])) {
               roomBedNum[stackCounter][0] = bufRoomBedNum[0];
               roomBedNum[stackCounter][1] = bufRoomBedNum[1];
               roomBedNum[stackCounter][2] = bufRoomBedNum[2];
               roomBedNum[stackCounter][3] = bufRoomBedNum[3];
               stackCounter++;
               tempIndex = stackCounter ;
               flag7Buzzer = 1 ; 
               flagVibre = 1 ; 
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
  }
  // buttons 
  if (!nextTimeKey) {
    if(((flag7Segment == 0) && (buttonCounter1 < 8)) || ((flag7Segment == 1) && (buttonCounter1 < 800))) {
      buttonCounter1++;
    } else {
      if(buttonCounter1 < 1001) {
        switchToTime(); flagTime = 1 ;timeCounter = 0 ; ShiftRegister[Buzzer] = 0; flagVibre = 0;tempIndex = stackCounter;flag7Segment = 1 ;Serial.print("nextTimeKey 5s");
        buttonCounter1 = 1001;
      }
    }
  } else {
    if( ((flag7Segment == 0) && (buttonCounter1 > 8)) || ((flag7Segment == 1) && (buttonCounter1 > 800)) ) {
      switchToTime(); flagTime = 1 ;timeCounter = 0 ; ShiftRegister[Buzzer] = 0; flagVibre = 0;tempIndex = stackCounter;flag7Segment = 1 ;Serial.print("nextTimeKey 5s");
    } else if( ((flag7Segment == 0) && (buttonCounter1 > 1)) || ((flag7Segment == 1) && (buttonCounter1 > 20)) ) {
//      Serial.print("nextTimeKey 2s");
        if( stackCounter > 0 ) { 
           if(tempIndex == 0) {tempIndex = stackCounter;}
           switchToRoomBedNum();tempIndex--;timeCounter = 0 ; ShiftRegister[Buzzer] = 0; flagVibre = 0; flagTime = 0 ;flag7Segment = 1 ;
        } else {
           flag7Segment = 0;timeCounter = 0;ShiftRegister[Buzzer] = 0; flagVibre = 0;flagTime = 1 ;// ledOn
        }
    } 
    buttonCounter1 = 0 ;
  }
  
  if (silentPowerKey) {
    Serial.println("silentPowerKey");
    if(((flag7Segment == 0) && (buttonCounter2 < 10)) || ((flag7Segment == 1) && (buttonCounter2 < 1000))) {
      buttonCounter2++;
    } else {
      buttonCounter2 = 1001;
      turnOff();
    }
  } else {
    if( ((flag7Segment == 0) && (buttonCounter2 > 10)) || ((flag7Segment == 1) && (buttonCounter2 > 1000)) ) {
//      Serial.print("silentPowerKey 7s");
      turnOff();
    } else if( ((flag7Segment == 0) && (buttonCounter2 > 1)) || ((flag7Segment == 1) && (buttonCounter2 > 20)) ) {
        sleepDevice();
//        Serial.print("sleepDevice");
    } 
    buttonCounter2 = 0 ;
  }
   
  // proccess
  checkMessageBox();
  vibreProcess();
  process7Segment() ;
  shiftRegisterProcess();
  
  // sleep
 // sleepDevice();
    // 60S
    if( (timeCounter > 29) && (timeCounter < 32) ) {
      sleepDevice();
    }
    // 20S
    if( (timeCounter > 10) && (flagTime == 1)) {
     sleepDevice();
    }
    if(timeCounter > 200) {
      timeCounter = 40;
    }
}

void buttonCheck() {
   // button
  if (silentPowerKey) {
    if( buttonCounter2 < 20 ) {
     buttonCounter2++;  
    } else {
     buttonCounter2 = 21 ; turnOff();
    }  
  } else {
    if( buttonCounter2 > 18) {
        turnOff();
    } else if( buttonCounter2 > 0 ) {
       counterWifiTest = 2000;
    }
    buttonCounter2 = 0 ;
  }
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
  Serial.begin(9600);
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
  //sleepMode
  pinMode(16, OUTPUT);
  
  SRCLKN_Off;RCLKN_Off;DataIN_Off; 
  PowerControl_On;
//  Serial.print("PowerControl_On");
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
  for(uint8_t i=0;i<4;i++) { set7Segment(Time[i],i);  }
}

void switchToRoomBedNum() {
  for(uint8_t i=0;i<4;i++) { set7Segment((roomBedNum[tempIndex-1][i]),i);  }
}
void set7Segment(uint8_t num,uint8_t which) {
  for(int i=0;i<16;i++) { SevenSegment4[which][i] = number[num][i];  }
}

void process7Segment() {
  // CA 7Segment
  ShiftRegister[A1] = 1;
  ShiftRegister[A2] = 1;
  ShiftRegister[A3] = 1;
  ShiftRegister[A4] = 1;
  if(counter7Seg > 3) { counter7Seg = 0 ; }
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
 } else {
  shiftRegisterProcess();
  delay(200);
  batteryVoltage = analogRead(A0) ; //*(0.0684)
  Serial.print(batteryVoltage);
 }
}

void shiftRegisterProcess() {
  for(int i=0;i<16;i++) {
    if(ShiftRegister[i] == 0) { DataIN_Off;} else { DataIN_On; }
    SRCLKN_On;delayMicroseconds(100);SRCLKN_Off;delayMicroseconds(100);
  }
   RCLKN_Off;delayMicroseconds(100);RCLKN_On;delayMicroseconds(100);RCLKN_Off;
}

void checkMessageBox() { if(stackCounter > 0){ if(  ((flagGreenLed < 100)&&(flag7Segment == 1)) || ((flagGreenLed < 3) && (flag7Segment == 0)) ) { ShiftRegister[GLED] = 0 ; flagGreenLed++ ;} else {flagGreenLed++;ShiftRegister[GLED]=1;if(((flagGreenLed > 1100)&&(flag7Segment == 1)) || ((flagGreenLed > 20) && (flag7Segment == 0))) {flagGreenLed = 0;}} } else {ShiftRegister[GLED] = 1;} }

void checkBatteryVoltage() { if(batteryVoltage < minBatteryVoltageAccebtable ){ if(flagBatteryVoltage > 10) { ShiftRegister[RLED] = 0 ; flagBatteryVoltage = 0 ;} else {flagBatteryVoltage++;ShiftRegister[RLED]=1; Serial.println(flagBatteryVoltage);} } else {ShiftRegister[RLED] = 1;} }

void buzzerProcess() { if(flag7Buzzer){ ShiftRegister[Buzzer] = !ShiftRegister[Buzzer] ; } else {ShiftRegister[Buzzer] = 0;} }

void vibreProcess() { if(flagVibre){ if(vibreCount < 200) { ShiftRegister[Vibre] = 1;vibreCount++;} else {ShiftRegister[Vibre] = 0;vibreCount++; if(vibreCount > 1200) {vibreCount = 0;} } } else {ShiftRegister[Vibre] = 0;} }

void turnOnShowStart() { process7Segment() ;flagVibre = 1; ShiftRegister[Vibre] = 1;ShiftRegister[Buzzer] = 1; ShiftRegister[GLED] = 0;ShiftRegister[RLED] = 0;ShiftRegister[BLED] = 0;}

void turnOnShowStop() { ShiftRegister[Vibre] = 0;flagVibre = 0 ; ShiftRegister[Buzzer] = 0; ShiftRegister[GLED] = 1;ShiftRegister[RLED] = 1;ShiftRegister[BLED] = 1;}

void wifiConnecting() {ShiftRegister[BLED] = 0;shiftRegisterProcess(); delay(500);ShiftRegister[BLED] = 1;shiftRegisterProcess(); delay(500); }

void turnOff() {ShiftRegister[GLED] = 0;ShiftRegister[RLED] = 0;ShiftRegister[BLED] = 0;ShiftRegister[Buzzer] = 0; flag7Buzzer = 0 ;flagVibre = 0;ShiftRegister[Vibre] = 0;flag7Segment = 0;shiftRegisterProcess();delay(500);PowerControl_Off;}//

uint8_t charToNum(char temp) { return temp-'0'; } 

void sleepDevice() {
 ShiftRegister[GLED] = 1;ShiftRegister[RLED] = 1;ShiftRegister[BLED] = 1;ShiftRegister[Buzzer] = 0; flag7Buzzer = 0 ; flagVibre = 0 ; ShiftRegister[Vibre] = 0;flagTime = 0 ;flag7Segment = 0;
 tempIndex = stackCounter; process7Segment();shiftRegisterProcess(); wifi_set_sleep_type(MODEM_SLEEP_T); delay(500); //gpio_pin_wakeup_enable(2, GPIO_PIN_INTR_HILEVEL); wifi_set_sleep_type(LIGHT_SLEEP_T); // wifi_set_sleep_type(LIGHT_SLEEP_T);         //ESP.deepSleep(5 * 1000000); system_deep_sleep_instant(5000*1000);//ESP.deepSleep(ESP.deepSleepMax(), WAKE_RF_DEFAULT);//user_func();//    
}

