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

#define Vibre_on digitalWrite(5, HIGH) 
#define Vibre_off digitalWrite(5, LOW) 

#define Buzzer_on  digitalWrite(16, HIGH)
#define Buzzer_off digitalWrite(16, LOW)

#define Power_on  digitalWrite(14, HIGH)
#define Power_off digitalWrite(14, LOW)

#define read_Button digitalRead(4)
// timer 
Ticker blinker;

// modem data
const char* ssid = "Mohammad";  //MAAD    
const char* password = "13741374"; //SalamSalam   
// display Data
SSD1306  display(0x3C, 4, 5); // SDA D2  SCK D1
String Time = "";
String bufTime = "";
String bufRoomNum = "";
String bufBedNum = "";
String roomNum[10] = {""};
String bedNum[10] = {""}; 
uint8_t stackCounter = 0;
uint8_t tempIndex = 0 ;
uint16_t buttonCounter = 0 ;
WiFiUDP Udp;
unsigned int localUdpPort = 4211;  // local port to listen on Reciever transmitter
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer for incoming packets
IPAddress IP_Remote(192, 168, 1, 17);
 int timeCounter = 0;
//=======================================================================
void ICACHE_RAM_ATTR onTimerISR(){
    timeCounter++;
    timer1_write(8000000);
    if(timeCounter == 30) {
      Vibre_off;Buzzer_off;display.clear();display.display();wifi_set_sleep_type(LIGHT_SLEEP_T);delay(100);
    }
    Serial.println("On");
}
void setup()
{
  display.init();display.clear();display.display();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);display.setFont(ArialMT_Plain_16);
}

void loop()
{
 display.drawString(63, 31, "Wifi Connecting");display.display();
 delay(1000);
}

void removeAlarmList(String roomNumTemp,String bedNumTemp) {
  for(uint8_t i=0;i<stackCounter;i++)
  {
    if( (roomNum[i] == roomNumTemp) && (bedNum[i] == bedNumTemp) ) {
      for(uint8_t j=i;j<stackCounter;j++) {
        if( (j+1) ==  stackCounter ) {
          stackCounter--;
          tempIndex = stackCounter ;
        } else {
          roomNum[j] = roomNum[j+1];
          bedNum[j] = bedNum[j+1];
        }
      }
    }
  }
}
uint8_t checkNew(String roomNumTemp,String bedNumTemp) {
  for(uint8_t i=0;i<stackCounter;i++)
  {
    if( (roomNum[i] == roomNumTemp) && (bedNum[i] == bedNumTemp) ) {
      return 0;
    }
  }
  return 1 ;
}

void showOled() {
    display.clear();
    display.display();
    delay(1);
    Serial.println("showOled");
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, Time );
    display.drawString(0, 10, "room: " + roomNum[tempIndex-1] );
    display.drawString(0, 20, "bed: " + bedNum[tempIndex-1] );
    display.display();
}

void initializing() {
  Serial.begin(9600);
  
  display.init();
  display.clear();
  display.display();
//  display.setTextAlignment(TEXT_ALIGN_LEFT);
//  display.setFont(ArialMT_Plain_10);
//  display.drawString(0, 0,"Hello World");
//  display.display();
  //Buzzer
  pinMode(16, OUTPUT);
  //Vibre
  pinMode(5, OUTPUT);
  //PowerControl
  pinMode(14, OUTPUT);
  //Key
  pinMode(4, INPUT);

  Power_on;
}

