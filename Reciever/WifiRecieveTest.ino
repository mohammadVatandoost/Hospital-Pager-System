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
SSD1306  display(0x3C, 0, 2); // SDA D2  SCK D1
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
  initializing();
  // Wifi Connect
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
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  Udp.begin(localUdpPort);
  // Timer
//    timer1_attachInterrupt(onTimerISR);
//    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
//    timer1_write(8000000); 
  //  Enable light sleep
  wifi_set_sleep_type(LIGHT_SLEEP_T);
  delay(1000);
//  wifi_set_sleep_type(MODEM_SLEEP_T);
//    system_deep_sleep(0);
}

void loop()
{
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
    bufRoomNum = "";
    bufBedNum = "";
    bufTime = "";
    for (int i = 0; i < packetSize; i++)
    {
     if(packetBuffer[i] == '-') {
      c++;
     } else {
      if(c == 0)
      {
       bufTime += packetBuffer[i];
      }
      if(c == 1)
      {
       bufRoomNum += packetBuffer[i];
      }
      if(c == 2)
      {
       bufBedNum += packetBuffer[i];
      }
      if(c == 3)
      {
        Time = bufTime ;
        Serial.println(packetBuffer);
        if(packetBuffer[i] == 'T') {
          if(checkNew(bufRoomNum,bufBedNum)) {
            roomNum[stackCounter] = bufRoomNum ;
            bedNum[stackCounter] = bufBedNum ;
            stackCounter++;
            tempIndex = stackCounter ;
            Vibre_on ;Buzzer_on ;timeCounter = 0 ;showOled();
          }
        } 
        if(packetBuffer[i] == 'F') {
          removeAlarmList(bufRoomNum,bufBedNum);
          Vibre_off ;Buzzer_off ;display.clear();display.display();wifi_set_sleep_type(LIGHT_SLEEP_T);delay(100);
        }
      }
     }
    }
  }
  // delay 1 ms
  delay(1);
  
  
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

