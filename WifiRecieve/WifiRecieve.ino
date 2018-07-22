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
#define minBatteryVoltageAccebtable 868  //   3.5V 
#define batterySeg 38  // (1023 - 868) / 4

#define companyName "Khavar dorr"

//       note, period, &  frequency.
#define  c     3830    // 261 Hz 
#define  d     3400    // 294 Hz 
#define  e     3038    // 329 Hz 
#define  f     2864    // 349 Hz 
#define  g     2550    // 392 Hz 
#define  a     2272    // 440 Hz 
#define  b     2028    // 493 Hz 
#define  C     1912    // 523 Hz 
// Define a special note, 'R', to represent a rest
#define  R     0

uint16_t frequencyMaker = 25 ;

// MELODY and TIMING  =======================================
//  melody[] is an array of notes, accompanied by beats[], 
//  which sets each note's relative length (higher #, longer note) 
int melody[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c,  g, a, C };
int beats[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8 }; 
int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping.

// Set overall tempo
long tempo = 10000; // 10000
// Set length of pause between notes
int pause = 1000;
// Loop variable to increase Rest length
int rest_count = 100; //<-BLETCHEROUS HACK; See NOTES

// Initialize core variables
int tone_ = 0;
int beat = 0;
long duration  = 0;

// timer 
Ticker blinker;

// modem data
const char* ssid = "Tenda_266360";  //MAAD    
const char* password = "Raiwan1234"; //SalamSalam   
// display Data
SSD1306  display(0x3C, 0, 2); // SDA D2  SCK D1
String Time = "";
String bufTime = "";
String bufRoomNum = "";
String bufBedNum = "";
String bufBatteryVoltage = "";
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
 uint8_t flagVibre = 0 ;
 uint8_t flagBuzzer = 0 ;
 uint8_t flagWifi = 0 ;uint16_t counterWifiTest = 0 ;
 uint8_t buzzerCount = 0 ;
 uint16_t vibreCount = 0;
 uint8_t connectCount = 0 ;
 int batteryVoltage = 0;
//=======================================================================
void ICACHE_RAM_ATTR onTimerISR(){
    timeCounter++;
    timer1_write(8000000);
    checkBatteryVoltage();
}
void setup()
{
  initializing(); turnOnShowStart();delay(1000);turnOnShowStop();delay(1000);
  while(flagWifi == 0) {
    WiFi.mode(WIFI_AP);delay(100);WiFi.begin(ssid, password); wifiConnecting();delay(100);connectCount= 0;
    while (WiFi.status() != WL_CONNECTED){ 
      if(connectCount > 25) {  flagWifi = 0 ;WiFi.mode(WIFI_OFF);break; }  // display.clear();display.drawString(63, 31, "Try again");display.display();delay(1000);display.clear();display.display();ESP.reset();
      else {wifiConnecting();delay(500);Serial.print("."); connectCount++; flagWifi = 1 ; } 
    } 
    display.clear();display.display();
    if(flagWifi == 0) { for(counterWifiTest=0; counterWifiTest<2000; counterWifiTest++) { delay(250);buttonCheck();delay(250);buttonCheck(); } }
  }
  Udp.begin(localUdpPort);
  // Timer
    timer1_attachInterrupt(onTimerISR);timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);timer1_write(8000000); sleepDevice();
}

void loop()
{
 // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    int count=0;
    bufRoomNum = "";
    bufBedNum = "";
    bufTime = "";
    for (int i = 0; i < packetSize; i++)
    {
     if(packetBuffer[i] == '-') {
      count++;
     } else {
      if(count == 0){bufTime += packetBuffer[i];}
      if(count == 1){bufRoomNum += packetBuffer[i];}
      if(count == 2){bufBedNum += packetBuffer[i];}
      if(count == 3){  Time = bufTime ;
       if( (bufRoomNum != "99") && (bufBedNum != "99") ) {
        if(packetBuffer[i] == 'T') {
          if(checkNew(bufRoomNum,bufBedNum)) {
            roomNum[stackCounter] = bufRoomNum ;bedNum[stackCounter] = bufBedNum ;stackCounter++;tempIndex = stackCounter ;flagVibre = 1;flagBuzzer = 1 ;timeCounter = 0 ;showOled();
          }
        } 
        if(packetBuffer[i] == 'F') {
          removeAlarmList(bufRoomNum,bufBedNum);sleepDevice();
        }
       }
      }
     }
    }
  }
  // delay 500 ms
  if( flagBuzzer == 0 ) {delay(500);}
  delay(1);
  
  // button
  if (read_Button) {
    if( ((buttonCounter < 10) && (flagBuzzer == 0)) || ((buttonCounter < 5000) && (flagBuzzer == 1)) ) {
     buttonCounter++;  
    } else {
     buttonCounter = 5001 ; turnOff();
    }  
  } else {
    if( ((buttonCounter > 9) && (flagBuzzer == 0)) || ((buttonCounter > 5000) && (flagBuzzer == 1))) {
        turnOff();
    } else if(((buttonCounter > 4) && (flagBuzzer == 0)) || ((buttonCounter > 2000) && (flagBuzzer == 1))) {
       sleepDevice();
    } else if(((buttonCounter > 0) && (flagBuzzer == 0)) || ((buttonCounter > 100) && (flagBuzzer == 1))) {
      if( stackCounter > 0 ) { 
      if(tempIndex == 0) {tempIndex = stackCounter;}
      flagVibre = 0;flagBuzzer = 0 ; Vibre_off ;Buzzer_off ;showOled();tempIndex--;timeCounter = 0 ;
     } else {
      flagVibre = 0;flagBuzzer = 0 ;Vibre_off ;Buzzer_off ;timeCounter = 0 ;
      display.clear();display.setTextAlignment(TEXT_ALIGN_CENTER);display.setFont(ArialMT_Plain_16);
      showBattery();
      display.drawString(63, 0, Time );display.drawString(63, 15, "No Alarm" );display.display();
     }
    }
    buttonCounter = 0 ;
  }
  // process
// buzzerProcess();
 vibreProcess();
  // 60S
    if( (timeCounter > 29) && (timeCounter < 32) ) {sleepDevice();}
    if( ((timeCounter%10)==0) && (flagWifi == 0) ) { wifiTry(); }
    if(timeCounter > 200) {timeCounter = 40;}
}

void buttonCheck() {
   // button
  if (read_Button) {
    if( buttonCounter < 20 ) {
     buttonCounter++;  
    } else {
     buttonCounter = 21 ; turnOff();
    }  
  } else {
    if( buttonCounter > 18) {
        turnOff();
    } else if( buttonCounter > 0 ) {
       counterWifiTest = 2000;
    }
    buttonCounter = 0 ;
  }
}

void wifiTry() { WiFi.mode(WIFI_AP); WiFi.begin(ssid, password); wifiConnecting();connectCount= 0;
  while (WiFi.status() != WL_CONNECTED){ 
    if(connectCount > 25) {  flagWifi = 0 ;WiFi.mode(WIFI_OFF);break; }  // display.clear();display.drawString(63, 31, "Try again");display.display();delay(1000);display.clear();display.display();ESP.reset();
    else {wifiConnecting();delay(500);Serial.print("."); connectCount++; flagWifi = 1 ; } } 
    display.clear();display.display();
}

void removeAlarmList(String roomNumTemp,String bedNumTemp) {
  for(uint8_t i=0;i<stackCounter;i++)
  {
    if( (roomNum[i] == roomNumTemp) && (bedNum[i] == bedNumTemp) ) {
      for(uint8_t j=i;j<stackCounter;j++) {
        if( (j+1) ==  stackCounter ) {
          stackCounter--;tempIndex = stackCounter ;
        } else {
          roomNum[j] = roomNum[j+1];bedNum[j] = bedNum[j+1];
        }
      }
    }
  }
}
uint8_t checkNew(String roomNumTemp,String bedNumTemp) {
  for(uint8_t i=0;i<stackCounter;i++){ if( (roomNum[i] == roomNumTemp) && (bedNum[i] == bedNumTemp) ) {return 0;} }
  return 1 ;
}

void showOled() {
    display.clear();display.display();delay(1);display.setTextAlignment(TEXT_ALIGN_CENTER);display.setFont(ArialMT_Plain_16);
    showBattery();display.drawString(63, 0, Time );display.drawString(63, 15, "room: " + roomNum[tempIndex-1] );
    display.drawString(63, 30, "bed: " + bedNum[tempIndex-1] );display.display();
}

void initializing() {
  Serial.begin(9600);  display.init();display.clear();display.display();display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);display.setFont(ArialMT_Plain_16); 
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

// PLAY TONE  ==============================================
// Pulse the speaker to play a tone for a particular duration
void playTone() {
  long elapsed_time = 0;
  if (tone_ > 0) { // if this isn't a Rest beat, while the tone has 
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration) {
      Buzzer_on;
      delayMicroseconds(tone_ / 2);
      // DOWN
      Buzzer_off;
      delayMicroseconds(tone_ / 2);
      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else { // Rest beat; loop times delay
    
//    Serial.println(rest_count);
//    for (int j = 0; j < rest_count; j++) { // See NOTE on rest_count
//      delayMicroseconds(duration);  
//       ESP.wdtFeed();
   // }                                
  }                                 
}

void buzzerProcess() { 
  if(flagBuzzer){ 
    Buzzer_on;delayMicroseconds(frequencyMaker);Buzzer_off;delayMicroseconds(frequencyMaker);
     tone_ = melody[buzzerCount];
     beat = beats[buzzerCount];
     duration = beat * tempo; // Set up timing
     playTone(); 
     // A pause between notes...
     delayMicroseconds(pause);
      buzzerCount++;
      if( buzzerCount == 12 ) {
        buzzerCount = 0 ;
      }
   
  } else {Buzzer_off;} 
}

void checkBatteryVoltage() { batteryVoltage = analogRead(A0) - minBatteryVoltageAccebtable ;Serial.print(batteryVoltage);  }//*(0.0684)

void showBattery() {display.drawLine(0, 0, 0, 11);display.drawLine(0, 0, 15, 0);display.drawLine(15, 0, 15, 1);display.drawLine(15, 1, 20, 1);display.drawLine(20, 1, 20, 10);
                    display.drawLine(0, 11, 15, 11);display.drawLine(15, 10, 15, 11);display.drawLine(15, 10, 20, 10);
                    display.fillRect(2, 2, 3, 8); 
                    if(batteryVoltage > (batterySeg*2) ) { display.fillRect(7, 2, 3, 8); }
                    if(batteryVoltage > (batterySeg*3) ) { display.fillRect(12, 2, 3, 8); }
                    if(batteryVoltage > (batterySeg*4) ) { display.fillRect(16, 4, 3, 4); }
                   }// fillRect(int16_t x, int16_t y, int16_t width, int16_t height); drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);

void vibreProcess() { if(flagVibre){ if(vibreCount < 400) {Vibre_on;vibreCount++;} else {Vibre_off;vibreCount++; if(vibreCount > 3000) {vibreCount = 0;} } } else {Vibre_off;} }

void sleepDevice() {flagVibre = 0;flagBuzzer = 0 ;Vibre_off;Buzzer_off;display.clear();display.display();delay(500);}//wifi_set_sleep_type(MODEM_SLEEP_T);
 
void turnOnShowStart() {Vibre_on ;Buzzer_on ; display.clear();display.display();display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);display.drawString(63, 31, companyName);display.display();
      }

void turnOnShowStop() { display.clear();display.display();Vibre_off ;Buzzer_off ;}

void wifiConnecting() {display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);display.drawString(63, 31, "Wifi Connecting");display.display();}

void turnOff() {flagVibre = 0;flagBuzzer = 0 ;Vibre_off ;Buzzer_off ;display.clear();display.display();display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);display.drawString(63, 31, "Turn off");display.display();delay(1000);Power_off;}
