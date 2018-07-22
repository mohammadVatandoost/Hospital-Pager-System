extern "C" {
  #include "user_interface.h"
}

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Ticker.h>
//#include "SSD1306.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


#define Vibre_on digitalWrite(5, HIGH) 
#define Vibre_off digitalWrite(5, LOW)

#define Buzzer_on  digitalWrite(16, HIGH)
#define Buzzer_off digitalWrite(16, LOW)

#define Power_on  digitalWrite(14, HIGH)
#define Power_off digitalWrite(14, LOW)

#define read_Button digitalRead(4)

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
//SSD1306  display(0x3C, 0, 2); // SDA D2  SCK D1
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
 uint8_t flagVibre = 0 ;
 uint8_t flagBuzzer = 0 ;
 uint8_t buzzerCount = 0 ;
 uint16_t vibreCount = 0;
 uint8_t connectCount = 0 ;
//=======================================================================
void ICACHE_RAM_ATTR onTimerISR(){
    timeCounter++;
    timer1_write(8000000);
}
void setup()
{
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
//  Wire.beginOnPins(2, 0);//Wire.beginOnPins(SCLpin, SDApin);
  initializing(); turnOnShowStart();
//  Serial.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  delay(1000);
 turnOnShowStop();
  delay(1000);
  WiFi.begin(ssid, password); wifiConnecting();
  while (WiFi.status() != WL_CONNECTED){ if(connectCount > 25) {  clearOLED();display.drawString(20, 25, "Try again");display.display();delay(1000);clearOLED();ESP.reset(); } else {wifiConnecting();delay(500);Serial.print("."); connectCount++; } } clearOLED();display.display();
//  Serial.println(" connected");
//  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  Udp.begin(localUdpPort);
  // Timer
    timer1_attachInterrupt(onTimerISR);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
    timer1_write(8000000); 
   sleepDevice();
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
    int count=0;
    bufRoomNum = "";
    bufBedNum = "";
    bufTime = "";
    for (int i = 0; i < packetSize; i++)
    {
     if(packetBuffer[i] == '-') {
      count++;
     } else {
      if(count == 0)
      {
       bufTime += packetBuffer[i];
      }
      if(count == 1)
      {
       bufRoomNum += packetBuffer[i];
      }
      if(count == 2)
      {
       bufBedNum += packetBuffer[i];
      }
      if(count == 3)
      {
        Time = bufTime ;
//        Serial.println(packetBuffer);
       if( (bufRoomNum != "99") && (bufBedNum != "99") ) {
        if(packetBuffer[i] == 'T') {
          if(checkNew(bufRoomNum,bufBedNum)) {
            roomNum[stackCounter] = bufRoomNum ;
            bedNum[stackCounter] = bufBedNum ;
            stackCounter++;
            tempIndex = stackCounter ;
            flagVibre = 1;flagBuzzer = 1 ;timeCounter = 0 ;showOled();
          }
        } 
        if(packetBuffer[i] == 'F') {
          removeAlarmList(bufRoomNum,bufBedNum);
          sleepDevice();
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
//    Serial.println("read_Button");
    if(buttonCounter < 10) {
     buttonCounter++;  
    } else {
     buttonCounter = 10 ; 
     turnOff();
    }  
  } else {
    if(buttonCounter > 9) {
//        Serial.println("5S >");
        turnOff();
    } else if(buttonCounter > 4) {
//       Serial.println("2S >");
       sleepDevice();
    } else if(buttonCounter > 0) {
//      Serial.println("1S >");
      if( stackCounter > 0 ) { 
      if(tempIndex == 0) {tempIndex = stackCounter;}
      flagVibre = 0;flagBuzzer = 0 ; Vibre_off ;Buzzer_off ;showOled();tempIndex--;timeCounter = 0 ;
     } else {
      flagVibre = 0;flagBuzzer = 0 ;Vibre_off ;Buzzer_off ;timeCounter = 0 ;
      clearOLED();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(30, 0, Time );
      display.drawString(30, 15, "No Alarm" );
      display.display();
     }
    }
    buttonCounter = 0 ;
  }
  // process
// buzzerProcess();
 vibreProcess();
  // 60S
    if( (timeCounter > 29) && (timeCounter < 32) ) {
      sleepDevice();
    }
    if(timeCounter > 200) {
      timeCounter = 40;
    }
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
    if( (roomNum[i] == roomNumTemp) && (bedNum[i] == bedNumTemp) ) {return 0;}
  }
  return 1 ;
}

void showOled() {
    display.clearDisplay();display.display();
    delay(1);
    display.setTextAlignment(TEXT_ALIGN_LEFT);display.setFont(ArialMT_Plain_16);
    display.drawString(35, 0, Time );
    display.drawString(25, 15, "room: " + roomNum[tempIndex-1] );
    display.drawString(25, 30, "bed: " + bedNum[tempIndex-1] );
    display.display();
}

void initializing() {
  Serial.begin(9600);  
  display.init();clearOLED();

  //Buzzer
  pinMode(16, OUTPUT);
  //Vibre
  pinMode(5, OUTPUT);
  //PowerControl
  pinMode(14, OUTPUT);
  //Key
  pinMode(4, INPUT);
 
  Power_on;
//  ESP.wdtDisable();
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


void vibreProcess() { if(flagVibre){ if(vibreCount < 400) {Vibre_on;vibreCount++;} else {Vibre_off;vibreCount++; if(vibreCount > 3000) {vibreCount = 0;} } } else {Vibre_off;} }

void sleepDevice() {flagVibre = 0;flagBuzzer = 0 ;Vibre_off;Buzzer_off;clearOLED();delay(500);}//wifi_set_sleep_type(MODEM_SLEEP_T);
 
void turnOnShowStart() {Vibre_on ;Buzzer_on ; clearOLED();display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);display.setRotation(2);display.drawString(20, 20, companyName);display.display();
      }

void turnOnShowStop() { clearOLED();Vibre_off ;Buzzer_off ;}

void wifiConnecting() {display.drawString(10, 20, "Wifi Connecting");display.display();}

void turnOff() {flagVibre = 0;flagBuzzer = 0 ;Vibre_off ;Buzzer_off ;clearOLED();display.drawString(35, 30, "Turn off");display.display();delay(1000);Power_off;}

void clearOLED() {

    display.clearDisplay();
//    display.clear();display.display();
  }
