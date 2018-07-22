#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>

const char* ssid = "Tenda_266360";  //MAAD    
const char* password = "Raiwan1234"; // SalamSalam  
WiFiUDP Udp;
unsigned int localUdpPort = 4211;  // local port to listen on Sender
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer for incoming packets
//IPAddress IP_Remote(192, 168, 1, 15); // 1.15   43.206

IPAddress IP_Remote(192, 168, 1, 255); //19 BigPager 17 Watch

// send data
#define sending 1
#define notSending 0
uint8_t sendFlag = notSending ;
uint8_t sendTimeFlag = notSending ;
char  sendBuffer[] = "10:31-02-03-T\n";
char  sendBuffer2[] = "10:31-99-99-F\n";
uint8_t sendCounter = 0 ;
// Time 
uint8_t timeHour =0;
uint8_t timeMinute = 0;
uint8_t sendTimeCounter = 0; 
uint8_t connectCount = 0 ;
// beds and rooms
uint8_t room[16] = {0} ;
uint8_t bed[16] = {0} ;
// Reza 
#define Packet_Length 10
#define START_BYTE0 0xA5
#define START_BYTE1 0x5A
#define START_BYTE2 0x00
#define START_BYTE3 0xFF
#define STOP_BYTE 0x80
 // state depack
#define read_req(A)      (A&0x40)==0x40

uint8_t PCK_RCV = 0 ;

typedef enum{
  MASTER_HELLO,
  MASTER_REQ_AUDIO,
  MASTER_AUDIO_ALL,
  MASTER_REGISTER,
  
  SLAVE1_HELLO,
  SLAVE1_REGISTER,
  
  SLAVE2_HELLO,
  SLAVE2_ACK_AUDIO,
  SLAVE2_REGISTER,
  BigPager,
}FUNCTION;
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
  }DIST_PCK;                              // Distributed packet
  
  uint8_t ASS_PCK[10];         // Associated  packet
}PCK_CONV;

typedef enum {
  PCK_WAIT=0,
  PCK_Unknown =1,
  PCK_Without_Me=2 ,
  PCK_With_Me=3 ,
  PCK_REGISTERY
}PCK_STATE;

PCK_CONV Received_pck;
//////////////////////////////



void setup()
{
  Serial.begin(500000);
  Serial1.begin(9600);
  delay(100);
  Serial1.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    if(connectCount > 25) {
      ESP.reset();
    }
    connectCount++;
    delay(500);
    Serial1.print(".");
  }
  Serial1.println(" connected");
// reserve 200 bytes for the inputString:
//  inputString.reserve(200);
  // we need to call this to enable interrupts
    interrupts();
  Udp.begin(localUdpPort);
  Serial1.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}

void loop()
{
  // send
  if ( sendFlag == sending) {
    Serial1.println("S");
    sendUDP();
  }
  // send time
  if ( sendTimeFlag == sending) {
    Serial1.println("ST");
    sendTimeUDP();
  }
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    serialEvent();
  }
  delay(1);
 // sendUDP();
}
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    GetNewData(inChar);
  }
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
 // Serial1.println("Check");
  if( (Received_pck.DIST_PCK.func==MASTER_HELLO) && (Received_pck.DIST_PCK.addr == 0xFF) ){
     timeHour = Received_pck.DIST_PCK.Sensor1;
     timeMinute = Received_pck.DIST_PCK.Sensor2;
     sendBuffer[0] = (timeHour/10) + '0'; sendBuffer2[0] = (timeHour/10) + '0';
     sendBuffer[1] = (timeHour%10) + '0'; sendBuffer2[1] = (timeHour%10) + '0';
     sendBuffer[3] = (timeMinute/10) + '0'; sendBuffer2[3] = (timeMinute/10) + '0';
     sendBuffer[4] = (timeMinute%10) + '0'; sendBuffer2[4] = (timeMinute%10) + '0';
     sendTimeCounter++;
     if( (sendTimeCounter > 59) && (sendFlag == notSending) ) {
       sendTimeCounter = 0;
       sendTimeFlag = sending ;
     }
  } else if(Received_pck.DIST_PCK.func==SLAVE2_HELLO) {
    Serial1.println("C");
   // turn on alarm 
   if( (read_req(Received_pck.DIST_PCK.State) == 1) && (room[Received_pck.DIST_PCK.addr>>4] == 0) && (bed[Received_pck.DIST_PCK.addr%16] == 0)  ){  //  
       sendBuffer[6] = ((Received_pck.DIST_PCK.addr>>4)/10) + '0';
       sendBuffer[7] = ((Received_pck.DIST_PCK.addr>>4)%10) + '0';
       sendBuffer[9] = ((Received_pck.DIST_PCK.addr%16)/10) + '0';
       sendBuffer[10] = ((Received_pck.DIST_PCK.addr%16)%10) + '0';
       sendBuffer[12] = 'T';
       sendFlag = sending ;
         Serial1.println("r");
         Serial1.println(Received_pck.DIST_PCK.addr>>4);
         Serial1.println("b");
         Serial1.println(Received_pck.DIST_PCK.addr%16);
         Serial1.println("a");
         Serial1.println(read_req(Received_pck.DIST_PCK.State));
   }
   // turn off alarm
   if( (read_req(Received_pck.DIST_PCK.State) == 0) && (room[Received_pck.DIST_PCK.addr>>4] == 1) && (bed[Received_pck.DIST_PCK.addr%16] == 1)  ) {  // 
       sendBuffer[6] = (Received_pck.DIST_PCK.addr>>4)/10 + '0';
       sendBuffer[7] = (Received_pck.DIST_PCK.addr>>4)%10 + '0';
       sendBuffer[9] = (Received_pck.DIST_PCK.addr%16)/10 + '0';
       sendBuffer[10] = (Received_pck.DIST_PCK.addr%16)%10 + '0';
       sendBuffer[12] = 'F';
       sendFlag = sending ;
       Serial1.println("r");
         Serial1.println(Received_pck.DIST_PCK.addr>>4);
         Serial1.println("b");
         Serial1.println(Received_pck.DIST_PCK.addr%16);
         Serial1.println("a");
         Serial1.println(read_req(Received_pck.DIST_PCK.State));
   }

    room[Received_pck.DIST_PCK.addr>>4] = read_req(Received_pck.DIST_PCK.State) ;
    bed[Received_pck.DIST_PCK.addr%16] = read_req(Received_pck.DIST_PCK.State) ;
  }
  return PCK_Without_Me;
}

void sendUDP() {
  int j = 0;
  int i=0;
  for(j=0;j<3;j++) {   
    Udp.beginPacket(IP_Remote, localUdpPort);
    Udp.write(sendBuffer);// ReplyBuffer
    Udp.endPacket();
    delay(10);
    Serial1.println("SD");
  }
  sendFlag = notSending ;
}

void sendTimeUDP() {  
    int j = 0 ;
    for(j=0;j<13;j++) {
      Serial1.print(sendBuffer2[j]);
    }
    Udp.beginPacket(IP_Remote, localUdpPort);
    Udp.write(sendBuffer2);// ReplyBuffer
    Udp.endPacket();
    delay(10);
    Serial1.println("SDT");
  sendTimeFlag = notSending ;
}

