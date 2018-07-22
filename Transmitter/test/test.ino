#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>

const char* ssid = "Tenda_266360";  //MAAD    
const char* password = "Raiwan1234"; // SalamSalam  
WiFiUDP Udp;
unsigned int localUdpPort = 4211;  // local port to listen on Sender
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer for incoming packets
//IPAddress IP_Remote(192, 168, 1, 255); // 1.15   43.206
//IPAddress netmask(255, 255, 255, 0);

// send data
#define sending 1
#define notSending 0
uint8_t sendFlag = notSending ;
char  sendBuffer[] = "10:31-02-03-T\n";
char  sendBuffer2[] = "10:31-02-03-F\n";
uint8_t sendCounter = 0 ;
// Time 
uint8_t timeHour =0;
uint8_t timeMinute = 0;
uint8_t sendTimeCounter = 0; 
// beds and rooms
uint8_t room[16] = {0} ;
uint8_t bed[16] = {0} ;

IPAddress IP_Remote(192, 168, 1, 255); //19 BigPager 17 Watch

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.printf("Connecting to %s ", ssid);
//  WiFi.mode(WIFI_STA);
  delay(1000);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
// reserve 200 bytes for the inputString:
//  inputString.reserve(200);
  // we need to call this to enable interrupts
    interrupts();
//  Udp.begin(localUdpPort);
  while (! Udp.begin(localUdpPort) ) { 
    Serial.print("+");
  }
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
}

void loop()
{
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
    // *************************
    if(inChar == 'q')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-02-03-T\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-02-03-T");
    }
    if(inChar == 'a')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-02-03-F\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-02-03-F");
    }
    // *************************
    if(inChar == 'w')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-04-05-T\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-04-05-T");
    }
    if(inChar == 's')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-04-05-F\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-04-05-F");
    }
    // *************************
    if(inChar == 'e')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-06-07-T\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-06-07-T");
    }
    if(inChar == 'd')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-06-07-F\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-06-07-F");
    }
    // *************************
    if(inChar == 'r')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-08-09-T\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-08-09-T");
    }
    if(inChar == 'f')
    {
      Serial.println("Udp.beginPacket(IP_Remote, localUdpPort)");
      Udp.beginPacket(IP_Remote, localUdpPort);
      Udp.write("10:31-08-09-F\n");// ReplyBuffer 
      Udp.endPacket();
      Serial.println("10:31-08-09-F");
    }
  }
}

void sendUDP() {
  int i = 0;
  int j = 0;
  for(j=0;j<5;j++) {
//   for(i=0;i<100;i++) {
    
    Udp.beginPacket(IP_Remote, localUdpPort);
    Udp.write(sendBuffer);// ReplyBuffer
    Udp.endPacket();
//    delay(500);
//   }
   delay(10);
  }
//  sendFlag = sending ;
}


