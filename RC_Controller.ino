/* 
RC Controller and Wifi Station (STA)


Nas Dombrowski


*/

#include <WiFi.h>
#include <WiFiUDP.h>

const char* ssid = "TwasbBrilligAndTheSlithyToves"; //jabberwocky

// variables for UDP
WiFiUDP udp;
unsigned int UDPlocalPort = 2100;   // UDP port number for local ESP
unsigned int UDPtargetPort = 2200;  // UDP port number on the target ESP
const int packetSize = 100;          // define packetSize (length of message)
byte sendBuffer[packetSize];        // create the sendBuffer array
int constrained;

  //AP MODE
  // IP Addresses
IPAddress IPlocal(192,168,1,114);         // initialize local IP address (Nas)
IPAddress IPtarget(192,168,1,161);      // initialize target IP address (James)

//joystick setup
const int xPin = 39; //X attach to A1/gpio39
const int yPin = 36; //Y attach to A0/gpio36
const int zPin = 34; //Bt attach to gpio34
int xVal, yVal, y, x;
bool z, zVal;

void setup() {
  
  Serial.begin(115200);
 
    // setup WiFi Network
  WiFi.mode(WIFI_AP);         // sets the WiFi mode to be AP mode
  WiFi.softAP(ssid);          // configures the ESP in AP mode with network name
  // WiFi.softAP(ssid, pass); // configures the ESP in AP mode with network name and password
  delay(100);                 // hack need to wait 100ms for AP_START...

  Serial.print("Set softAPConfig "); Serial.println(ssid);    // debug statement
    
  IPAddress gateway(192,168,1,1);                 // initializes gateway IP
  IPAddress subnet(255,255,255,0);                // initializes subnet mask
  WiFi.softAPConfig(IPlocal, gateway, subnet);    // sets the IP addr of ESP to IPlocal

  udp.begin(UDPlocalPort);    // configure a port for UDP comms

  IPAddress myIP = WiFi.softAPIP();   // demo the usage of the softAPIP command
  Serial.print("AP IP address: "); Serial.println(myIP);      // debug statement

    pinMode(zPin,INPUT); //set btpin as INPUT
    pinMode(xPin, INPUT);
    pinMode(yPin, INPUT);
    
}

void UDPsend(int x, int y, int z)
{
  sendBuffer[0] = x; // send vals
  sendBuffer[1] = y;
  sendBuffer[2] = z; 

  // send the message
  udp.beginPacket(IPtarget, UDPtargetPort);   // target IP and target port num to send info to
  udp.printf("%s", sendBuffer);               // send the contents of sendBuffer over WiFiUDP
  udp.endPacket();
  // end message
}

void loop() {
  y = analogRead(yPin);
  x = analogRead(xPin);
  z = analogRead(zPin);
  
  
  xVal = map(constrain(x, 100, 4095), 100, 4095, 1, 255); //constrained so never below operating PWM/no negatives
  yVal = map(constrain(y, 100, 4095), 100, 4095, 1, 255);//mapped from 1-255 to avoid sending zeros
  zVal = z;

  UDPsend(xVal, yVal, zVal);
  
}
