/*
DC Motor Control with ISRs and Servo Control

- AP Mode

Nas Dombrowski

*/
#include <WiFi.h>
#include <WiFiUDP.h>

//---------------------------LEDC ----------------------------------------

#define LEDC_CHANNEL1 0 //forward pin a
#define LEDC_CHANNEL2 1 //forward pin b
#define LEDC_CHANNEL3 2 //reverse pin a
#define LEDC_CHANNEL4 3 //reverse pin b
#define LEDC_CHANNELs 4 //steering servo control

#define LEDC_RESOLUTION_BITS 10 //10 bit res bay-BEE
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1)
#define LEDC_TIMER_BIT 10 //10 bit resolution for ledctimer
#define LEDC_FREQ_HZ 200 //for motor -- 5000Hz -- changed to 200
#define LEDC_BASE_FREQ 50 //for servo -- 50Hz
//
  
//---------------------------Var for UDP ----------------------------------------

WiFiUDP udp;
bool dataReceived = false;
unsigned int UDPlocalPort = 2200;   // UDP port number for local ESP
unsigned int UDPtargetPort = 2100;  // UDP port number on the target ESP
const int packetSize = 100;          // define packetSize (length of message)
byte receiveBuffer[packetSize];        // create the receiveBuffer array
int yUDP, xUDP;
bool zIO; //stop button
const char* ssid = "TwasbBrilligAndTheSlithyToves"; //jabberwocky


//---------------------------Motor Control ----------------------------------------

int FWD_PINA = 23; //forward motorA
int REV_PINA = 33; //backward A

int FWD_PINB =  22; // forward motorB
int REV_PINB = 32; //backward B
  
int servoPin = 25; //servo control pin

int Min = 40; // (40/1024) * 20ms
int Max = 70; // (70/1024) * 20ms 

  //AP MODE
  
  // IP Addresses
IPAddress IPlocal(192,168,1,161);         // initialize local IP address 
IPAddress IPtarget(192,168,1,114);      // initialize target IP address

//---------------------------Software ISR for UDP ----------------------------------------

volatile int ISRcnt; //poor ISRcnt has terribly volatile mood swings :(
//honestly, its relationship with ISR is so toxic, there's no stability

hw_timer_t * timer = NULL; //point to timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //handles syncing shared var btwn loop() & ISR

void IRAM_ATTR onTimer() { //place in IRAM 
  portENTER_CRITICAL_ISR(&timerMux); //enter critical section  ** sweating intensifies **
  ISRcnt++; //counter that will be checked before calling UDPreceive()
  portEXIT_CRITICAL_ISR(&timerMux);
}


//---------------------------VIVE ----------------------------------------


int vivePin = 32;
int syncCnt = 0;
volatile unsigned long pulseWidth, pulseFall, x, y, distance;  //holds the length of the input pulse.  volatile because it is updated by the ISR
volatile unsigned long pulseRise = 0;  //time the pulse started.  Used in calculation of the pulse length
volatile boolean newPulse = false;  //flag to indicate that a new pulse has been detected
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pW() //isr for getting pulse width and distance
{
  portENTER_CRITICAL(&mux);
  if (digitalRead(vivePin) == HIGH)  //if the pin is HIGH
  {
    pulseRise = micros();  //save the current time in microseconds
    newPulse = true;
    distance = pulseRise - pulseFall; //get the distance between end of last pulse and start of new pulse
  } else if (newPulse == true)  //otherwise if we are on a new pulse
  {
    pulseFall = micros(); //save time of fall
    pulseWidth = pulseFall - pulseRise;  //calculate the pulse width
    newPulse = false;  //set flag to indicate that we are done with new pulse
  }
  portEXIT_CRITICAL(&mux);
}
//---------------------------END VIVE ----------------------------------------


// can set syntax to be like analogWrite() with input[ 0 : valueMax ] 
//aka a Canal street version of analogWrite() --> suuuure, that Prada bag is totally real for $50 on a sidewalk       
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {            
  uint32_t duty =  LEDC_RESOLUTION * min(value, valueMax) / valueMax;   
  ledcWrite(channel, duty);  // write duty to LEDC
}

void setup() {
  Serial.begin(115200);

  //TIMER SETUP
  timer = timerBegin(0, 80, true); //start timer0, prescaler 80 (80MHz/80 means microsecs), count up (turn up woot woot)
  timerAttachInterrupt(timer, &onTimer, true); //attach timer to onTimer() which executes on interrupt
  timerAlarmWrite(timer, 10000, true); //trigger interrupt at set intervals (reset@10000 counts)
  timerAlarmEnable(timer); //duh

  //AP WIFI SETUP
   
  WiFi.begin(ssid);           // connect to network (ssid)

  IPAddress gateway(192,168,1,1);         // init gateway IP
  IPAddress subnet(255,255,255,0);        // init subnet mask
  WiFi.config(IPlocal, gateway, subnet);     // set IP address of ESP

  udp.begin(UDPlocalPort);    // configure a port for UDP comms

    // hold the code here and wait until the WiFi is connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!"); 
  
  //MOTOR SETUP
  
  pinMode(REV_PINA, OUTPUT); //reverse pin a
  pinMode(REV_PINB, OUTPUT);  //reverse pin a
  pinMode(FWD_PINA, OUTPUT); //fwd pin a
  pinMode(FWD_PINB, OUTPUT); //fwd pin b
  pinMode(servoPin, OUTPUT); //servo pwm control pin
  
  ledcSetup(LEDC_CHANNEL1, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS); //channel0, 60Hz, 13bits
  ledcSetup(LEDC_CHANNEL2, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS); //channel0, 60Hz, 13bits
  ledcSetup(LEDC_CHANNEL3, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS); //channel0, 60Hz, 13bits
  ledcSetup(LEDC_CHANNEL4, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS); //channel0, 60Hz, 13bits
  ledcAttachPin(FWD_PINA, LEDC_CHANNEL1);// forward A
  ledcAttachPin(FWD_PINB, LEDC_CHANNEL2); //forward B
  ledcAttachPin(REV_PINA, LEDC_CHANNEL3); //backward A
  ledcAttachPin(REV_PINB, LEDC_CHANNEL4); //backward B

  // Setup timer and attach timer to a pin
  ledcSetup(LEDC_CHANNELs, LEDC_BASE_FREQ, LEDC_TIMER_BIT); 
  ledcAttachPin(servoPin, LEDC_CHANNELs);

  //VIVE SETUP
  
  pinMode (vivePin, INPUT);
  //call interrupt function on change in pin value
  attachInterrupt(digitalPinToInterrupt(vivePin), pW, CHANGE); 
}


int UDPreceive () { //the UDP reception desk is pretty flaky, but speedy, solid 2.5 star hotel
  int cb = udp.parsePacket(); // read data (check to see the number of bytes of the packet)
  
  if (cb)
  {
    dataReceived = true; //data flag
    udp.read(receiveBuffer, packetSize); // y'all know what's up
    xUDP = receiveBuffer[0]; //save in global val
    yUDP = receiveBuffer[1];    //save in global val
    zIO = receiveBuffer[2];
  }
}


void HeyBuddySlowItDown() { //stop function for motors
  digitalWrite(FWD_PINA, LOW); // set low
  digitalWrite(FWD_PINB, LOW);
  ledcAnalogWrite(LEDC_CHANNEL3, 0);// 0% duty cycle
  ledcAnalogWrite(LEDC_CHANNEL4, 0);
}

void servoControl (int val) { //val is xUDP
  if (val > 122) {
    val = map(val, 122, 255, Max, Min); //map to max and min duty vals
    ledcWrite(LEDC_CHANNELs, val); //use existing write function

  } else if (val < 112) {
    val = map(val, 1, 112, Max, Min); //map to max and min duty vals -- check if inverted
    ledcWrite(LEDC_CHANNELs, val);  //use existing write function

  } else {
    ledcWrite(LEDC_CHANNELs, 55); //midpoint of min&max
  }
}


void setMotor(int val, bool onOff)
{
  if (val > 120) {  //120 is upper bound of center value for joystick
    val = map(val, 120, 255, 1, 255); //map half range of joystick to full PWM range
    if (onOff == 0) { //if below PWM threshold, 
      HeyBuddySlowItDown(); //it's a family neighborhood for crying out loud
    }
    digitalWrite(REV_PINB, LOW);  //set reverse pins low
    digitalWrite(REV_PINA, LOW);
    ledcAnalogWrite(LEDC_CHANNEL1, val);  //write PWM to fwd pins 
    ledcAnalogWrite(LEDC_CHANNEL2, val); 
    
  } else if (val < 116) { //115 is around lower bound of center value
    val = map(val, 1, 116, 255, 1); //half to full range
    
    if (onOff == 0) {
      HeyBuddySlowItDown(); //don't make me call the neighborhood watch
    }
    digitalWrite(FWD_PINA, LOW); //set forward pins low
    digitalWrite(FWD_PINB, LOW);
    ledcAnalogWrite(LEDC_CHANNEL3, val);  //write PWM to rev pins
    ledcAnalogWrite(LEDC_CHANNEL4, val);
    
  } else if (onOff == 0) {
      HeyBuddySlowItDown(); // this isn't a racetrack, you know, there are children playing!!
    }
} 

void viveLocalization () {
  if (pulseWidth != -1) //restricts bc loop is faster than ISR is called
  {
    Serial.print("syncCnt: ");
    Serial.println(syncCnt);
    if (pulseWidth > 60) // threshold for sync pulses, subject to change based on nature's whimsy
    {
      syncCnt++; //increment count for sync pulses
      pulseWidth = -1; //reset 
      if (syncCnt > 3) //make sure synccnt is set up to go into next sweep loop
      {
        syncCnt = 3;
      }
    }
    else
    {
      pulseWidth = -1; 
      if (syncCnt == 3) //if we have counted 3 syncs
      {
        syncCnt = 0; //reset
        x = distance; //use distance calc from ISR to get how far we are on x axis
        Serial.print("x: ");
        Serial.println(x);
      }
      else if (syncCnt == 1) //after sync pulse between x&y
      {
        syncCnt = 0; //reset counter
        y = distance; //get distance along y axis
        Serial.print("y: ");
        Serial.println(y);
      }
    }
  }
}

void loop() {
  if (ISRcnt > 0) { //if interrupt has been called
    portENTER_CRITICAL(&timerMux); //handles syncing shared var
    ISRcnt--; //bring the count back down (kinda a bully, poor count's self esteem is in the pits)
    portEXIT_CRITICAL(&timerMux);     //exit crit period
    UDPreceive(); //check controller output
  }
  if (dataReceived) { //if boolean was set in UDPreceive()
    servoControl(xUDP); //steer first
    setMotor(yUDP, zIO); //then power motor
    dataReceived = false; //reset flag (let your freak flag fly)
  }
  viveLocalization();
}
