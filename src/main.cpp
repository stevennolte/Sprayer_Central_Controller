// TODO
//    ads1115 methods
//    process if wifi disconnects
//    udp library
//    try different wifi connections



#include <Arduino.h>
#include <AsyncUDP.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_PWMServoDriver.h>
#include <ezOutput.h>
#include <TaskScheduler.h>
#include <SPI.h>
// CONSTANTS ///////////////////////////////////////////////////
const char* ssid     = "SSEI";
const char* password = "Nd14il!la";
// const char* ssid = "BSTRIEGEL";
// const char* password = "6sUDCRp5L4ps";
const uint8_t STATUS_LED = 2;
const uint8_t STATUS_LED_CHAN = 0;
const uint8_t WIFI_LED = 14;
const uint8_t WIFI_LED_CHAN = 2;
const uint8_t UDP_LED = 15;
const uint8_t UDP_LED_CHAN = 4;
const uint8_t MAIN_FLOW_PIN = 14;
const uint8_t SECTION_1_PIN = 13;
const uint8_t SECTION_2_PIN = 12;
const uint8_t SECTION_3_PIN = 5;
const uint8_t SECTION_4_PIN = 4;
const uint8_t SECTION_5_PIN = 3;
// const uint8_t EXTRA_FREQ_1 = 6;
// const uint8_t EXTRA_FREQ_2 = 5;



// const uint8_t PUMP_CMD_PIN = 16;
// const uint8_t BYPASS_CMD_PIN = 15;

// CLASSES /////////////////////////////////////////////////////


 //default address 0x40

AsyncUDP udp;
IPAddress local_IP(192, 168, 0, 123);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// DEBUG //////////////////////////////////////////////////
Scheduler runner;

struct debugMsgs_t{
  String debugName;
  float debugValue;
} debugMsgs[1];


// VARIABLES ///////////////////////////////////////////////////
int debugTimer = 0;
// INCOMING UDP STRUCTS ////////////////////////////////////////


// OUTGOING UDP STRUCTS ////////////////////////////////////////


// INTERNAL STRUCTS ////////////////////////////////////////////
struct programStates_t{
  bool pwmDriverConnected;
  bool adsConnected;
  bool udpConnected;
  uint32_t udpTimer;

} programStates;


struct cmdData_t{
  uint16_t avgSpeed;
  uint16_t targetFlowrate;
  uint16_t targetRate;
  uint8_t rowsActive;
} cmdData;



void IRAM_ATTR pulseCountMain(){
  Serial.println("pulse");
  return;
}
void IRAM_ATTR pulseCountSec1(){
  Serial.println("pulse");
  return;
}
void IRAM_ATTR pulseCountSec2(){
  Serial.println("pulse");
  return;
}
void IRAM_ATTR pulseCountSec3(){
  Serial.println("pulse");
  return;
}
void IRAM_ATTR pulseCountSec4(){
  Serial.println("pulse");
  return;
}
void IRAM_ATTR pulseCountSec5(){
  Serial.println("pulse");
  return;
}
void interruptSetup(){
  pinMode(MAIN_FLOW_PIN, INPUT_PULLUP);
  pinMode(SECTION_1_PIN, INPUT_PULLUP);
  pinMode(SECTION_2_PIN, INPUT_PULLUP);
  pinMode(SECTION_3_PIN, INPUT_PULLUP);
  pinMode(SECTION_4_PIN, INPUT_PULLUP);
  pinMode(SECTION_5_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAIN_FLOW_PIN), pulseCountMain, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_1_PIN), pulseCountSec1, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_2_PIN), pulseCountSec2, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_3_PIN), pulseCountSec3, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_4_PIN), pulseCountSec4, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_5_PIN), pulseCountSec5, RISING);
}

class PWMDriver{
  public:
    PWMDriver(){
    }
    void init(){
      byte error;
      Wire.beginTransmission(0x40);
      error = Wire.endTransmission();
      if (error == 0){
        // Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
        // pwm.begin();
        // pwm.setOscillatorFrequency(27000000);
        // pwm.setPWMFreq(150);
        programStates.pwmDriverConnected = true;
      } else {
        programStates.pwmDriverConnected = false;
        Serial.println(error);
        Serial.println("pwm driver not connected");
      }
    } 
};
PWMDriver pwmDriver = PWMDriver();

class StatusLed{
  public:
    StatusLed(){

    }
    void init(){
      ledcSetup(STATUS_LED_CHAN,4,8);
      ledcSetup(WIFI_LED_CHAN,4,8);
      ledcSetup(UDP_LED_CHAN,4,8);
      ledcAttachPin(STATUS_LED, 0);
      ledcAttachPin(WIFI_LED,2);
      ledcAttachPin(UDP_LED,4);
      ledcWrite(STATUS_LED_CHAN, 50);
      ledcWrite(WIFI_LED_CHAN, 50);
      ledcWrite(UDP_LED_CHAN, 50);
    }

    void wifiGood(){
      ledcWrite(WIFI_LED_CHAN, 255);
    }

    void udpGood(){
      ledcWrite(UDP_LED_CHAN, 255);
    }

    void statusGood(){
      ledcWrite(STATUS_LED_CHAN, 255);
    }
};
StatusLed statusLed = StatusLed();

class UDPMethods{
  public:
    UDPMethods(){
    }

    void init(){
      udp.listen(8888);
      Serial.println("UDP Listening");
      udp.onPacket([](AsyncUDPPacket packet) {
        if (packet.data()[0]==0x80 & packet.data()[1]==0x81){
          programStates.udpTimer=esp_timer_get_time();
          switch(packet.data()[3]){
            case 254:
              cmdData.avgSpeed = (uint16_t)(packet.data()[6])*256+(uint16_t)(packet.data()[5]);
              // Serial.println(cmdData.avgSpeed);
              // debugMsgs[(int)(sizeof(debugMsgs))].debugName="avgSpeed";
              // debugMsgs[1].debugValue=cmdData.avgSpeed;
              break;

            case 101:

              break;
          }
        }
      });
    }

    void udpCheck(){
      if(esp_timer_get_time()-programStates.udpTimer < 1000){
        programStates.udpConnected=true;
      } else {
        programStates.udpConnected=false;
      }
    }

};
UDPMethods udpMethods = UDPMethods();

class WifiMethods{
  public:
    WifiMethods(){

    }
    
    void init(){
      if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure");
      }
      WiFi.mode(WIFI_AP);
      WiFi.begin(ssid, password);
      while(WiFi.status() != WL_CONNECTED){ 
        delay(10);
      }
      Serial.println(WiFi.localIP());
      if (WiFi.status() == WL_CONNECTED){
        statusLed.wifiGood();
      }
      delay(1000);
    }
};
WifiMethods wifiMethods = WifiMethods();


// TIMERS //////////////////////////////////////////////////////

void debugPrint(){
  if (esp_timer_get_time()-debugTimer>10000){
    Serial.println("speed "+String(cmdData.avgSpeed));
    debugTimer=esp_timer_get_time();
  }
  
}


void setup() {
  Serial.begin(115000);
  // Wire.setClock(400000);
  statusLed.init();
  wifiMethods.init();
  udpMethods.init();
  pwmDriver.init();
  interruptSetup();
  // pwmDriver.init();
  
    
}

void loop() {
  debugPrint();
  // udpMethods.udpCheck();
  delay(150);
}

