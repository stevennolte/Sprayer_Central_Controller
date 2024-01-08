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


#include <Adafruit_ADS1X15.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include "driver/Ledc.h"
#include <UMS3.h>

UMS3 ums3;
// CONSTANTS ///////////////////////////////////////////////////
#define adsGain GAIN_TWOTHIRDS
#define I2C_Freq 100000
#define SDA_0 8
#define SCL_0 9
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_CHANNEL1            LEDC_CHANNEL_0
#define LEDC_CHANNEL2            LEDC_CHANNEL_1
#define LEDC_CHANNEL3            LEDC_CHANNEL_2
#define LEDC_CHANNEL4            LEDC_CHANNEL_3

const char* ssids[] = {"SSEI","BSTRIEGEL","FERT"};
const char* passwords[] = {"Nd14il!la","6sUDCRp5L4ps","Fert504!"};
// const char* ssid     = "SSEI";
// const char* password = "Nd14il!la";
// const char* ssid = "BSTRIEGEL";
// const char* password = "6sUDCRp5L4ps";
// const char* ssid = "FERT";
// const char* password = "Fert504!";
const uint8_t STATUS_LED = 2;
const uint8_t STATUS_LED_CHAN = 0;
const uint8_t WIFI_LED = 14;
const uint8_t WIFI_LED_CHAN = 2;
const uint8_t UDP_LED = 15;
const uint8_t UDP_LED_CHAN = 4;
// const uint8_t MAIN_FLOW_PIN = 14;
const uint8_t SECTION_1_PIN = 1;
const uint8_t SECTION_2_PIN = 2;
const uint8_t SECTION_3_PIN = 3;
const uint8_t SECTION_4_PIN = 4;
// const uint8_t SECTION_5_PIN = 3;
const uint8_t PWM_BOARD_1 = 0x40;
const uint8_t ADS1115_BOARD = 0x48;


// const uint8_t EXTRA_FREQ_1 = 6;
// const uint8_t EXTRA_FREQ_2 = 5;



// const uint8_t PUMP_CMD_PIN = 16;
// const uint8_t BYPASS_CMD_PIN = 15;

// CLASSES /////////////////////////////////////////////////////


TwoWire twoWire = TwoWire(0);
Adafruit_ADS1115 ads;
AsyncUDP udp;
IPAddress local_IP(192, 168, 0, 123);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// DEBUG //////////////////////////////////////////////////
// Scheduler runner;

struct debugMsgs_t{
  String debugName;
  float debugValue;
} debugMsgs[1];


// VARIABLES ///////////////////////////////////////////////////
int debugTimer = 0;
int rows = 0;
// INCOMING UDP STRUCTS ////////////////////////////////////////

struct __attribute__ ((packed)) rowStatesStruct_t{
  uint8_t row1:1;
  uint8_t row2:1;
  uint8_t row3:1;
  uint8_t row4:1;
  uint8_t row5:1;
  uint8_t row6:1;
  uint8_t row7:1;
  uint8_t row8:1;
};

union rowStates_u{
  rowStatesStruct_t rowStatsStruct;
  uint8_t bytes[1];
} rowStates;

// OUTGOING UDP STRUCTS ////////////////////////////////////////
struct sensorData_t{
  uint32_t railPressure;
};
sensorData_t sensorData;


// INTERNAL STRUCTS ////////////////////////////////////////////
struct valveDataStruct {
  uint8_t ValveNum;
  uint16_t dutyCycleCMD;
  uint16_t frequencyCMD;
  uint16_t frequencyRPT;
  uint16_t current_ma;
};

struct programStates_t{
  bool pwmDriverConnected;
  bool adsConnected;
  bool udpConnected;
  uint32_t udpTimer;
  

} programStates;


struct cmdData_t{
  uint8_t productEnable;
  uint16_t avgSpeed;
  uint16_t targetFlowrate;
  uint16_t targetRate;
  uint8_t rowsActive;
  uint8_t hydFlowTarget;
  uint8_t lhOuterWingRotate;
  uint8_t lhWingRotate;
  uint8_t lhWingLift;
  uint8_t centerLift;
  uint8_t rhWingLift;
  uint8_t rhWingRotate;
  uint8_t rhOuterWingRotate;
  uint8_t rowSpacing;
} cmdData;

struct reportData_t{
  uint16_t voltage1;
  uint16_t voltage2;
  uint16_t voltage3;
  uint16_t voltage4;
} reportData;

void IRAM_ATTR pulseCountMain(){
  
  return;
}
void IRAM_ATTR pulseCountSec1(){
  
  return;
}
void IRAM_ATTR pulseCountSec2(){
  
  return;
}
void IRAM_ATTR pulseCountSec3(){
 
  return;
}
void IRAM_ATTR pulseCountSec4(){

  return;
}
void IRAM_ATTR pulseCountSec5(){

  return;
}
void interruptSetup(){
  // pinMode(MAIN_FLOW_PIN, INPUT_PULLUP);
  pinMode(SECTION_1_PIN, INPUT);
  pinMode(SECTION_2_PIN, INPUT);
  pinMode(SECTION_3_PIN, INPUT);
  pinMode(SECTION_4_PIN, INPUT);
  // pinMode(SECTION_5_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(MAIN_FLOW_PIN), pulseCountMain, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_1_PIN), pulseCountSec1, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_2_PIN), pulseCountSec2, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_3_PIN), pulseCountSec3, RISING);
  attachInterrupt(digitalPinToInterrupt(SECTION_4_PIN), pulseCountSec4, RISING);
  // attachInterrupt(digitalPinToInterrupt(SECTION_5_PIN), pulseCountSec5, RISING);
}

class VoltageMonitor{
  public:
    VoltageMonitor(){
    
    }
    float voltMult = 0;
    void init(){
        ads.setGain(adsGain);
        if (!ads.begin(ADS1X15_ADDRESS, &twoWire)) {
          programStates.adsConnected = false;
        }
        switch(adsGain){
          case GAIN_TWOTHIRDS:
            voltMult = 1.875;
            break;
          case GAIN_ONE:
            voltMult = 1.25;
            break;
          case GAIN_TWO:
            voltMult = 0.625;
            break;
          case GAIN_FOUR:
            voltMult = .3125;
            break;
        }
    }

    void getVoltages(){
      if (programStates.adsConnected == true){
        
        reportData.voltage3=ads.readADC_SingleEnded(2)*voltMult;
        reportData.voltage4=ads.readADC_SingleEnded(3)*voltMult;
      }
    }

    void getCurrent(){
       reportData.voltage1 = ads.readADC_Differential_0_1()*voltMult;
    }
};
VoltageMonitor voltMon = VoltageMonitor();

class Valve{
  private:
    byte pin;
      byte duty = 0;
      byte freq = 50;
      ledc_channel_t channel;
      byte resolution = 8;
  public:
    valveDataStruct valveData;
    Valve(byte pin, ledc_channel_t channel){
      this ->pin = pin;
      this ->channel = channel;
    }

    void init()
      {
    // Prepare and then apply the LEDC PWM timer configuration
          ledc_timer_config_t ledc_timer = {
              .speed_mode       = LEDC_MODE,
              .duty_resolution  = LEDC_DUTY_RES,
              .timer_num        = LEDC_TIMER,
              .freq_hz          = freq,  // Set output frequency at 4 kHz
              .clk_cfg          = LEDC_AUTO_CLK
          };
          ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

          ledc_channel_config_t ledc_channel = {
              .gpio_num       = pin,
              .speed_mode     = LEDC_MODE,
              .channel        = channel,
              .intr_type      = LEDC_INTR_DISABLE,
              .timer_sel      = LEDC_TIMER,
              
              
              .duty           = 0, // Set duty to 0%
              .hpoint         = 0
          };
          ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
 
      }

      void updateValve(){
        // ledc_set_freq(LEDC_MODE, LEDC_TIMER, valveData.frequencyCMD);
        ledc_set_duty(LEDC_MODE, channel, valveData.dutyCycleCMD);
        ledc_update_duty(LEDC_MODE, channel);
      }
};
Valve valves[4] = {Valve(12,LEDC_CHANNEL1), Valve(13,LEDC_CHANNEL2),Valve(14,LEDC_CHANNEL3),Valve(15,LEDC_CHANNEL4)};

class PWMDriver{
  public:
    PWMDriver(){
    }
    void init(){
      byte error;
      twoWire.beginTransmission(0x40);
      error = twoWire.endTransmission();
      if (error == 0){
        Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();   //default address 0x40
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(150);
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

    void wifiFault(){
      ledcChangeFrequency(WIFI_LED_CHAN, 10, 8);
    }
    
    void statusFault(){
      ledcChangeFrequency(STATUS_LED_CHAN, 10, 8);
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
              break;
            case 229:
              
              cmdData.rowsActive=0;
              for (int i=5;i<11;i++){
                rowStates.bytes[0]=packet.data()[i];  
                cmdData.rowsActive+=rowStates.rowStatsStruct.row1;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row2;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row3;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row4;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row5;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row6;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row7;
                cmdData.rowsActive+=rowStates.rowStatsStruct.row8; 
              }
              break;

            case 151:
              
              cmdData.productEnable = packet.data()[5];
              cmdData.targetFlowrate = (uint16_t)(packet.data()[7])*256+(uint16_t)(packet.data()[6]);
              cmdData.targetRate = (uint16_t)(packet.data()[9])*256+(uint16_t)(packet.data()[8]);
              valves[0].valveData.dutyCycleCMD = packet.data()[13]*256+packet.data()[12];
              valves[0].updateValve();
              valves[1].valveData.dutyCycleCMD = packet.data()[17]*256+packet.data()[16];
              valves[1].updateValve();
              valves[2].valveData.dutyCycleCMD = packet.data()[21]*256+packet.data()[20];
              valves[2].updateValve();
              break;
            case 150:
              cmdData.hydFlowTarget = packet.data()[5];
              cmdData.lhOuterWingRotate = packet.data()[6];
              cmdData.lhWingRotate = packet.data()[7];
              cmdData.lhWingLift = packet.data()[8];
              cmdData.centerLift = packet.data()[9];
              cmdData.rhWingLift = packet.data()[10];
              cmdData.rhWingRotate = packet.data()[11];
              cmdData.rhOuterWingRotate = packet.data()[12];
          }
        }
      });
    }

    void udpCheck(){
      if(esp_timer_get_time()-programStates.udpTimer < 100000){
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
      int n = WiFi.scanNetworks();

      if (!WiFi.config(local_IP, gateway, subnet)) {
        Serial.println("STA Failed to configure");
      }
      WiFi.mode(WIFI_AP);
      for (int i=0;i<n;i++){
        for (int ii=0;ii<sizeof(ssids);ii++){
          if (WiFi.SSID(i) == ssids[ii]){
            WiFi.begin(ssids[ii], passwords[ii]);
          }
          if (WiFi.status() == WL_CONNECTED){
            break;
          }
        }
        if (WiFi.status() == WL_CONNECTED){
            break;
      }
      
      int wifiStartTime = esp_timer_get_time();
      while(WiFi.status() != WL_CONNECTED & esp_timer_get_time()-wifiStartTime < 1000000){ 
        delay(10);
      }
      Serial.println(WiFi.localIP());
      if (WiFi.status() == WL_CONNECTED){
        statusLed.wifiGood();
      } else {
        Serial.print("Can't connect to Wifi");
        statusLed.wifiFault();
        statusLed.statusFault();
      }
      delay(1000);
      
    }
    }
};
WifiMethods wifiMethods = WifiMethods();

class ProductControl{
  public:
    ProductControl(){
      cmdData.rowSpacing = 20;
    }

    void calcFlowRate(){
      uint16_t toolWidth = cmdData.rowsActive*cmdData.rowSpacing;
    }

};

void scanI2C(){
  uint8_t error = 0;
  Serial.println("Scanning...");
  for(uint8_t address = 17; address < 127; address++ ) {
    twoWire.beginTransmission(address);
    error = twoWire.endTransmission();
    if (error == 0) {
      
      switch(address){
        case 0x48:
          programStates.adsConnected=true;
          break;
        case 0x40:
          programStates.pwmDriverConnected=true;
    }

    }
  
  
  }
}

// TIMERS //////////////////////////////////////////////////////

void debugPrint(){
  if (esp_timer_get_time()-debugTimer>10000){
    Serial.print("Target Rate ");
    Serial.print(cmdData.targetRate);
    Serial.print(" speed "+String(cmdData.avgSpeed));
    Serial.print(" ");
    Serial.print(" Rows Active: ");
    Serial.print(cmdData.rowsActive);
    Serial.print(" UDPcheck ");
    Serial.print(programStates.udpConnected);
    Serial.print(" ADS State ");
    Serial.print(programStates.adsConnected);
    Serial.print(" Voltage Reading ");
    Serial.print(reportData.voltage1);
    Serial.print(" row4 duty ");
    Serial.print(valves[3].valveData.dutyCycleCMD);
    Serial.print(" row4 freq ");
    Serial.print(valves[3].valveData.frequencyCMD);
    Serial.println();
    debugTimer=esp_timer_get_time();
  }
}


void setup() {
  ums3.begin();
  ums3.setPixelBrightness(255);
  ums3.setPixelPower(true);
  ums3.setPixelColor(UMS3::color(245, 78, 66));
  
  
  Serial.begin(115000);
  Serial.println("Initializing");
  twoWire.begin(SDA_0, SCL_0);
  // Wire.begin(SDA_0, SCL_0, I2C_Freq);
  Serial.print("Wire clock ");
  Serial.println(twoWire.getClock());
  scanI2C();
  wifiMethods.init();
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  
  // byte error, address;
  // byte addressList[]={};
  
  
  
  
  
  // statusLed.init();
  Serial.println("OTA enabled");

  
  Serial.println("Wifi enabled");
  udpMethods.init();

  if (programStates.pwmDriverConnected){
    pwmDriver.init();
  }
  Serial.print("ads state ");
  Serial.println(programStates.adsConnected);
  if (programStates.adsConnected){
    voltMon.init();
  }
  interruptSetup();
  // pwmDriver.init();
  for (int i=0; i<4; i++){
    valves[i].init();
  }
  ums3.setPixelColor(UMS3::color(72,245,66));
    
}

void loop() {
  ArduinoOTA.handle();
  if (programStates.adsConnected){
    voltMon.getCurrent();
  }
  udpMethods.udpCheck();
  debugPrint();
  delay(1000);
}

