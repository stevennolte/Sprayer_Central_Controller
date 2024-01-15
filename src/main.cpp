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
#include "driver/gpio.h"
#include "driver/twai.h"

// CONSTANTS ///////////////////////////////////////////////////
#define adsGain GAIN_TWOTHIRDS
#define I2C_Freq 400000
#define SDA_0 8
#define SCL_0 9
#define RX_PIN        7
#define TX_PIN        6
#define CAN_POWER_PIN 38

// #define RGB_BRIGHTNESS 100
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
const char* ssid = "";
const char* password = "";
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
const uint8_t addressPins[] = {39,40,41,42};
// const uint8_t EXTRA_FREQ_1 = 6;
// const uint8_t EXTRA_FREQ_2 = 5;



// const uint8_t PUMP_CMD_PIN = 16;
// const uint8_t BYPASS_CMD_PIN = 15;

// CLASSES /////////////////////////////////////////////////////

hw_timer_t * timer = NULL; 
UMS3 ums3;
TwoWire twoWire = TwoWire(0);
Adafruit_ADS1115 ads;
AsyncUDP udp;

IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);



// VARIABLES ///////////////////////////////////////////////////
volatile int color = 0;
int rows = 0;
uint8_t address;
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

// INCOMMING CANBUS ///////////////////////////////////////////
struct __attribute__ ((packed)) canIDStruct_t{
  uint32_t canID : 32;
};

union canID_u{
canIDStruct_t canIDStruct;
uint8_t bytes[sizeof(canIDStruct_t)];
} canID;

struct __attribute__ ((packed)) incommingCANflowStruct_t{
  uint32_t flowMeasure : 32;
  uint32_t volumeCnt : 32;
};
union incommingCANflow{
incommingCANflowStruct_t incommingCANflowStruct;
uint8_t bytes[sizeof(incommingCANflowStruct_t)];
} incommingCANflow;

struct __attribute__ ((packed)) incommingCANstatusStruct_t{
  uint8_t magneticIssue : 1;
  uint8_t rvsd1 : 1;
  uint8_t powerVoltageAlarm : 1;
  uint8_t overtemp : 1;
  uint8_t noValidCalibration : 1;
  uint8_t memoryAccessError : 1;
  uint8_t rvsd2 : 1;
  uint8_t outAstatus : 2;
  uint8_t outBstatus : 2;
  uint8_t rvsd3 : 4;
  uint8_t measureActive : 1;
  uint8_t measurePaused : 1;
  uint8_t flowing : 1;
  uint8_t empty : 1;
  uint8_t rvsd4 : 1;
  uint8_t cleanActive : 1;
  uint8_t virtualRateEnable : 1;
  uint8_t rvsd5 : 1;
  uint8_t unstableMeasure : 1;
  uint8_t flowmeterService : 1;
  uint64_t placeholder : 39;
};

union incommingCANstatus{
incommingCANstatusStruct_t incommingCANstatusStruct;
uint8_t bytes[sizeof(incommingCANstatusStruct_t)];
} incommingCANstatus;

struct __attribute__ ((packed)) flowmeterCANstartupStruct_t{
  uint8_t serialNumberDigit1 : 4;
  uint8_t serialNumberDigit2 : 4;
  uint8_t serialNumberDigit3 : 4;
  uint8_t flowmeterSubtype : 4;
  uint8_t flowmeterType : 4;
  uint8_t placeholder : 4;
};

union flowmeterCANstartup{
flowmeterCANstartupStruct_t flowmeterCANstartupStruct;
uint8_t bytes[sizeof(flowmeterCANstartupStruct_t)];
} flowmeterCANstartup;

// OUTGOING UDP STRUCTS ////////////////////////////////////////
struct sensorData_t{
  uint32_t railPressure;
};

sensorData_t sensorData;

struct __attribute__ ((packed)) heartbeatStruct_t{
  uint8_t aogByte1 : 8;
  uint8_t aogByte2 : 8;
  uint8_t aogByte3 : 8;
  uint8_t aogHeartBeatPGN : 8;
  uint8_t length : 8;
  uint8_t ipAddress : 8;
  uint8_t moduleState : 8;
  uint8_t adsState : 8;
  uint8_t pwmState : 8;
  uint8_t udpState : 8;
  uint8_t canFM1state : 8;
  uint8_t canFM2state : 8;
  uint8_t canFM3state : 8;
  uint8_t canFM4state : 8;
  uint8_t placeholder2 : 8;
  uint8_t checkSum : 8;
};

union heartbeat{
heartbeatStruct_t heartbeatStruct;
uint8_t bytes[sizeof(heartbeatStruct_t)];
} heartbeat;

// INTERNAL STRUCTS ////////////////////////////////////////////
// uint16_t sectionCmds[48];

struct valveDataStruct {
  uint8_t ValveNum;
  uint16_t dutyCycleCMD;
  uint16_t frequencyCMD;
  uint16_t frequencyRPT;
  uint16_t current_ma;
};

valveDataStruct sectionCmds[48];

struct programStates_t{
  bool wifiConnected;
  bool pwmDriverConnected;
  bool adsConnected;
  bool udpConnected;
  bool canFM1connected;
  bool canFM2connected;
  bool canFM3connected;
  bool canFM4connected;
  bool canFM5connected;
  uint32_t udpTimer;
  uint8_t canFMrow1sa;
  uint8_t canFMrow2sa;
  uint8_t canFMrow3sa;
  uint8_t canFMrow4sa;
  

} programStates;


struct cmdData_t{
  uint8_t productEnable;
  uint16_t avgSpeed;
  uint16_t targetFlowrate;
  uint16_t targetRate;
  uint8_t rowsActive;
  uint8_t numOfsections;
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
  uint16_t freq1;
  uint16_t freq2;
  uint16_t freq3;
  uint16_t freq4;
} reportData;

void IRAM_ATTR rgbTimer() {      //Defining Inerrupt function with IRAM_ATTR for faster access
  if (color > 255){
    color = 0;
  } else {
  color++;
  }
  ums3.setPixelColor(UMS3::colorWheel(color));
}

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
        reportData.voltage1=ads.readADC_SingleEnded(0)*voltMult;
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
  private:
    
    // sectionCmdPins_t section1CmdPins;
    
    
    struct sectionCmdPins_t{
      uint8_t powerPin;
      uint8_t directionPin;
    };
    sectionCmdPins_t sectionCmdPins[5];
    
    uint8_t pumpNum = 0;
    uint8_t bypassOpen = 1;
    uint8_t bypassClose = 2;
    uint16_t pumpCmdPrev = 0;
    
    uint16_t prevSectionCmds[5];
    uint16_t section1CmdPrev = 0;
    uint16_t sectiom2CmdPrev = 1;
    uint16_t section3CmdPrev = 2;
    uint16_t section4CmdPrev = 3;
    uint16_t section5CmdPrev = 4;
    
  public:
    PWMDriver(){
    }
    uint16_t pumpCmd=0;
    uint16_t sectionCmds[5] = {0,0,0,0,0};
    uint16_t section1Cmd = 0;     //for bi-directional functions, the command is 0-65535 with 0-32767 being close and 32768-65535 bing open
    uint16_t section2Cmd = 1;
    uint16_t section3Cmd = 2;
    uint16_t section4Cmd = 3;
    uint16_t section5Cmd = 4;
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, twoWire);
    
    void init(){
           //default address 0x40
        
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(150); 
        sectionCmdPins[0].powerPin = 3;
        sectionCmdPins[0].directionPin = 4;
        sectionCmdPins[1].powerPin = 5;
        sectionCmdPins[1].directionPin = 6;
        sectionCmdPins[2].powerPin = 7;
        sectionCmdPins[2].directionPin = 8;
    } 

    void updateSectionValves(){
      for (uint8_t i=0; i<5; i++){
        if (prevSectionCmds[i] != sectionCmds[i]){
          int tempCmd = sectionCmds[i]*0.125002 - 4096;
          if (tempCmd < 0){
            pwm.setPWM(sectionCmdPins[i].directionPin,0, 4096);
          } else {
            pwm.setPWM(sectionCmdPins[i].directionPin, 4096, 0);
          }
          pwm.setPWM(sectionCmdPins[i].powerPin, 0, 4096-tempCmd);
        }
      }
    }
};
PWMDriver pwmDriver = PWMDriver();

class StatusLed{
  private:
    int colorCnt = 0;
  public:
    StatusLed(){
      timer = timerBegin(0, 80, true);
      timerAttachInterrupt(timer, rgbTimer, true);
      timerAlarmWrite(timer, 50000, true);
      timerAlarmEnable(timer);
      ums3.setPixelBrightness(ledSettings.brightness);
      ums3.setPixelPower(ledSettings.power); 
    }

    struct ledSettings_t{
      uint8_t brightness = 60;
      uint8_t redValue = 255;
      uint8_t greenValue = 0;
      uint8_t blueValue = 0;
      uint16_t flashDuration;
      bool flash = false;
      bool power = true;
      bool colorWheel = false;

    } ledSettings;

    void init(){
      
    }

    void waiting(){
      // ums3.setPixelColor(UMS3::colorWheel(color));
      // ums3.setPixelBrightness(ledSettings.brightness);
      // ums3.setPixelPower(ledSettings.power);
      // ums3.setPixelColor(ledSettings.redValue,ledSettings.greenValue,ledSettings.blueValue);
    }

    void wifiGood(){
      timerDetachInterrupt(timer);
      ledSettings.redValue = 0;
      ledSettings.blueValue = 0;
      ledSettings.greenValue = 255;
      
      ums3.setPixelBrightness(ledSettings.brightness);
      ums3.setPixelPower(ledSettings.power);
      ums3.setPixelColor(ledSettings.redValue,ledSettings.greenValue,ledSettings.blueValue);
    }

    void wifiFault(){
      ledSettings.redValue = 255;
      ledSettings.blueValue = 0;
      ledSettings.greenValue = 0;
      ums3.setPixelPower(ledSettings.power);
      ums3.setPixelColor(ledSettings.redValue,ledSettings.greenValue,ledSettings.blueValue);
    }
    
    void statusFault(){
      
    }

    void udpGood(){
      
    }

    void statusGood(){
      
    }
};
StatusLed statusLed = StatusLed();

class UDPMethods{
  private:
    int heartbeatTimePrevious=0;
    int heartbeatTimeTrip=1000000;
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

            case 154:
              
              for (int i=0; i<cmdData.numOfsections; i++){
                uint16_t cmd =packet.data()[i*2+5]*256+packet.data()[i*2+6];
                if (cmd < 60000){
                  sectionCmds[i].dutyCycleCMD = cmd;
                }
              }
              
              break;

            case 151:
              
              cmdData.productEnable = packet.data()[5];
              cmdData.numOfsections = packet.data()[6];
              cmdData.rowSpacing = (uint16_t)(packet.data()[8])*256+(uint16_t)(packet.data()[7]);
              cmdData.targetFlowrate = (uint16_t)(packet.data()[10])*256+(uint16_t)(packet.data()[9]);
              cmdData.targetRate = (uint16_t)(packet.data()[12])*256+(uint16_t)(packet.data()[11]);
              
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
      if(esp_timer_get_time()-programStates.udpTimer < 1000000){
        programStates.udpConnected=true;
      } else {
        programStates.udpConnected=false;
      }
    }

    void sendHeartbeat(){
      if (esp_timer_get_time()-heartbeatTimePrevious > heartbeatTimeTrip){
        heartbeat.heartbeatStruct.aogByte1 = 0x80;
        heartbeat.heartbeatStruct.aogByte2 = 0x81;
        heartbeat.heartbeatStruct.aogByte3 = address;
        heartbeat.heartbeatStruct.aogHeartBeatPGN = 0x9B;
        heartbeat.heartbeatStruct.length = sizeof(heartbeatStruct_t)-6;
        heartbeat.heartbeatStruct.ipAddress = address;
        heartbeat.heartbeatStruct.adsState = programStates.adsConnected;
        heartbeat.heartbeatStruct.pwmState = programStates.pwmDriverConnected;
        heartbeat.heartbeatStruct.canFM1state = programStates.canFM1connected;
        heartbeat.heartbeatStruct.canFM2state = programStates.canFM2connected;
        heartbeat.heartbeatStruct.moduleState = programStates.udpConnected;
        int cksum=0;
        for (int i=2;i<=heartbeat.heartbeatStruct.length+1;i++)
        {
          cksum += heartbeat.bytes[i];
        }
        heartbeat.heartbeatStruct.checkSum = (byte)(cksum);
        // uint8_t udpSendBuffer[64];
        // for (int i=0; i<sizeof(heartbeatStruct_t);i++){
        //   udpSendBuffer[i]=heartbeat.bytes[i];
        // }
        udp.writeTo(heartbeat.bytes,sizeof(heartbeatStruct_t),IPAddress(192,168,0,255),9999);
        heartbeatTimePrevious = esp_timer_get_time();
      }
    }

};
UDPMethods udpMethods = UDPMethods();

class WifiMethods{
  public:
    WifiMethods(){

    }
    
    void init(){
      IPAddress local_IP(192, 168, 0, address);
      int n = WiFi.scanNetworks();
      Serial.print(n);
      Serial.println(" networks found");
      for (int i=0; i<min(n,25); i++){
        for (int si=0; si<3;si++){
          if (WiFi.SSID(i) == ssids[si]){
            ssid = ssids[si];
            password = passwords[si];
          }
        }
      }
      if (ssid == ""){
        Serial.println("No WiFi Found, REBOOTING");
        ESP.restart();
      }
      if (!WiFi.config(local_IP, gateway, subnet)) {
              Serial.println("STA Failed to configure");
            }
      WiFi.mode(WIFI_AP);
      WiFi.begin(ssid, password);
      while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        delay(1000);
      }
      if (WiFi.status() == WL_CONNECTED){
        statusLed.wifiGood();
      } else {
        statusLed.wifiFault();
      }
      delay(1000);
      
      
    
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

class CanHandler{
  private:
    uint16_t canFM1sn = 461;
    uint16_t canFM2sn = 460;
    uint16_t canFM3sn = 65000;
    uint16_t canFM4sn = 65000;
    uint64_t canFM1timer = 0;
    uint64_t canFM2timer = 0;
    uint64_t canFM3timer = 0;
    uint64_t canFM4timer = 0;
    

  public:
    CanHandler(){
    
      twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);  // TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK or TWAI_MODE_LISTEN_ONLY
      twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_250KBITS();
      twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();
      twai_driver_install(&g_config, &t_config, &f_config);
      twai_start();
      uint16_t canFMsns[] = {canFM1sn};
    }
    
    
    void checkForCanFM(){
      Serial.println("Checking for CAN Valves");
      
      pinMode(CAN_POWER_PIN, OUTPUT);
      digitalWrite(CAN_POWER_PIN, LOW);
      delay(2000);
      digitalWrite(CAN_POWER_PIN, HIGH);
      delay(1000);
      int canScanStartTime = esp_timer_get_time();
      while (esp_timer_get_time()-canScanStartTime > 10000000){
        canRecieve();
        delay(2);
      }
    }
    
    void wakeup_message(twai_message_t *message, uint32_t id, uint8_t dlc, uint8_t *data)
    {
    
      message->flags = TWAI_MSG_FLAG_EXTD;
      message->identifier = id;
      message->data_length_code = dlc;
      for (int i = 0; i < dlc; i++) {
        message->data[i] = data[i];
      }
      printf("Message created\nID: %ld DLC: %d Data:\t", message->identifier, message->data_length_code);
      for (int i = 0; i < message->data_length_code; i++) {
        printf("%d\t", message->data[i]);
    }
    printf("\n");
}

    void transmit_message(twai_message_t *message)
{
    if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to send message\n");
    }
}
    void checkCAN(){
      if (esp_timer_get_time() - canFM1timer > 1000000){
        programStates.canFM1connected = false;
      }
      if (esp_timer_get_time() - canFM2timer > 1000000){
        programStates.canFM2connected = false;
      }
    }

    void canRecieve(){
      twai_message_t message;
      if (twai_receive(&message, pdMS_TO_TICKS(1)) == ESP_OK) {
        // if (message.identifier == 419389697){
        //   // Serial.println(message.identifier, HEX);
          

        // }else if (message.identifier == 419389441){
        //   // Serial.println(message.identifier, HEX);
          
        //   for(int i=0;i<message.data_length_code;i++) {
        //     incommingCANstatus.bytes[i]=message.data[i];
        //     if (message.data[i]<=0x0F) {
              
        //     }
        //   }
        //   canFM2timer = esp_timer_get_time();

        // }else {
          
          canID.canIDStruct.canID = message.identifier;
          if (canID.bytes[1] == 0xFF & canID.bytes[2] == 0xEE){
            for (int i=0; i<sizeof(flowmeterCANstartupStruct_t); i++){
              flowmeterCANstartup.bytes[i] = message.data[i];
            }
            int flowmeterSerialNum = flowmeterCANstartup.flowmeterCANstartupStruct.serialNumberDigit3*100+flowmeterCANstartup.flowmeterCANstartupStruct.serialNumberDigit2*10+flowmeterCANstartup.flowmeterCANstartupStruct.serialNumberDigit1;
            switch(flowmeterSerialNum){
              case 461:
                Serial.print("461 ");
                Serial.println(message.identifier, HEX);
                programStates.canFM1connected = true;
                programStates.canFMrow1sa = canID.bytes[0];
              case 460:
                Serial.print("460 ");
                Serial.println(message.identifier, HEX);
                programStates.canFM2connected = true;
                programStates.canFMrow2sa = canID.bytes[0];
            }
          } else if (canID.bytes[1] == 0xEE & canID.bytes[2] == 0xFF & canID.bytes[0] == programStates.canFMrow1sa){
            for(int i=0;i<message.data_length_code;i++) {
              incommingCANflow.bytes[i]=message.data[i]; 
            }
            canFM1timer = esp_timer_get_time();
          } else if (canID.bytes[1] == 0xEE & canID.bytes[2] == 0xFF & canID.bytes[0] == programStates.canFMrow2sa){
            for(int i=0;i<message.data_length_code;i++) {
              incommingCANflow.bytes[i]=message.data[i]; 
            }
            canFM2timer = esp_timer_get_time();
          }
        }
      // }
      // checkCAN();
    }
};
CanHandler canHandler = CanHandler();

void scanI2C(){
  uint8_t error = 0;
  Serial.println("Scanning...");
  for(uint8_t address = 17; address < 127; address++ ) {
    twoWire.beginTransmission(address);
    error = twoWire.endTransmission();
    if (error == 0) {
      Serial.print(address);
      Serial.println(" found");
      switch(address){
        case 0x48:
          programStates.adsConnected=true;
          break;
        case 0x40:
          programStates.pwmDriverConnected=true;
  }}}
  Serial.println();
}

void getAddress(){
  
  uint8_t val=0;
  
  for (int i = 0; i < 4; i++){
    pinMode(addressPins[i], INPUT);
    delay(100);
    if (digitalRead(addressPins[i]) == HIGH){
      val+=pow(2,i);
    }
  }
  address= 37 + val;
}
// TIMERS //////////////////////////////////////////////////////


class DebugPrinter{
  private:
    uint64_t debugTimePrevious=0;
    int debugTimeTrip=1000000;

  public:
    DebugPrinter(){

    }
    void print(){
      if (esp_timer_get_time()-debugTimePrevious > debugTimeTrip){
        topDebug();
        rowDebug();
        debugTimePrevious = esp_timer_get_time();
      }
    }
    void topDebug(){
      
        Serial.print("TarRate ");
        Serial.print(cmdData.targetRate);
        Serial.print(" spd "+String(cmdData.avgSpeed));
        Serial.print(" ");
        Serial.print(" Rows Active: ");
        Serial.print(cmdData.rowsActive);
        Serial.print(" UDP ");
        Serial.print(programStates.udpConnected);
        Serial.print(" ADS ");
        Serial.print(programStates.adsConnected);
        Serial.print(" Volt ");
        Serial.print(reportData.voltage1);
        // Serial.print(" row4 duty ");
        // Serial.print(valves[3].valveData.dutyCycleCMD);
        // Serial.print(" row4 freq ");
        // Serial.print(valves[3].valveData.frequencyCMD);
        Serial.print(" CANfm1 ");
        Serial.print(programStates.canFM1connected);
        Serial.print(" CANfm2 ");
        Serial.print(programStates.canFM2connected);
        Serial.println();
        
      
    }

    void rowDebug(){
    
      for (int x = 0; x < cmdData.numOfsections; x+=8){
        for (int y = 0; y < 8; y++){
        
          Serial.print("Row ");
          Serial.print(x+y);
          Serial.print(" ");
          Serial.print(sectionCmds[x+y].dutyCycleCMD);
          Serial.print("\t");
        }
      Serial.println();
      }
    
    }
};
DebugPrinter debugPrinter = DebugPrinter();


void setup() {
  ums3.begin();
  Serial.begin(115000);
  Serial.println("Initializing");
  getAddress();
  Serial.print("IP Address: ");
  Serial.println(address);
  twoWire.begin(SDA_0, SCL_0, I2C_Freq);
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
  ArduinoOTA.handle();
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
  canHandler.checkForCanFM();
  ums3.setPixelColor(UMS3::color(72,245,66));
    
}

void loop() {
  ArduinoOTA.handle();
  for (int i=0; i<4; i++){
    if (valves[i].valveData.dutyCycleCMD != sectionCmds[i].dutyCycleCMD){
      valves[i].valveData.dutyCycleCMD = sectionCmds[i].dutyCycleCMD;
      valves[i].updateValve();
    }
  }

  if (programStates.adsConnected){
    voltMon.getVoltages();
  }

  udpMethods.udpCheck();
  debugPrinter.print();
  canHandler.canRecieve();
  udpMethods.sendHeartbeat();
  delay(10);
}

