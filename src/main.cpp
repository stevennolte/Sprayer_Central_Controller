#include <Arduino.h>
#include <AsyncUDP.h>
#include <Wire.h>
#include <WiFi.h>
// #include <Adafruit_PWMServoDriver.h>
#include <ezOutput.h>
#include <TaskScheduler.h>
#include <SPI.h>
// CONSTANTS ///////////////////////////////////////////////////
const char* ssid     = "SSEI";
const char* password = "Nd14il!la";
const uint8_t STATUS_LED = 2;
const uint8_t WIFI_LED = 14;
const uint8_t UDP_LED = 15;


// const uint8_t PUMP_CMD_PIN = 16;
// const uint8_t BYPASS_CMD_PIN = 15;

// CLASSES /////////////////////////////////////////////////////


// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //default address 0x40

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

void debugCallback(){
  Serial.print("Debug ");
  // for (int i=0; i<sizeof(debugMsgs); i++){
  //   Serial.print(debugMsgs->debugName);
  //   Serial.print(" ");
  //   Serial.print(debugMsgs->debugValue);
  //   Serial.print(" : ");
  // }
  Serial.println();
}
Task debug(1000, TASK_FOREVER, &debugCallback);


// VARIABLES ///////////////////////////////////////////////////
int debugTimer = 0;
// INCOMING UDP STRUCTS ////////////////////////////////////////


// OUTGOING UDP STRUCTS ////////////////////////////////////////


// INTERNAL STRUCTS ////////////////////////////////////////////



struct cmdData_t{
  uint16_t avgSpeed;
  uint16_t targetFlowrate;
  uint16_t targetRate;
  uint8_t rowsActive;
} cmdData;


// class PWMDriver{
//   public:
//     PWMDriver(){
//       pwm.begin();
//       pwm.setOscillatorFrequency(27000000);
//       pwm.setPWMFreq(150);
//     }
       
    
// };
// PWMDriver pwmDriver = PWMDriver();

class Startup{
  public:
    Startup(){
    }

    void run(){  
      
      ledsetup();
      
      wifiInit();
      flowmeterInit();
      udpSetup();
      ledcWrite(0,255);
      return;
    }
    void ledsetup(){
      ledcSetup(0,4,8);
      ledcSetup(2,4,8);
      ledcAttachPin(STATUS_LED, 0);
      ledcAttachPin(WIFI_LED,2);
      ledcWrite(0,50);
    }
    

    void wifiInit(){
      ledcWrite(2,100);
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
        ledcWrite(2,255);
      }
      delay(1000);
      return;
    }
    void pwmdriver(){
      
    }
    void flowmeterInit(){
      return;
    }
    
    void udpSetup(){
      udp.listen(8888);
      udp.onPacket([](AsyncUDPPacket packet) {
        if (packet.data()[0]==0x80 & packet.data()[1]==0x81){
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
};
Startup startup = Startup();

// TIMERS //////////////////////////////////////////////////////

void debugPrint(){
  if (esp_timer_get_time()-debugTimer>10000){
    Serial.println("speed "+String(cmdData.avgSpeed));
    debugTimer=esp_timer_get_time();
  }
  
}

void setup() {
  
  Serial.begin(115000);
  Serial.println("starting setup");
  // Wire.setClock(400000);
  startup.run();
  Serial.println("setup complete");
  
  
}

void loop() {
  
  debugPrint();
  // put your main code here, to run repeatedly:
  delay(250);
}

