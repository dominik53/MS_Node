/*
  Dominik Kijak, 2024r.
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_VL53L0X.h"
#include "I2C_BM8563.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

const char* ssid = "MS_Station";
const char* password = "MS_Station2115";
const char* serverUrl = "http://192.168.1.1";
const int serverPort = 80;
const String endpointGet = "/";
const String endpointUpdate = "/sendUpdate";
const String endpointPost = "/post";
#define SW0       9
#define PIN_BATT  0
#define PIN_WAKE  10
#define PIN_INT   7
#define PIN_SHUT  4
#define PIN_SDA   5
#define PIN_SCL   6

#define VOLTAGE_MEASUREMENTS 7
#define TOF_MEASUREMENTS 7
#define MIN_CONNECT_TIME 10 // seconds
#define MIN_WAKE_INTERVAL 300 // seconds
const float detectPrecision = 0.975;

struct Data {
    float distance;
    float batt;
};

struct UpdateData {
    int wakeInterval; // seconds
    int connectTime;  // seconds
    bool calibrate;
};

/*
  I2C ADRESSES
    0x29 - VL53L0X
    0x51 - RTC
*/

TaskHandle_t task1;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
Data acquiredData={0,0};
UpdateData receivedData={-1,-1,0};
Preferences preferences;
unsigned long startTime=0;
float battPercentage=0; // %
float ToFMeasurement=0; // mm

float ToFCalibratedDist=0; // mm
int wakeInterval=MIN_WAKE_INTERVAL; // seconds
int connectTime=MIN_CONNECT_TIME;   // seconds

bool ToFReady=0;
bool RTCReady=0;

bool batteryDone=0;
bool detectionDone=0;
bool postDone=0;
bool readyToSleep=0;
bool updateHandleDone[2]={0,0};

void TOFMeasureContinous(void *parameter);  // calculate median distance
void APConnect(void *parameter);            // connect to station's AP
void HTTPUpdate(void *parameter);           // get updated values
void HTTPPost(void *parameter);             // post data to station
void TOFStart(void *parameter);             // connect ToF sensor
void RTCStart(void *parameter);             // connect RTC
void readADC(void *parameter);              // calculate battery voltage
void updateValues(void *parameter);         // update values acquired from station
void controlTime(void *parameter);          // end scenarios, software watchdog

void setup() {
  Serial.begin(115200);

  // buttons
  pinMode(SW0, INPUT_PULLUP);

  // wake
  pinMode(PIN_WAKE, OUTPUT);
  digitalWrite(PIN_WAKE, HIGH);

  // NVS read
  preferences.begin("AppData", false);
  wakeInterval = preferences.getInt("wake", MIN_WAKE_INTERVAL);
  connectTime = preferences.getInt("connect", MIN_CONNECT_TIME);

  // i2c
  Wire.begin(PIN_SDA,PIN_SCL);

  // delay(10000);

  // tasks
  xTaskCreatePinnedToCore(APConnect,"APConnect",4096,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(HTTPPost,"HTTPPost",4096,nullptr,1,&task1,1 );

  xTaskCreatePinnedToCore(RTCStart,"RTCStart",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(TOFStart,"TOFStart",2048,nullptr,1,&task1,1 );

  xTaskCreatePinnedToCore(readADC,"readADC",2048,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(TOFMeasureContinous,"TOFMeasureContinous",2048,nullptr,1,&task1,1 );
  
  startTime = millis();
  xTaskCreatePinnedToCore(controlTime,"controlTime",2048,nullptr,1,&task1,1 );
}

void loop() {

}
////////////////////////////


void controlTime(void *parameter){
  for(;;){
    if(!RTCReady){ 
      if((millis()-startTime)/1000>60){
        // end if not connected with RTC
        Serial.println("RTC ERROR END");
        preferences.end();

        digitalWrite(PIN_WAKE, LOW);
        delay(20);
      }
      delay(20);
      continue;
    }

    unsigned long checkTime = millis();
    unsigned long runningTime = checkTime - startTime;

    if(runningTime/1000 > 60){
      // force end
      Serial.println("FORCE END");
      rtc.clearIRQ();
      rtc.SetAlarmIRQ(wakeInterval);
      preferences.end();

      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }

    if(runningTime/1000>connectTime && connectTime>=10 && (WiFi.status() != WL_CONNECTED)){
      // end, connection timeout
      Serial.println("END, CONNECTION TIMEOUT");
      rtc.clearIRQ();
      rtc.SetAlarmIRQ(wakeInterval);
      preferences.end();
 
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }else if(runningTime/1000>connectTime*1.5 && connectTime>=10){
      // end, error past post
      Serial.println("END, ERROR PAST POST");
      rtc.clearIRQ();
      rtc.SetAlarmIRQ(wakeInterval);
      preferences.end();
      
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }

    if(readyToSleep || (updateHandleDone[0]==1 && updateHandleDone[1]==1)){
      // wake after wakeInterval
      Serial.println("NORMAL END");
      rtc.clearIRQ();
      rtc.SetAlarmIRQ(wakeInterval);
      preferences.end();
      
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }

    delay(200);
  }

  vTaskDelete(NULL);
}


void updateValues(void *parameter){
  if(receivedData.calibrate){
    ToFCalibratedDist=ToFMeasurement;
    updateHandleDone[1]=1;
  }else{
    updateHandleDone[1]=1;
  }

  for(;;){
    if(updateHandleDone[1]){
      break;
    }else{
      delay(20);
    }
  }

  if(receivedData.connectTime!=-1){
    if(receivedData.connectTime>=MIN_CONNECT_TIME){
      connectTime=receivedData.connectTime;
    }else{
      connectTime=MIN_CONNECT_TIME;
    }
    preferences.putInt("connect", connectTime);

    Serial.print("New connect time: ");
    Serial.println(connectTime);
  }
  

  if(receivedData.wakeInterval!=-1){
    wakeInterval=receivedData.wakeInterval;
    preferences.putInt("wake", wakeInterval);

    Serial.print("New wake interval: ");
    Serial.println(wakeInterval);
  }

  preferences.end();
  updateHandleDone[0]=1;
  
  vTaskDelete(NULL);
}

void TOFMeasureContinous(void *parameter){
  for(;;){
    if(!ToFReady){
      taskYIELD();
    } else{
      break;
    } 
  }

  uint16_t ToFValue[TOF_MEASUREMENTS];
  lox.startRangeContinuous();  

  for(int i=0;i<TOF_MEASUREMENTS;i++){
    if (lox.isRangeComplete()) {
      ToFValue[i]=lox.readRange();
    }else{
      i--;
      taskYIELD();
    }
  }
  lox.stopRangeContinuous();

  // bubble sort
  for (int i = 0; i < TOF_MEASUREMENTS-1; i++) {
      for (int j = 0; j < TOF_MEASUREMENTS-i-1; j++) {
          if (ToFValue[j] > ToFValue[j+1]) {
              uint16_t temp = ToFValue[j];
              ToFValue[j] = ToFValue[j+1];
              ToFValue[j+1] = temp;
          }
      }
  }

  // median
  ToFMeasurement=(float)ToFValue[(int)round((TOF_MEASUREMENTS-1)/2)];
  Serial.println("Distance median: " + String(ToFMeasurement,0) + " mm");

  acquiredData.distance=ToFMeasurement;
  detectionDone=1;

  vTaskDelete(NULL);
}

void readADC(void *parameter){
  adcAttachPin(PIN_BATT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  uint32_t battValue[VOLTAGE_MEASUREMENTS];

  for(int i=0;i<VOLTAGE_MEASUREMENTS;i++){
    battValue[i] = 2.07 * analogReadMilliVolts(PIN_BATT);
  }

  // bubble sort
  for (int i = 0; i < VOLTAGE_MEASUREMENTS-1; i++) {
      for (int j = 0; j < VOLTAGE_MEASUREMENTS-i-1; j++) {
          if (battValue[j] > battValue[j+1]) {
              uint32_t temp = battValue[j];
              battValue[j] = battValue[j+1];
              battValue[j+1] = temp;
          }
      }
  }

  // median
  float battVoltage=((float)battValue[(int)round((VOLTAGE_MEASUREMENTS-1)/2)])/1000;
  battPercentage=(battVoltage-3.2)*100;
  batteryDone=1;
  Serial.println("Battery percentage: " + String(battPercentage) + " %");

  acquiredData.batt = battPercentage;

  if(battPercentage<10.0){
    Serial.println("Battery low, turning off");
    digitalWrite(PIN_WAKE, LOW);
  }

  vTaskDelete(NULL);
}

void HTTPPost(void *parameter){
  for(;;){
    if(WiFi.status() == WL_CONNECTED){

      for(;;){
        if(batteryDone && detectionDone){
          break;
        }else{
          delay(100);
        }
      }

      String messageDistance=String(acquiredData.distance);
      String messageBatt=String(acquiredData.batt);

      HTTPClient http;
      const String temp = ":";
      String url = serverUrl + temp + String(serverPort) + endpointPost;

      http.begin(url);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      String postData="";
      postData += "messageDistance=" + messageDistance;
      postData += "&messageBatt=" + messageBatt;

      int httpResponseCode = http.POST(postData);
      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);

        String response = http.getString();
        Serial.print("Server response: ");
        Serial.println(response);

        if(response=="update" && !postDone){
          xTaskCreatePinnedToCore(HTTPUpdate,"HTTPUpdate",4096,nullptr,1,&task1,1 );
        }else{
          readyToSleep=1;
        }

      } else {
        Serial.print("Error in HTTP request. HTTP Response code: ");
        Serial.println(httpResponseCode);
      }
      http.end();

      break;
    }else{
      Serial.println("HTTP Post waiting for connection...");
      delay(100);
    }
  }
  postDone=1;

  vTaskDelete(NULL);
}

void TOFStart(void *parameter){
  if (!lox.begin()) {
    for(;;){
      Serial.println(F("Failed to boot VL53L0X"));
      delay(1000);
      if(lox.begin()) break;
    }
  }
  ToFReady=1;
  Serial.println("I2C VL53L0X CONNECTED");
  
  vTaskDelete(NULL);
}


void RTCStart(void *parameter){
  rtc.begin();
  rtc.clearIRQ();
  RTCReady=1;
  Serial.println("I2C RTC CONNECTED");

  vTaskDelete(NULL);
}

void HTTPUpdate(void *parameter){
  for(;;){
    if(WiFi.status() == WL_CONNECTED && postDone){
      HTTPClient http;
      const String temp = ":";
      String url = serverUrl + temp + String(serverPort) + endpointUpdate;

      http.begin(url);
      int httpCode = http.GET();
      if (httpCode > 0) {

        Serial.print("Update received successfully. Response code: ");
        Serial.println(httpCode);

        String response = http.getString();
        sscanf(response.c_str(), "%d,%d,%d", &receivedData.wakeInterval, &receivedData.connectTime, &receivedData.calibrate);
        
        xTaskCreatePinnedToCore(updateValues,"updateValues",2048,nullptr,1,&task1,1 );

      } else {
        Serial.printf("HTTP Update failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
      break;
    }else{
      Serial.println("HTTP Update waiting for connection...");
      delay(100);
    }
  }
  vTaskDelete(NULL);
}


void APConnect(void *parameter){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  Serial.println("Connected to WiFi");

  vTaskDelete(NULL);
}
