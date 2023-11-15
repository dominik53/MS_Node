#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_FRAM_I2C.h"
#include "I2C_BM8563.h"
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "MS_Station";
const char* password = "MS_Station2115";
const char* serverUrl = "http://192.168.1.1";
const int serverPort = 80;
const String endpointGetTime = "/getTime";
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

/*
  I2C ADRESSES
    0x29 - VL53L0X
    0x50 - FRAM
    0x51 - RTC
*/

/*
  todo:
    - obsluga battPercentage
    - obsluga wynikow ToF
    - komunikacja ze station (wysylanie wynikow)
    - odczyt/zapis fram
    - zapis nowej daty po otrzymaniu z http get
    - obsluga kalibracji
*/

TaskHandle_t task1;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
float battPercentage=0; // %
float ToFMeasurement=0; // mm
int secondsToAlarm=0;

bool ToFReady=0;
bool FRAMReady=0;
bool RTCReady=0;
bool batteryDone=0;
bool ToFDone=0;

void taskPrint(void *parameter);            // dev only
void scanI2C(void *parameter);              // dev only
void TOFMeasure(void *parameter);           // dev only
void TOFMeasureContinous(void *parameter);  // calculate median distance
void RTCGet(void *parameter);
void RTCSet(void *parameter);
void FRAMRead(void *parameter);
void FRAMWrite(void *parameter);
void APConnect(void *parameter);    // connect to station's AP
void HTTPGetTime(void *parameter);
void HTTPPost(void *parameter);
void TOFStart(void *parameter);     // connect ToF sensor
void FRAMStart(void *parameter);    // connect FRAM
void RTCStart(void *parameter);     // connect RTC
void readADC(void *parameter);      // calculate battery voltage

void setup() {
  Serial.begin(115200);
  // while(!Serial);

  // buttons
  pinMode(SW0, INPUT_PULLUP);

  // wake
  pinMode(PIN_WAKE, OUTPUT);
  digitalWrite(PIN_WAKE, HIGH);

  // i2c
  Wire.begin(PIN_SDA,PIN_SCL);

  // tasks
  // xTaskCreatePinnedToCore(APConnect,"APConnect",4096,nullptr,1,&task1,1 );
  // xTaskCreatePinnedToCore(HTTPPost,"HTTPPost",4096,nullptr,1,&task1,1 );

  xTaskCreatePinnedToCore(FRAMStart,"FRAMStart",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(RTCStart,"RTCStart",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(TOFStart,"TOFStart",2048,nullptr,1,&task1,1 );

  delay(5000);
  xTaskCreatePinnedToCore(readADC,"readADC",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(TOFMeasureContinous,"TOFMeasureContinous",2048,nullptr,1,&task1,1 );
  
}

void loop() {
  int pushedSwitch = digitalRead(SW0);

  if(pushedSwitch==LOW){
    Serial.println("WCISNIETO.");
    delay(100);
  }

}
////////////////////////////


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
  ToFDone=1;
  Serial.println("Distance median: " + String(ToFMeasurement,0) + " mm");

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
  battPercentage=((float)battValue[(int)round((VOLTAGE_MEASUREMENTS-1)/2)])/37;
  batteryDone=1;
  Serial.println("Battery percentage: " + String(battPercentage) + " %");

  if(battPercentage<10.0){
    Serial.println("Battery low, turning off");
    digitalWrite(PIN_WAKE, LOW);
  }

  vTaskDelete(NULL);
}

void HTTPPost(void *parameter){
  for(;;){
    if(WiFi.status() == WL_CONNECTED){
      HTTPClient http;
      const String temp = ":";
      String url = serverUrl + temp + String(serverPort) + endpointPost;

      http.begin(url);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      String postData = "message=testowanie1&message2=testowanie2"; // battery, objDetected, time
      int httpResponseCode = http.POST(postData);

      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);

        String response = http.getString();
        Serial.print("Server response: ");
        Serial.println(response);
      } else {
        Serial.print("Error in HTTP request. HTTP Response code: ");
        Serial.println(httpResponseCode);
      }
      http.end();

      delay(1000);
    }else{
      Serial.println("HTTP Post waiting for connection...");
      delay(100);
    }
  }

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

void FRAMStart(void *parameter){
  if (!fram.begin(0x50)) {
    for(;;){
      Serial.println("I2C FRAM not identified");
      delay(1000);
      if(fram.begin(0x50)) break;
    }
  }
  FRAMReady=1;
  Serial.println("I2C FRAM CONNECTED");

  vTaskDelete(NULL);
}

void RTCStart(void *parameter){
  rtc.begin();
  RTCReady=1;
  Serial.println("I2C RTC CONNECTED");

  vTaskDelete(NULL);
}

void HTTPGetTime(void *parameter){
  for(;;){
    if(WiFi.status() == WL_CONNECTED){
      HTTPClient http;
      const String temp = ":";
      String url = serverUrl + temp + String(serverPort) + endpointGetTime;

      http.begin(url);
      int httpCode = http.GET();
      if (httpCode > 0) {
        Serial.printf("HTTP GET TIME request successful, status code: %d\n", httpCode);
        Serial.println(http.getString());
        /*
          todo: tu wywołanie taska probujacego zapisac nowa date
        */
      } else {
        Serial.printf("HTTP GET TIME request failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
      delay(1000);
    }else{
      Serial.println("HTTP Get waiting for connection...");
      delay(100);
    }
  }  
  vTaskDelete(NULL);
}

void APConnect(void *parameter){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    // Serial.println("Connecting to WiFi...");
    delay(100);
  }
  Serial.println("Connected to WiFi");

  vTaskDelete(NULL);
}

void FRAMRead(void *parameter){
  for(;;){
    uint8_t value;

    value = fram.read(0x0);
    Serial.print("Wynik: 0x");
    Serial.print(value, HEX);
    /*
    uint8_t test = fram.read(0x0); //max 0x7FFF
    delay(10000);
    Serial.print("Restarted ");
    Serial.print(test);
    Serial.println(" times");
    fram.write(0x0, test+1);
    */
    vTaskDelete(NULL);
  }
  vTaskDelete(NULL);
}

void FRAMWrite(void *parameter){
  for(;;){
    // fram.write(0x0, test+1);
  }
  vTaskDelete(NULL);
}

void RTCSet(void *parameter){
  // Set RTC Date
  I2C_BM8563_DateTypeDef dateStruct;
  dateStruct.weekDay = 5;
  dateStruct.month = 11;
  dateStruct.date = 10;
  dateStruct.year = 2023;
  rtc.setDate(&dateStruct);

  // Set RTC Time
  I2C_BM8563_TimeTypeDef timeStruct;
  timeStruct.hours   = 19;
  timeStruct.minutes = 45;
  timeStruct.seconds = 0;
  rtc.setTime(&timeStruct);

  vTaskDelete(NULL);
}

void RTCGet(void *parameter){
  I2C_BM8563_DateTypeDef dateStruct;
  I2C_BM8563_TimeTypeDef timeStruct;
  for(;;){
    rtc.getDate(&dateStruct);
    rtc.getTime(&timeStruct);

    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d\n",
                  dateStruct.year,
                  dateStruct.month,
                  dateStruct.date,
                  timeStruct.hours,
                  timeStruct.minutes,
                  timeStruct.seconds
                 );

    delay(1000);
  }
  vTaskDelete(NULL);
}

void TOFMeasure(void *parameter){
  for(;;){
    VL53L0X_RangingMeasurementData_t measure;
    
    Serial.print("Reading a measurement... ");
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" out of range ");
    }
    delay(500);
  }
  vTaskDelete(NULL);
}

void taskPrint(void *parameter){
  for(int i=0;i<10;i++){
    Serial.println("taskPrint!");
    delay(1000);
  }
  vTaskDelete(NULL);
}

void scanI2C(void *parameter){
  for(;;){
    byte error, address;
    int nDevices;
  
    Serial.println("Scanning...");
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
  
      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address<16)
          Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
  
        nDevices++;
      }
      else if (error==4)
      {
        Serial.print("Unknown error at address 0x");
        if (address<16)
          Serial.print("0");
        Serial.println(address,HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
  
    delay(5000);
  }
  vTaskDelete(NULL);
}