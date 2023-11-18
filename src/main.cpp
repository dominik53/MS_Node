// return IPAddress(0U);        ==>  return IPAddress((uint32_t)0);
// mbedtls_md5_starts(&_ctx);   ==>  mbedtls_md5_starts_ret(&_ctx);

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
const String endpointGet = "/";
const String endpointUpdate = "/sendUpdate";
const String endpointUpdateReceived = "/updateReceived";
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
#define MIN_WAKE_INTERVAL 5 // minutes
const float detectPrecision = 0.975;

struct Data {
    bool detected;
    float batt;
    int day;
    int month;
    int year;
    int hours;
    int minutes;
    int seconds;
};

struct UpdateData {
    int day;
    int month;
    int year;
    int hours;
    int minutes;
    int wakeInterval; // minutes
    int connectTime;  // seconds
    bool calibrate;
};

/*
  I2C ADRESSES
    0x29 - VL53L0X
    0x50 - FRAM
    0x51 - RTC
*/

/*
  todo:
    - obsluga kalibracji
*/

TaskHandle_t task1;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
I2C_BM8563 rtc(I2C_BM8563_DEFAULT_ADDRESS, Wire);
Data acquiredData={0,0,0,0,0,0,0,0};
UpdateData receivedData={-1,-1,-1,-1,-1,-1,-1,0};
unsigned long startTime=0;
float battPercentage=0; // %
float ToFMeasurement=0; // mm

float ToFCalibratedDist=0; // mm    // todo: write to fram
int wakeInterval=MIN_WAKE_INTERVAL; // minutes
int connectTime=MIN_CONNECT_TIME;   // seconds

bool ToFReady=0;
bool FRAMReady=0;
bool RTCReady=0;
bool dateGot=0;
bool timeGot=0;

bool batteryDone=0;
bool detectionDone=0;
bool dateDone=0;
bool timeDone=0;
bool postDone=0;
bool updateDone=0;
bool updateReceivedDone=0;
bool readyToSleep=0;
bool updateHandleDone[2]={0,0};
bool calibratedDistReady=0;

void scanI2C(void *parameter);              // dev only
void TOFMeasureContinous(void *parameter);  // calculate median distance
void RTCGet(void *parameter);               // get values from RTC (date & time)
void FRAMRead(void *parameter);             // read values from FRAM and update RTC if needed
void APConnect(void *parameter);            // connect to station's AP
void HTTPUpdate(void *parameter);           // get updated values
void HTTPUpdateReceived(void *parameter);   // confirm getting the updated values
void HTTPPost(void *parameter);             // post data to station
void TOFStart(void *parameter);             // connect ToF sensor
void FRAMStart(void *parameter);            // connect FRAM
void RTCStart(void *parameter);             // connect RTC
void readADC(void *parameter);              // calculate battery voltage
void updateValues(void *parameter);         // update values acquired from station
void calibrate(void *parameter);            // calibrate new distance
void controlTime(void *parameter);          // end scenarios, software watchdog

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

  // delay(10000);

  // tasks
  xTaskCreatePinnedToCore(APConnect,"APConnect",4096,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(HTTPPost,"HTTPPost",4096,nullptr,1,&task1,1 );

  xTaskCreatePinnedToCore(FRAMStart,"FRAMStart",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(RTCStart,"RTCStart",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(TOFStart,"TOFStart",2048,nullptr,1,&task1,1 );

  xTaskCreatePinnedToCore(RTCGet,"RTCGet",2024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(readADC,"readADC",1024,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(TOFMeasureContinous,"TOFMeasureContinous",2048,nullptr,1,&task1,1 );
  xTaskCreatePinnedToCore(FRAMRead,"FRAMRead",2048,nullptr,1,&task1,1 );
  
  startTime = millis();
  xTaskCreatePinnedToCore(controlTime,"controlTime",2048,nullptr,1,&task1,1 );
}

void loop() {
  if(digitalRead(SW0)==LOW){
    Serial.println("WCISNIETO.");
    delay(100);
  }
  

}
////////////////////////////


void controlTime(void *parameter){
  for(;;){
    if(!RTCReady){
      delay(20);
      continue;
    }

    unsigned long checkTime = millis();
    unsigned long runningTime = checkTime - startTime;

    if(runningTime/1000 > 60){
      // force end
      Serial.println("FORCE END");
      rtc.clearIRQ();
      if(wakeInterval>=MIN_WAKE_INTERVAL){
        rtc.SetAlarmIRQ(wakeInterval*60);
      }else{
        rtc.SetAlarmIRQ(MIN_WAKE_INTERVAL*60);
      }
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }

    if(runningTime/1000>connectTime && connectTime>=10 && (WiFi.status() != WL_CONNECTED)){
      // end, connection timeout
      Serial.println("END, CONNECTION TIMEOUT");
      rtc.clearIRQ();
      if(wakeInterval>=MIN_WAKE_INTERVAL){
        rtc.SetAlarmIRQ(wakeInterval*60);
      }else{
        rtc.SetAlarmIRQ(MIN_WAKE_INTERVAL*60);
      }
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }else if(runningTime/1000>connectTime*1.5 && connectTime>=10){
      // end, error past post
      Serial.println("END, ERROR PAST POST");
      rtc.clearIRQ();
      if(wakeInterval>=MIN_WAKE_INTERVAL){
        rtc.SetAlarmIRQ(wakeInterval*60);
      }else{
        rtc.SetAlarmIRQ(MIN_WAKE_INTERVAL*60);
      }
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }

    if(updateHandleDone[0]==1 && updateHandleDone[1]==1){
      readyToSleep=1;
    }
    if(readyToSleep){
      // wybudz po wakeInterval
      Serial.println("NORMAL END");
      rtc.clearIRQ();
      if(wakeInterval>=MIN_WAKE_INTERVAL){
        rtc.SetAlarmIRQ(wakeInterval*60);
      }else{
        rtc.SetAlarmIRQ(MIN_WAKE_INTERVAL*60);
      }
      digitalWrite(PIN_WAKE, LOW);
      delay(20);
    }

    delay(200);
  }

  vTaskDelete(NULL);
}

void calibrate(void *parameter){
  // todo

  updateHandleDone[1]=1;
  vTaskDelete(NULL);
}

void updateValues(void *parameter){
  if(receivedData.calibrate){
    xTaskCreatePinnedToCore(calibrate,"calibrate",2024,nullptr,1,&task1,1 );
  }else{
    updateHandleDone[1]=1;
  }

  bool dateChanged=0;

  if(receivedData.day!=-1 || receivedData.month!=-1 || receivedData.year!=-1){
    dateChanged=1;
    I2C_BM8563_DateTypeDef dateStruct;
    dateStruct.weekDay=1;
    
    if(receivedData.day!=-1){
      dateStruct.date=receivedData.day;
      acquiredData.day=receivedData.day;
    }else{
      dateStruct.date=acquiredData.day;
    }
    if(receivedData.month!=-1){
      dateStruct.month=receivedData.month;
      acquiredData.month=receivedData.month;
    }else{
      dateStruct.month=acquiredData.month;
    }
    if(receivedData.year!=-1){
      dateStruct.year=receivedData.year;
      acquiredData.year=receivedData.year;
    }else{
      dateStruct.year=acquiredData.year;
    }

    rtc.setDate(&dateStruct);

    fram.write(0x01, dateStruct.month);
    fram.write(0x02, dateStruct.date);
    uint8_t tempBuffer1[2];
    tempBuffer1[0] = dateStruct.year & 0xFF;
    tempBuffer1[1] = (dateStruct.year >> 8) & 0xFF;
    fram.write(0x03, tempBuffer1, 2);
  }

  if(receivedData.hours!=-1 || receivedData.minutes!=-1){
    dateChanged=1;
    I2C_BM8563_TimeTypeDef timeStruct;
    timeStruct.seconds=acquiredData.seconds;

    if(receivedData.hours!=-1){
      timeStruct.hours=receivedData.hours;
    }else{
      timeStruct.hours=acquiredData.hours;
    }
    if(receivedData.minutes!=-1){
      timeStruct.minutes=receivedData.minutes;
    }else{
      timeStruct.minutes=acquiredData.minutes;
    }

    rtc.setTime(&timeStruct);

    fram.write(0x05, timeStruct.hours);
    fram.write(0x06, timeStruct.minutes);
    fram.write(0x07, timeStruct.seconds);
  }

  for(;;){
    if(updateReceivedDone){
      break;
    }else{
      delay(20);
    }
  }

  if(dateChanged){
    xTaskCreatePinnedToCore(HTTPPost,"HTTPPost",4096,nullptr,1,&task1,1 );
  }
  
  /*
    FRAM Memory (32k):
    (...)  
    0x08 calibratedDist[0]
    0x09 calibratedDist[1]
    0x0a calibratedDist[2]
    0x0b calibratedDist[3]
    0x0c wakeInterval[0]
    0x0d wakeInterval[1]
    0x0e wakeInterval[2]
    0x0f wakeInterval[3]
    0x10 connectTime[0]
    0x11 connectTime[1]
    0x12 connectTime[2]
    0x13 connectTime[3]

  */

  if(receivedData.connectTime!=-1){
    if(receivedData.connectTime>=MIN_CONNECT_TIME){
      connectTime=receivedData.connectTime;
    }else{
      connectTime=MIN_CONNECT_TIME;
    }
    uint8_t tempBuffer2[4];
    tempBuffer2[0] = connectTime & 0xFF;
    tempBuffer2[1] = (connectTime >> 8) & 0xFF;
    tempBuffer2[2] = (connectTime >> 16) & 0xFF;
    tempBuffer2[3] = (connectTime >> 24) & 0xFF;
    fram.write(0x10, tempBuffer2, 4);

    Serial.print("Nowy czas na polaczenie: ");
    Serial.println(connectTime);
  }
  

  if(receivedData.wakeInterval!=-1){
    if(receivedData.wakeInterval>=MIN_WAKE_INTERVAL){
      wakeInterval=receivedData.wakeInterval;
    }else{
      wakeInterval=MIN_WAKE_INTERVAL;
    }
    uint8_t tempBuffer2[4];
    tempBuffer2[0] = wakeInterval & 0xFF;
    tempBuffer2[1] = (wakeInterval >> 8) & 0xFF;
    tempBuffer2[2] = (wakeInterval >> 16) & 0xFF;
    tempBuffer2[3] = (wakeInterval >> 24) & 0xFF;
    fram.write(0x0c, tempBuffer2, 4);

    Serial.print("Nowy interwal wybudzania: ");
    Serial.println(wakeInterval);
  }
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

  for(;;){
    if(calibratedDistReady){
      break;
    }else{
      delay(10);
    }
  }

  if(ToFMeasurement<ToFCalibratedDist*detectPrecision){
    acquiredData.detected=1;
  }else{
    acquiredData.detected=0;
  }
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
  battPercentage=((float)battValue[(int)round((VOLTAGE_MEASUREMENTS-1)/2)])/37;
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
        if(batteryDone && detectionDone && dateDone && timeDone){
          break;
        }else{
          delay(100);
        }
      }

      String messageDetected=String(acquiredData.detected);
      String messageBatt=String(acquiredData.batt);
      String messageDay=String(acquiredData.day);
      String messageMonth=String(acquiredData.month);
      String messageYear=String(acquiredData.year);
      String messageHours=String(acquiredData.hours);
      String messageMinutes=String(acquiredData.minutes);
      String messageSeconds=String(acquiredData.seconds);

      HTTPClient http;
      const String temp = ":";
      String url = serverUrl + temp + String(serverPort) + endpointPost;

      http.begin(url);
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      String postData="";
      postData += "messageDetected=" + messageDetected;
      postData += "&messageBatt=" + messageBatt;
      postData += "&messageDay=" + messageDay;
      postData += "&messageMonth=" + messageMonth;
      postData += "&messageYear=" + messageYear;
      postData += "&messageHours=" + messageHours;
      postData += "&messageMinutes=" + messageMinutes;
      postData += "&messageSeconds=" + messageSeconds;

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
        sscanf(response.c_str(), "%d,%d,%d,%d,%d,%d,%d,%d", &receivedData.day, &receivedData.month, &receivedData.year, &receivedData.hours, &receivedData.minutes, &receivedData.wakeInterval, &receivedData.connectTime, &receivedData.calibrate);
        
        xTaskCreatePinnedToCore(updateValues,"updateValues",2048,nullptr,1,&task1,1 );
        xTaskCreatePinnedToCore(HTTPUpdateReceived,"HTTPUpdateReceived",4096,nullptr,1,&task1,1 );

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
  updateDone=1;
  vTaskDelete(NULL);
}

void HTTPUpdateReceived(void *parameter){
  for(;;){
    if(WiFi.status() == WL_CONNECTED && updateDone){
      HTTPClient http;
      const String temp = ":";
      String url = serverUrl + temp + String(serverPort) + endpointUpdateReceived;

      http.begin(url);
      int httpCode = http.GET();
      if (httpCode > 0) {

        Serial.print("Update confirmed successfully. Response code: ");
        Serial.println(httpCode);

      } else {
        Serial.printf("HTTP Update Received failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
      break;
    }else{
      Serial.println("HTTP Update Received waiting for connection...");
      delay(100);
    }
  }
  updateReceivedDone=1;
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
    if(FRAMReady){
      break;
    }else{
      delay(10);
    }
  }
  /*
    FRAM Memory (32k):
      
    0x00 weekDay
    0x01 month
    0x02 date
    0x03 year[0]
    0x04 year[1]

    0x05 hours
    0x06 minutes
    0x07 seconds

    0x08 calibratedDist[0]
    0x09 calibratedDist[1]
    0x0a calibratedDist[2]
    0x0b calibratedDist[3]
    0x0c wakeInterval[0]
    0x0d wakeInterval[1]
    0x0e wakeInterval[2]
    0x0f wakeInterval[3]
    0x10 connectTime[0]
    0x11 connectTime[1]
    0x12 connectTime[2]
    0x13 connectTime[3]
  */
  I2C_BM8563_DateTypeDef dateStruct;
  I2C_BM8563_TimeTypeDef timeStruct;

  dateStruct.month=fram.read(0x01);
  dateStruct.date=fram.read(0x02);
  uint8_t tempBuffer[2];
  if(fram.read(0x03,tempBuffer,2)){
    dateStruct.year=(tempBuffer[1] << 8) | tempBuffer[0];
  }else{
    dateStruct.year=0;
  }

  timeStruct.hours=fram.read(0x05);
  timeStruct.minutes=fram.read(0x06);
  timeStruct.seconds=fram.read(0x07);

  uint8_t tempBuffer2[4];
  if(fram.read(0x08,tempBuffer2,4)){
    ToFCalibratedDist=(tempBuffer2[3] << 24) | (tempBuffer2[2] << 16) | (tempBuffer2[1] << 8) | tempBuffer2[0];
  }
  Serial.print("Calibrated dist: ");
  Serial.println(ToFCalibratedDist);
  calibratedDistReady=1;

  if(fram.read(0x0c,tempBuffer2,4)){
    wakeInterval=(tempBuffer2[3] << 24) | (tempBuffer2[2] << 16) | (tempBuffer2[1] << 8) | tempBuffer2[0];
  }
  Serial.print("Wake up interval: ");
  Serial.println(wakeInterval);

  if(fram.read(0x10,tempBuffer2,4)){
    connectTime=(tempBuffer2[3] << 24) | (tempBuffer2[2] << 16) | (tempBuffer2[1] << 8) | tempBuffer2[0];
  }
  Serial.print("Connect time: ");
  Serial.println(connectTime);

  for(;;){
    if(timeGot && dateGot){
      break;
    }else{
      delay(10);
    }
  }

  // jesli data z RTC jest mniejsza od tej z FRAM
  if(acquiredData.year<dateStruct.year || (acquiredData.year==dateStruct.year && acquiredData.month<dateStruct.month) || ((acquiredData.year==dateStruct.year && acquiredData.month==dateStruct.month && acquiredData.day<dateStruct.date))){
    //aktualizacja danych do wyslania
    acquiredData.day=dateStruct.date;
    acquiredData.month=dateStruct.month;
    acquiredData.year=dateStruct.year;
    acquiredData.seconds=timeStruct.seconds;
    acquiredData.minutes=timeStruct.minutes;
    acquiredData.hours=timeStruct.hours;

    //aktualizacja czasu RTC
    rtc.setDate(&dateStruct);
    rtc.setTime(&timeStruct);

  // jesli data jest rowna
  }else if(acquiredData.year==dateStruct.year && acquiredData.month==dateStruct.month && acquiredData.day==dateStruct.date){
    // jesli godzina jest mniejsza (precyzja do minuty)
    if(acquiredData.hours<timeStruct.hours || (acquiredData.hours==timeStruct.hours && acquiredData.minutes<timeStruct.minutes)){
      //aktualizuj czas
      acquiredData.seconds=timeStruct.seconds;
      acquiredData.minutes=timeStruct.minutes;
      acquiredData.hours=timeStruct.hours;
      rtc.setTime(&timeStruct);
    }else{
      // zapisz w fram czas z RTC
      fram.write(0x05, acquiredData.hours);
      fram.write(0x06, acquiredData.minutes);
      fram.write(0x07, acquiredData.seconds);
    }
  }else{
    //zapisz w fram z RTC
    fram.write(0x01, acquiredData.month);
    fram.write(0x02, acquiredData.day);
    uint8_t tempBuffer1[2];
    tempBuffer1[0] = acquiredData.year & 0xFF;
    tempBuffer1[1] = (acquiredData.year >> 8) & 0xFF;
    fram.write(0x03, tempBuffer1, 2);

    fram.write(0x05, acquiredData.hours);
    fram.write(0x06, acquiredData.minutes);
    fram.write(0x07, acquiredData.seconds);
  }

  dateDone=1;
  timeDone=1;
  
  vTaskDelete(NULL);
}

void RTCGet(void *parameter){
  I2C_BM8563_DateTypeDef dateStruct;
  I2C_BM8563_TimeTypeDef timeStruct;
  for(;;){
    rtc.getDate(&dateStruct);
    rtc.getTime(&timeStruct);

    acquiredData.day=dateStruct.date;
    acquiredData.month=dateStruct.month;
    acquiredData.year=dateStruct.year;
    dateGot=1;

    acquiredData.hours=timeStruct.hours;
    acquiredData.minutes=timeStruct.minutes;
    acquiredData.seconds=timeStruct.seconds;
    timeGot=1;

    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d\n",
                  dateStruct.year,
                  dateStruct.month,
                  dateStruct.date,
                  timeStruct.hours,
                  timeStruct.minutes,
                  timeStruct.seconds
                 );

    break;
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