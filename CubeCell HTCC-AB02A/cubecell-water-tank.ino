#include "Arduino.h"
#include "LoRa_APP.h"
#include "LoRa_Radio.h"
#include <softSerial.h>
#include "DistanceSensor_A02YYUW.h"
#include <ArduinoJson.h>

#define timetillwakeup                              5000
#define devEUI                                      "00:00:00:00:00:00:00:01"

typedef enum
{
  LOWPOWER,
  RX,
  SEND_WATERLEVEL
}States_t;

States_t state;

static TimerEvent_t wakeUp;

unsigned char data[4]={};
int distance;
DistanceSensor_A02YYUW distanceSensor(&Serial1);

int readSensor(){
  DistanceSensor_A02YYUW_MEASSUREMENT_STATUS meassurementStatus;

  // Gets the distance from the sensor and if the measurement is wrong, it retries to get the distance
  do {
    meassurementStatus = distanceSensor.meassure();

    if (meassurementStatus == DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK) {
      return distanceSensor.getDistance();
    } else {
      return meassurementStatus;
    }
  } while (meassurementStatus != DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK);
}

void sendJsonData(){
  StaticJsonDocument<250> doc;
  doc["devEUI"] = devEUI;
  doc["application"] = "Water Tank";
  doc["board"] = "CubeCell 1/2AA Node (HTCC-AB02A)";
  int sensorReadTries = 0;
  do{
    distance = readSensor();
    sensorReadTries++;
  }while(distance == -4 && sensorReadTries < 5);
  doc["data"]["distance"] = distance;
  jsonToCharArray(doc);
  sendPacket(jsonChArray);
}

void sendPacket(char* data){
  txNumber++;
  Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",data, strlen(data));
  Radio.Send( (uint8_t *)data, strlen(data) );
}
      
void sleep(){
  TimerSetValue( &wakeUp, timetillwakeup );
  TimerStart( &wakeUp );
  lowPowerHandler();
}

void onWakeUp(){
  state = SEND_WATERLEVEL;
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); 
  txNumber=0;
  Rssi=0;
  
  radioSetup();
  TimerInit( &wakeUp, onWakeUp );
  state = SEND_WATERLEVEL;                                  
}

void loop()
{
  switch (state){
    case SEND_WATERLEVEL:
      sendJsonData();  
      break;
    case RX:
      Radio.Rx(0);
      state = LOWPOWER;
      break;
    case LOWPOWER:
        sleep();
      break;
    default: 
      break; 
  }
  delay(1000);
}