/*
  Para programar ligar a um módulo FTDI
  FDTI      Lora Node
  
  DTR       DTR
  TX        RX
  RX        TX
  VCC       3V3
  GND       GND

*/

#include <SPI.h>                    // Biblioteca SPI do Arduino necessária para a comunicação com o modulo LoRa
#include <LoRa.h>                   // Biblioteca LoRa https://github.com/sandeepmistry/arduino-LoRa
#include <ArduinoJson.h>            // Biblioteca JSON https://github.com/bblanchon/ArduinoJson
#include <SoftwareSerial.h>         // Biblioteca SoftwareSerial do Arduino necessária para a comunicação Serial com o Sensor
#include <RTClib.h>                 // Biblioteca RTC https://github.com/adafruit/RTClib
#include "LowPower.h"               // Biblioteca LowPower https://github.com/LowPowerLab/LowPower
#include "DistanceSensor_A02YYUW.h" // Biblioteca para Sensor Ultrassonico (Biblioteca Original https://github.com/pportelaf/DistanceSensor_A02YYUW)


#define BAND 868E6                        // Frequência do rádio LoRa
#define SPREADING_FACTOR 7                // Spreading Factor do rádio LoRa
#define PREAMBLE_LENGTH 8                 // Preamble Length do rádio LoRa
#define BANDWIDTH 125E3                   // Bandwidth do rádio LoRa
#define CODING_RATE 7                     // Coding Rate do rádio LoRa
#define SYNC_WORD 0x12                    // Sync Word do rádio LoRa

#define devEUI "00:00:00:00:00:00:01:01"  // Endereço de identificação do dispositivo
#define boardName "LoRa Radio Node v1"    // Nome da placa utilizada
#define application "Water Tank"          // Nome da aplicação

#define CLOCK_INTERRUPT_PIN 3             // Pino em que SQW do RTC está conectado no qual o alarme vai ser disparado (Pino digital)
#define TIME_TO_SLEEP 60                  // Tempo que o dispositivo dorme em cada ciclo (Em segundos)

#define txSensorPin A0                    // Pino onde o Tx do Sensor está conectado
#define rxSensorPin A1                    // Pino onde o Rx do Sensor está conectado

#define measureTries 5                    // Número de tentativas que vai tentar ler o sensor até dar um valor válido

typedef enum
{
  LOWPOWER,
  SEND_WATERLEVEL
}States_t;

States_t state;

RTC_DS3231 rtc;
StaticJsonDocument<200> doc;
SoftwareSerial mySerial(txSensorPin,rxSensorPin);
DistanceSensor_A02YYUW distanceSensor(&mySerial);

int distance = 0;


// Método de inicialização do rádio LoRa
void initLoRa(){
  // Parametrização do rádio
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setSyncWord(SYNC_WORD); 
  
  // Verifica a comunicação com o rádio LoRa
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}


// Método que é chamado ao disparar o alarme do RTC
void onAlarm() {
    //Serial.println("Alarm occured!");
}

// Método de inicialização do RTC
void setupRTC(int interruptPin){

  // Inicialização do RTC
  if(!rtc.begin()) {
      Serial.println("Couldn't find RTC!");
      Serial.flush();
      while (1) delay(10);
  }

  if(rtc.lostPower()) {
      // Coloca a data de compilação no RTC
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Desativa o Pino que produz uma squarewave a 32KHz
  rtc.disable32K();

  // Inicializar o pin que dispara o alarme
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onAlarm, FALLING);

  // Desativar as flags dos alarmes do RTC
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  // Desativar a produção de squarewave para poder usar o pino como trigger do alarme
  rtc.writeSqwPinMode(DS3231_OFF);

  // Desativar o alarme 1
  rtc.disableAlarm(1);

  //Serial.println("RTC initialized");
}

// Método que desativa o alarme
void clearAlarm(){
  if (rtc.alarmFired(2)) {
      rtc.clearAlarm(2);
      //Serial.print(" - Alarm cleared");
  }
}

// Método que programa o alarme para um intervalo de tempo
void setAlarm(int seconds){
  if(!rtc.setAlarm2(
            rtc.now() + TimeSpan(seconds),
            DS3231_A2_Hour // Modo do alarme 2 que dispara quando as horas e os minutos forem iguais
    )) {
        //Serial.println("Error, alarm wasn't set!");
    }else {
        /*Serial.print("\nAlarm will happen in ");
        Serial.print(seconds);
        Serial.print(" seconds!");*/
    }

}

// Método que envia a String num pacote LoRa
void sendPacket(String data){
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();
}

// Método que recebe a distancia do sensor ou em caso de erro o Status
int readSensor(){
  DistanceSensor_A02YYUW_MEASSUREMENT_STATUS meassurementStatus;

  // Recebe o estado do sensor e caso esteja pronto recebe a distância
  do {
    meassurementStatus = distanceSensor.meassure();

    if (meassurementStatus == DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK) {
      return distanceSensor.getDistance();
    } else {
      return meassurementStatus;
    }
  } while (meassurementStatus != DistanceSensor_A02YYUW_MEASSUREMENT_STATUS_OK);
}

// Método que envia a distância do sensor
void sendJsonData(){
  mySerial.begin(9600);
  doc["devEUI"] = devEUI;
  doc["application"] = boardName;
  doc["board"] = application;

  int sensorReadTries = 0;
  // Leitura do sensor
  do{
    distance = readSensor();
    sensorReadTries++;
  }while(distance == -4 && sensorReadTries < measureTries);

  doc["data"]["distance"] = distance;

  String json;
  serializeJson(doc, json);
  Serial.println(json);
  sendPacket(json);   // Envia pacote LoRa
  mySerial.end();
}


// Método que é chamado ao acordar do dispositivo
void wakeUp(){
}

// Método de inicialização do dispositivo
void setup() {
  Serial.begin(57600);                      // Inicialização da comunicação Serial
  initLoRa();                               // Inicialização do rádio LoRa
  pinMode(CLOCK_INTERRUPT_PIN, INPUT);      // Colocar o pino do alarme em modo Input
  setupRTC(CLOCK_INTERRUPT_PIN);            // Inicializar o módulo RTC

  state = SEND_WATERLEVEL;
}

// Método de funcionamento do dispositivo
void loop() {
  switch (state){

    // Estado que envia a distância do sensor
    case SEND_WATERLEVEL:
      sendJsonData();
      state = LOWPOWER;
      break;
    
    // Estado do dispositivo que o dispositivo entra em modo low power
    case LOWPOWER:
      Serial.println("Going to sleep for " + String(TIME_TO_SLEEP/60) + " minutes");
      setAlarm(TIME_TO_SLEEP);                                                  // Programa o alarme para acordar o dispositivo
      attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), wakeUp, LOW); // Ativa a interrupção para quando o alarme disparar
      delay(100);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);                      // Coloca o dispositivo a dormir
      detachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN));              // Desativa a interrupção do alarne
      clearAlarm();                                                             // Desativa o alarme
      state = SEND_WATERLEVEL;
      break;
    default:
      state = LOWPOWER; 
      break; 
  }
}
