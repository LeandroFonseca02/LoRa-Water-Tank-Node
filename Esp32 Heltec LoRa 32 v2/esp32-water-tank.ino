#include <SPI.h>                    // Biblioteca SPI do Arduino necessária para a comunicação com o modulo LoRa
#include <LoRa.h>                   // Biblioteca LoRa https://github.com/sandeepmistry/arduino-LoRa
#include <ArduinoJson.h>            // Biblioteca JSON https://github.com/bblanchon/ArduinoJson
#include <SoftwareSerial.h>         // Biblioteca SoftwareSerial https://github.com/plerup/espsoftwareserial/
#include "DistanceSensor_A02YYUW.h" // Biblioteca para Sensor Ultrassonico (Biblioteca Original https://github.com/pportelaf/DistanceSensor_A02YYUW)

// Pinos do rśdio LoRa
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BAND 868E6                        // Frequência do rádio LoRa
#define SPREADING_FACTOR 7                // Spreading Factor do rádio LoRa
#define PREAMBLE_LENGTH 8                 // Preamble Length do rádio LoRa
#define BANDWIDTH 125E3                   // Bandwidth do rádio LoRa
#define CODING_RATE 7                     // Coding Rate do rádio LoRa
#define SYNC_WORD 0x12                    // Sync Word do rádio LoRa

#define devEUI "00:00:00:01:01:01:01:01"  // Endereço de identificação do dispositivo
#define boardName "Heltec Lora 32"        // Nome da placa utilizada
#define application "Water Tank"          // Nome da aplicação

#define TIME_TO_SLEEP 60                  // Tempo que o dispositivo dorme em cada ciclo (Em segundos)

#define txSensorPin 37                    // Pino onde o Tx do Sensor está conectado
#define rxSensorPin 36                    // Pino onde o Rx do Sensor está conectado

#define measureTries 5                    // Número de tentativas que vai tentar ler o sensor até dar um valor válido


typedef enum
{
  LOWPOWER,
  SEND_WATERLEVEL
}States_t;

States_t state;

StaticJsonDocument<255> doc;
SoftwareSerial softSerial(txSensorPin, rxSensorPin);
DistanceSensor_A02YYUW distanceSensor(&softSerial);

int distance = 0;


// Método de inicialização do rádio LoRa
void initLoRa(){
  // Definir os pinos do rádio LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
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
  doc["devEUI"] = devEUI;
  doc["application"] = application;
  doc["board"] = boardName;

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
}

// Método que envia a String num pacote LoRa
void sendPacket(String data){
  LoRa.beginPacket();
  LoRa.print(data);
  LoRa.endPacket();
}

// Método que coloca o dispositivo a dormir 
void sleep(){
  esp_deep_sleep_start();
}


// Método de inicialização do dispositivo
void setup() {
  Serial.begin(115200);                                 // Inicialização da comunicação Serial
  softSerial.begin(9600);                               // Inicialização da comunicação Serial com o Sensor
  initLoRa();                                           // Inicialização do rádio LoRa

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP*1000000); // Inicializar o timer para o dispositivo dormir

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
      Serial.println("Sleeping during " + String(TIME_TO_SLEEP) + " seconds");
      delay(100);
      sleep();
      break;
    default: 
      break; 
  }
}