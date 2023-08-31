#include "LoRa_APP.h"               // Biblioteca LoRa https://github.com/HelTecAutomation/CubeCell-Arduino
#include "DistanceSensor_A02YYUW.h" // Biblioteca para Sensor Ultrassonico (Biblioteca Original https://github.com/pportelaf/DistanceSensor_A02YYUW)
#include <ArduinoJson.h>            // Biblioteca JSON https://github.com/bblanchon/ArduinoJson
#include "LoRa_Radio.h"             // Biblioteca auxiliar com métodos do rádio LoRa

#define TIME_TO_SLEEP 10                  // Tempo que o dispositivo dorme em cada ciclo (Em segundos)

#define devEUI "00:00:00:00:00:00:00:01"  // Endereço de identificação do dispositivo
#define boardName "Cubecell 1/2AA Node"   // Nome da placa utilizada
#define application "Water Tank"          // Nome da aplicação

#define measureTries 5                    // Número de tentativas que vai tentar ler o sensor até dar um valor válido

typedef enum
{
  LOWPOWER,
  SEND_WATERLEVEL
}States_t;

States_t state;

static TimerEvent_t wakeUp;

int distance = 0;
DistanceSensor_A02YYUW distanceSensor(&Serial1);
StaticJsonDocument<255> doc;


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
  jsonToCharArray(doc);
  sendPacket(jsonChArray);
}

// Método que envia uam sequenia de caracteres num pacote LoRa
void sendPacket(char* data){
  txNumber++;
  Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",data, strlen(data));
  Radio.Send( (uint8_t *)data, strlen(data) );
}

// Método que coloca o dispositivo a dormir 
void sleep(){
  TimerSetValue( &wakeUp, TIME_TO_SLEEP * 1000 );
  TimerStart( &wakeUp );
  lowPowerHandler();
}

// Método que é chamado quando o dispositivo acorda
void onWakeUp(){
  state = SEND_WATERLEVEL;
}

// Método de inicialização do dispositivo
void setup() {
  Serial.begin(115200);             // Inicialização da comunicação Serial
  Serial1.begin(9600);              // Inicialização da comunicação Serial com o Sensor
  txNumber=0;
  Rssi=0;
  
  radioSetup();                     // Inicialização do rádio LoRa
  TimerInit( &wakeUp, onWakeUp );   // Inicializar o timer para o dispositivo dormir
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
      state = LOWPOWER;
      break; 
  }
  delay(500);
}