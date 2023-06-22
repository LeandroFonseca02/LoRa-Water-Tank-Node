#include <ArduinoJson.h>

#define BUFFER_SIZE                                 250

char jsonChArray[BUFFER_SIZE];
StaticJsonDocument<BUFFER_SIZE> receivedJson;

void jsonToCharArray(StaticJsonDocument<BUFFER_SIZE> doc);

void jsonToCharArray(StaticJsonDocument<BUFFER_SIZE> doc){
  String json;
  serializeJson(doc, json);
  json.toCharArray(jsonChArray, BUFFER_SIZE);
}

void stringToJson(String data){
  
  String receivedPacket = String(data);
  receivedPacket = receivedPacket.substring(1,receivedPacket.length());
  DeserializationError error = deserializeJson(receivedJson, receivedPacket);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
}