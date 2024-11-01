#ifndef predefined_valves_h
#define predefined_valves_h

#include <ArduinoJson.h>         //Json file format use
#include <ArduinoJson.hpp>       //Json file format use

//JsonObject predefinedValvesArray;
//extern ArduinoJson::JsonObject predefinedValvesArray;
//extern ArduinoJson::StaticJsonDocument<200> predefined_valves_doc;
extern ArduinoJson::JsonDocument json_doc;
extern ArduinoJson::JsonArray predefined_valves_array;


extern void setup_predefined_valves();

#endif