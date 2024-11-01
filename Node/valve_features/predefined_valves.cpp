#include "predefined_valves.h"
// #include <ArduinoJson.h>         //Json file format use
// #include <ArduinoJson.hpp>       //Json file format use

//ArduinoJson::StaticJsonDocument<200> predefined_valves_doc;

void setup_predefined_valves(){
  //json_doc = JsonDocument::to<JsonObject>();
  JsonObject json_obj = json_doc.to<JsonObject>();
  //JsonObject predefinedValvesArray = predefinedValvesDoc.createNestedObject("predefinedValvesArray");
  JsonArray predefined_valves_array = json_obj.createNestedArray("predefinedValvesArray");
  
  //predefinedValvesArray.createNestedArray("valve0");
  //predefinedValvesArray_valve0.add(5000);

  //{{"id": 0, "model": "something", ...}, {"id": 1, "model": "something", ...}, {}};
  //predefinedValvesArray = predefinedValvesDoc.to<JsonArray>();
  JsonObject valve0 = predefined_valves_array.createNestedObject();
  valve0["id"] = 0;
  valve0["model"] = "VALVE-LAT-NPT-0_75";
  valve0["link"] = "https://www.vegetronix.com/Products/g/Valves/VALVE-LAT-NPT-0_75/";
  valve0["power"] = "9VDC";
  valve0["valve_fittings"] = "3/4NPT";
  valve0["latching"] = true;
  valve0["idle_state_normally_closed"] = true;
  valve0["duty_cycle_type"] = "pulse";
  valve0["duty_cycle_ms"] = 50;
  valve0["wiring_three_way"] = false;
  valve0["pressure_min_psi"] =  15;
  valve0["pressure_max_psi"] = 150;
  valve0["resistance_ohms"] = 5.5;

  JsonObject valve1 = predefined_valves_array.createNestedObject();
  valve1["id"] = 1;
  valve1["model"] = "GCS3052";
  valve1["link"] = "https://www.dripdepot.com/irrigation-valve-with-dc-latching-solenoid-size-three-quarter-inch-fpt";
  valve1["power"] = "15-24VDC";
  valve1["valve_fittings"] = "3/4to2FPT";
  valve1["latching"] = true;
  valve1["idle_state_normally_closed"] = true;
  valve1["duty_cycle_type"] = "pulse";
  valve1["duty_cycle_ms"] = 50;
  valve1["wiring_three_way"] = true;
  valve1["pressure_min_psi"] =  7;
  valve1["pressure_max_psi"] = 145;
  valve1["resistance_ohms"] = 10;

  JsonObject valve2 = predefined_valves_array.createNestedObject();
  valve2["id"] = 2;
  valve2["model"] = "GCS3051";
  valve2["link"] = "https://www.dripdepot.com/irrigation-valve-with-dc-latching-solenoid-size-three-quarter-inch-fpt";
  valve2["power"] = "6-18VDC";
  valve2["valve_fittings"] = "3/4to2FPT";
  valve2["latching"] = true;
  valve2["idle_state_normally_closed"] = true;
  valve2["duty_cycle_type"] = "pulse";
  valve2["duty_cycle_ms"] = 50;
  valve2["wiring_three_way"] = false;
  valve2["pressure_min_psi"] =  7;
  valve2["pressure_max_psi"] = 145;
  valve2["resistance_ohms"] = 3;

  JsonObject valve3 = predefined_valves_array.createNestedObject();
  valve3["id"] = 3;
  valve3["model"] = "BVS6CE-XR22";
  valve3["link"] = "https://www.electricsolenoidvalves.com/1-inch-stainless-steel-motorized-electric-ball-valve-2-wire-auto-return/?gad_source=1&gclid=Cj0KCQjw-uK0BhC0ARIsANQtgGOcgHHqUHF3baPzmg-jInNEEL0i7T6fMIOtIkSZg0ez5QPZPoyaFCcaAs7aEALw_wcB#downloads";
  valve3["power"] = "9-24VAC/VDC";
  valve3["valve_fittings"] = "1NPT";
  valve3["latching"] = false;
  valve3["bidirectional_flow"] = true;
  valve3["idle_state_normally_closed"] = true;
  valve3["powerloss_return_to_state_logic"] = true;
  valve3["holding_current_mA"] = 500;
  valve3["duty_cycle_type"] = "continuous";
  valve3["duty_cycle_ms"] = 60000;
  valve3["duty_cycle_ratio"] = 100;
  valve3["wiring_three_way"] = false;
  valve3["pressure_min_psi"] =  0;
  valve3["pressure_max_psi"] = 185;

  JsonObject valve4 = predefined_valves_array.createNestedObject();
  valve4["id"] = 3;
  valve4["model"] = "BVS6CE-XR22";
  valve4["link"] = "https://www.electricsolenoidvalves.com/1-inch-stainless-steel-motorized-electric-ball-valve-2-wire-auto-return/?gad_source=1&gclid=Cj0KCQjw-uK0BhC0ARIsANQtgGOcgHHqUHF3baPzmg-jInNEEL0i7T6fMIOtIkSZg0ez5QPZPoyaFCcaAs7aEALw_wcB#downloads";
  valve4["power"] = "9-24VAC/VDC";
  valve4["valve_fittings"] = "1NPT";
  valve4["latching"] = false;
  valve4["bidirectional_flow"] = true;
  valve4["idle_state_normally_closed"] = true;
  valve4["powerloss_return_to_state_logic"] = true;
  valve4["holding_current_mA"] = 500;
  valve4["duty_cycle_type"] = "continuous";
  valve4["duty_cycle_ms"] = 60000;
  valve4["duty_cycle_ratio"] = 100;
  valve4["wiring_three_way"] = false;
  valve4["pressure_min_psi"] =  0;
  valve4["pressure_max_psi"] = 185;

  serializeJsonPretty(predefined_valves_array, Serial);
}