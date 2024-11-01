#define num_valves 10

int valve_pin = 10;
int pulse_width = 50;
long valve_open_time;

enum Valve_States{
  OPEN, CLOSED
};

int valve_pins[NUM_VALVES];
Valve_States valve_states[];

void toggle_valve(int pin, int valve_number){

  // __|---|___
  digitalWrite(pin, HIGH);
  valve_open_time = millis();
  // Probably switch to a blocking timer here
  // pulse width might be too small to go and do something else in the meantime, so blocking seems fine here?
  while (millis() < valve_open_time + pulse_width){
  }
  digitalWrite(valve_pin, LOW);
  if (valve_states[valve_number] == OPEN){
    valve_states[valve_number] = CLOSED;
  }
  else{
    valve_states[valve_number] = OPEN;
  }
}

int procedure(){
  // Given a valve_id, return the integer procedure
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  digitalWrite(valve_pin, LOW);

  valve_pins[0] = 10;
}

void loop() {
  // put your main code here, to run repeatedly:

  // Open valve
  toggle_valve(valve_pin);

  if (valve_states[valve_number] == OPEN){
    Serial.println("Valve opened");
  }
  // Wait for some amount of seconds

  toggle_valve(valve_pin);
  if (valve_states[valve_number] == OPEN){
    Serial.println("Valve closed");
  }

  // With acutal events, we can toggle_valve when event begins and toggle_valve when event ends
  // Essentially the old digitalWrites would just be replaced with more general opening and closing logic like above


}
