//-----Libraries--------------------------------------------------
//#include <vector>
#include <SPI.h>       //SPI communication
#include <SD.h>        //sd card functionality
#include <SPIFlash.h>  //Flash library for the Moteino Mega flash_CS pin is D23 on this board, Open_Irr is configured with a 4 MBit (0.5 Megabyte) Flash chip.
// Note: if other SPI devices are present, ensure their CS pins are pulled up or set HIGH
#include <Wire.h>               //I2C functions for rtc
#include <OneWire.h>            //OneWire protocol for DS18B20
#include <DallasTemperature.h>  //To ease DS18b20 use
#include "time.h"               //for time elements
#include "RTClib.h"
#include <EEPROM.h>              //Built in EEPROM routines
#include <DS3232RTC.h>           //Precision (generic) RTC library for DS3232 rtc module
#include <Streaming.h>           //C++ style output with << operator, maybe not needed with new rtc code?
#include <LowPower.h>            //Low power functionality
#include <avr/sleep.h>           //Sleep Functions
#include "avr/io.h"              //Register set and bit values for Atmel chips
#include "avr/interrupt.h"       //Interrupt functionality
#include "avr/wdt.h"             //Watchdog timer
#include <CD74HC4067.h>          //For controlling Multiplexor
#include <RH_RF95.h>             //RadioHead library for the RF95 radio transceiver
#include <RHReliableDatagram.h>  //Additional Radiohead library for ease of transmission
#include <RadioString.h>         //Library to ease sending and receiving of large strings
#include <ArduinoJson.h>         //Json file format use
#include <ArduinoJson.hpp>       //Json file format use
#include <math.h>                //Mathmatical functions

//-----Assign Pins-------------------------------------------------
#define LED 15         //LED pin on Moteino Mega board
#define DS18B20_pin 1  //Data line for Ds18b20, note all ds18b20 sensors are on a common data line / pullup resistor

#define in1 12  //The four pins on the Moteino-Mega that can be used as an low-level output
#define in2 13
#define in3 14
#define in4 3

#define MAX_PACKET_LEN (RH_RF95_MAX_MESSAGE_LEN - 1)  //250
#define MAX_STRING_LEN (MAX_PACKET_LEN * (2 ^ 8))     //64000 bytes, one character in string/packet = 1 byte

#define WM_path1 26            //Pin for path1 of WM reading, to switch polarity
#define WM_path2 27            //Pin for path2 of WM reading, to switch polarity
#define WM_analog_read_pin A1  //Analog Pin (A1) for reading the WM sensors
#define mux_enable 22          //Pin for EN (enable) of the CD7HC4067 multiplexor
//mux select pins for s0,s1,s2,s3 are respecively digital pins 18,19,20,21
#define pin_mBatt 30         //Digital pin to activate battery voltage measurement circuit
#define pin_battV 31         //Analog pin to read battery voltage (31 ~ A7)
#define ADC_REF_VOLTAGE 3.3  //The reference output of the moteino mega board
#define ADC_RESOLUTION 1024  //The analog digital converter in moteino mega is 10 bit resolution
#define ADC_MAXVALUE 1023    //1024 possibilities including 0 (0-1023) on 10 bit ADC

#define SD_CS 28  //Chip select (CS) pin on sd card module

#define FLASH_SS 23  //Slave select (SS) pin for the flash chip on moteino mega -> uses SPI

#define RTC_Interrupt 24  //(A0) for RTC_SQW for interrupts


//-----Declare Global Variables-------------------------------------------

char filename[] = "000_Data.txt";  //sd card file name, 000 to be replaced by IDnum

uint16_t expectedDeviceID = 0xEF30;  //expected flash manufacturer ID of Moteino mega
//////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
//
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 64mbit flash
//////////////////////////////////////////

char a[4];  //char array for itoa function
byte i;     //Global for-loop counter

uint8_t radioID;                     //Have to have a placeholder for radioID
char Data[RH_RF95_MAX_MESSAGE_LEN];  //Placeholder to be filled with data to transmit
char Data2[RH_RF95_MAX_MESSAGE_LEN];
uint8_t int_buff[RH_RF95_MAX_MESSAGE_LEN];  //integer type array buffer
uint8_t length_buff;                        //integer denoting the size of the buffer
uint8_t from_id;                            //integer specifying the radio id that a transmission is coming from

uint16_t TRANSMIT_TIMEOUT = 2000;  //Set the minimum retransmit timeout. If sendtoWait is waiting for an ack longer than this time (in milliseconds), it will retransmit the message. Defaults to 200ms.
uint8_t TRANSMIT_POWER = 20;       //Set transmission power, 0-20,  defaults to 14
float RADIO_FREQUENCY = 915.0;     //Set frequency of radio module
uint16_t ROUTINE_TIMEOUT = 2000;   //Set delay at the end of the routine, too low and some packets will be missed, too high and you may also miss packets
uint8_t RETRY_NUM = 3;             //times to send packets without receiving ACK

float battV;           //battery voltage
float lowBatt = 4.0;   //low-battery limit
bool battLow = false;  //tracks if battery voltage falls below specified threshold

byte secs;  //time and date values, RTClib
byte mins;
byte hrs;
byte dow;
byte days;
byte mnths;
int yrs;
byte alarm_1_Mins;
tmElements_t tm;

boolean isMenuOn = false;
int menuinput;                //user input to menu prompt
long menutimeout;             //length of time to wait for user input
int indata;                   //user input data
int input;                    //for conversion of indata in getdata()
int numincoming;              //To indicate how many bytes are coming by user input
int incoming[7];              //Arrays are 0 indexed so this is enough room for 1 Binary character (1byte|8bits)
char charInput[200]{ '\0' };  //for charinput function
char incomingChar[200]{ '\0' };

const byte numChars = 32;  //for reading in character user input

int WM_group1_mean;  //Integer to hold WM_group means. Group 1 to 4 means will be attached to a corresponding relay in pin, 1-4
int WM_group2_mean;
int WM_group3_mean;
int WM_group4_mean;
int wm_grace_window = 10;  //Define the tolerance window (+ or - ,in kpa) for removing an individual sensor from the calculation of the group mean. Only triggers if pdiff from raw mean is >= 20.
bool force_irr = false;
bool new_irr_event = false;  //set true to write eeprom.object to moteino eeprom, thereby saving the record of the event

char local_time_irr_update[numChars]{ '\0' };  //Empty character array to dump local time stamp from unix time

String data = "";
String header = "";
String WM_string = "";
String irrigation_prompt_string = "";

String temperature_string = "";
float Temp;


//Define Error Log Structure//
struct error_log_struct {  // was typedef struct {}error_log_struct; 7/27/2023

  bool write_log;
  char write_time[18]{ '\0' };

  struct sd_log {
    bool card_begin_failure;
    bool open_file_failure;

  } sd_struct;

  struct flash_log {

  } flash_struct;

  struct rtc_log {
    bool rtc_begin_failure;
    bool rtc_lostPower_failure;
  } rtc_struct;

  struct pwr_log {
    bool bat_low;
  } pwr_struct;

  struct wm_log {
    //Short sensor will be "s", 115 as a decimal
    //Open sensor will be "o", 111 as a decimal
    //Default sensor will be "d", 100 as a decimal
    //Positive value will be "p", 112 as a decimal
    bool mux_channel_error[16]{ false };
    uint8_t mux_channel_error_code[16]{ '\0' };
  } wm_struct;

  struct ds18b20_log {
    //Open sensor (reading -127) will be "o", 111 as a decimal
    //sensor reset during conversion (reading 85) will be "r", 114 as a decimal
    //Cause of error (reading -98) is unknown at this time and will be "u", 117 as a decimal
    bool ds_unit_error[16]{ false };
    uint8_t ds_unit_error_code[16]{ '\0' };
  } ds18b20_struct;

  struct irr_log {
    //Force of event due to error count will be "e", 101 as a decimal
    //Force of event due to outlier count will be "o", 111 as a decimal
    //Prevention of a forced event due to timeframe (minimum time between not elapsed) will be "t", 116 as a decimal
    //Prevention of a forced event due to permission window will be "p", 112 as a decimal
    //Prevention of a forced event as raw mean is higher than set threshold will be "h", 104 as a decimal
    uint8_t forced_group_code[4]{ '\0' };
    bool forced_group[4]{ false };
  } irr_struct;

  struct radio_log {
    bool manager_init_failure;
    bool packet_failure;  //holder as currently we cannot tell if the packet failed, that prompt is given by library code...
  } radio_struct;
};

error_log_struct update_error_log;  //globally declared extra to reset the eeprom_object.error_log elements easily

const size_t capacity = 5 * JSON_ARRAY_SIZE(4) + 2 * JSON_OBJECT_SIZE(4) + 4 * JSON_OBJECT_SIZE(16) + 100;  // From assessment tool https://arduinojson.org/v6/assistant/
ArduinoJson::StaticJsonDocument<capacity> jsonBuffer;                                                       // Hopefully declaring once here is OK
//

//EEPROM structure method for storing and retrieving values from eeprom: relevant for variables defined in the menu
// Note that each EEPROM position can save only one byte of information, i.e. 8-bit numbers 0-255 and leading values (001) ARE NOT INTERPRETABLE
// but more than one position can be accessed using eeprom.put and eeprom.get with a defined structure
// this also enables saving different data types together
// Note that you CANNOT store/update pointers in EEPROM
//ATMEL EEPROM LIFESPAN ~ 100,000 read/writes
//ATMEL FLASH LIFESPAN ~ 10,000 read/writes

int eeprom_address = 0;

struct eeprom_struct {
  uint8_t nodeID;                       //nodeID for radio networking
  uint8_t IDnum;                        //numeric board identifier 0-255
  uint8_t gatewayID;                    //gatewayID for radio networking
  char projectID[numChars] = { '\0' };  //Project identifier

  boolean firstTime = true;  //flag for first time for writing to sdcard module

  boolean is_water_manager_on;  //flag to initiate drought stress management routine

  int group_irr_thresholds[4]{};   //as an array, for defining water threshold for groups 1 to 4, 0 to -255, integer type
  long min_time_btwn_irr[4]{ 0 };  //Array of type long for defining a minimum time between irrigation events, user enters value of minutes and timing must be called in milliseconds (minutes * 60 * 1000)
  long irr_period[4]{ 0 };         //Array of type long for defining the duration of an irrigation events

  boolean include_resistance;  //for specifying if resistance is to be included in written data string. true=1 false=0

  boolean calibration_resistor_present;  //Is a calibration resistor present?
  uint8_t cal_resistor_loc;              //Channel location of the calibration resistor between the Multiplexors
  float cal_resistor_val;                //Value of the calibration resistor installed
  int fixed_resistor_val;                //The value of the fixed resistor attached in series to the sensor and ground (voltage divider circuit), an unchanged schematic will be 10,000 ohms
  uint8_t num_WM;                        //Define the number of watermark sensors present

  uint8_t WM_group1[16]{ 0 };  //Array to hold mux channel location of WM sensors to average.
  uint8_t WM_group2[16]{ 0 };
  uint8_t WM_group3[16]{ 0 };
  uint8_t WM_group4[16]{ 0 };

  uint8_t num_ds18b20;         //Define the number of ds18b20 temperature sensors present
  uint8_t ds18b20_sensor0[8];  //Define locations to store ds18b20_sensor_addresses
  uint8_t ds18b20_sensor1[8];
  uint8_t ds18b20_sensor2[8];
  uint8_t ds18b20_sensor3[8];
  uint8_t ds18b20_sensor4[8];
  uint8_t ds18b20_sensor5[8];
  uint8_t ds18b20_sensor6[8];
  uint8_t ds18b20_sensor7[8];
  uint8_t ds18b20_sensor8[8];
  uint8_t ds18b20_sensor9[8];
  uint8_t ds18b20_sensor10[8];
  uint8_t ds18b20_sensor11[8];
  uint8_t ds18b20_sensor12[8];
  uint8_t ds18b20_sensor13[8];
  uint8_t ds18b20_sensor14[8];
  uint8_t ds18b20_sensor15[8];

  uint8_t ALARM_1_Interval = 1;  //Set the interval for alarm 1 (wake and run routine), default is 1

  uint8_t n_channels_wm_group1;  //Integer to hold # of channels utilized in each WM_group mean
  uint8_t n_channels_wm_group2;
  uint8_t n_channels_wm_group3;
  uint8_t n_channels_wm_group4;

  bool demo_mode;  //controls trouble shooting demo loop or the real loop

  bool toggle_radio;  //can turn off radio transmission when there is no gateway utilized

  uint32_t last_irr_unix_time[4]{};  // to store the unix time of the last irrigation event for each group.

  bool run_notes;  //to control printing of extraneous runtime data during the loop.

  bool latchingValve;

  error_log_struct error_log;  //nested array element with proper structure for error_log

  event_window irrigation_events[500];

  event_window permitted_widnows[500];
  event_window denied_windows[500];
  int num_events;
  //Vector<Vector<window>> permittedWindows; // Permitted windows for each group

  //vector<window> deniedWindows;
};

struct event_window{
  event_window_endpoint start;
  event_window_endpoint end;
  int seconds_interval[2];
};

struct event_window_endpoint{
  int day;
  int hour;
  int minute;
  int second;
};



eeprom_struct eeprom_object = {};  //Declare an object with the eeprom_struct structure, access objects as eeprom_object."element of struct without quotes"

unsigned long groupMillis[4];

enum States {
  IDLE,
  WAITING,
  IRRIGATING
};

States group_states[4];

bool group_is_done[4];

//-----Initialize-----------------------------------------------------
RTC_DS3231 rtc;

CD74HC4067 mux_1(18, 19, 20, 21);  //connect s0,s1,s2,s3 select pins on mux to specified digital pins
CD74HC4067 mux_2(18, 19, 20, 21);  //The same pins are connected to mux 2 for reading Watermark200ss

OneWire oneWire(DS18B20_pin);         //The Data pin for the DS18b20 temp sensors
DallasTemperature sensors(&oneWire);  //Pass OneWire reference to Dallas Temperature

DeviceAddress DS18B20_Address;  //Array to hold sensor addresses

RH_RF95 driver(4, 2);
RadioString manager(driver, radioID);  //Set up the radio manager, with RadioString library

File myfile;  //make sd card file object

//FlashTools flash;                           //initialize flash memory

//-----Sleep Functions----------------------------------------------

//Power saving functions referenced from ArduinoSoilH2O <https://github.com/ArduinoSoilH2O>

//Trying new rtclib update...
void setRTCInterrupt() {
  sei();                    //turn on Global Interrupt Enable
  PCMSK0 |= (1 << PCINT0);  //set A0/D24/Chippin 37 as PCINT
  PCICR |= (1 << PCIE0);    //enable interrupts on vector 0
}
void clearRTCInterrupt() {
  PCICR &= (1 << PCIE0);
}

ISR(PCINT0_vect) {
  sleep_disable();
  clearRTCInterrupt();
}

// Variables for testing scheduling
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);

  Wire.begin();  //enable I2C bus for rtc
  delay(100);
  SPI.begin();
  delay(300);

  //-----Pin settings-----

  pinMode(SD_CS, OUTPUT);  //CS pin for sd card

  pinMode(in1, OUTPUT);  //declare relay pins as outputs
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);  //Four channel relay we want normally lOW and activated when HIGH.
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pinMode(WM_path1, OUTPUT);       //WM Sensor Vs or GND
  pinMode(mux_enable, OUTPUT);     //enable or disable the multiplexor, the EN pin on multiplexor
  pinMode(WM_path2, OUTPUT);       //WM Sensor Vs or GND
  digitalWrite(mux_enable, HIGH);  //Disable the mux on setup

  pinMode(pin_mBatt, OUTPUT);
  digitalWrite(pin_mBatt, LOW);  //Leave the battery voltage circuit open to avoid battery drain

  //-----

  //battV = calcbattV();  // Get board battery level on startup

  //----- check chip EEPROM for stored data and place into "eeprom_object" with the structure of "eeprom_struct" declared earlier

  EEPROM.get(eeprom_address, eeprom_object);                                //eeprom_address may be redundant if only writing one eeprom object (i.e. it would always begin at position 0)
  int datasize_group1 = sizeof(eeprom_object.WM_group1) / sizeof(uint8_t);  //Hold the element length of the WM_group arrays, needs to be after eeprom_objects are read.
  int datasize_group2 = sizeof(eeprom_object.WM_group2) / sizeof(uint8_t);
  int datasize_group3 = sizeof(eeprom_object.WM_group3) / sizeof(uint8_t);
  int datasize_group4 = sizeof(eeprom_object.WM_group4) / sizeof(uint8_t);

  //----- Check SD card
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD card not present or card failure."));
    eeprom_object.error_log.sd_struct.card_begin_failure = true;
    eeprom_object.error_log.write_log = true;
  }

  //Flash chip routine here instead of sdcard?
  /*

  */


  //-----Set initial radio settings
  delay(10);
  driver.setFrequency(915.0);
  delay(10);
  driver.setTxPower(20, false);
  delay(10);
  manager.setThisAddress(eeprom_object.nodeID);  //Set the current address for radio communication in the manager
  delay(10);
  manager.setTimeout(2000);  //set timeout period where if an ACK not recieved it will retransmit message Default is 200ms, this will vary based on transmission packet length
  delay(1000);


  //-----RTC setup-----
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC, routine hang up"));
    eeprom_object.error_log.rtc_struct.rtc_begin_failure = true;
    eeprom_object.error_log.write_log = true;
    while (1)
      ;
  }

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power."));
    //Note that this might need changed if lostPower = true after a sleep cycle. a conditional test of Oscillator Stop Flag Bit7 in the status register. see this post  https://forum.arduino.cc/t/rtclib-reset-time-power-interrupted/639578/15
    eeprom_object.error_log.rtc_struct.rtc_lostPower_failure = true;
    eeprom_object.error_log.write_log = true;
  }

  Serial.println("Initialization Completed");

  setRTCInterrupt();
  set_alarms();
}

void loop() {
  // put your main code here, to run repeatedly:

  

  

  Serial.println(F("Woke up and checking for upcoming irrigation events"));
  // If within 8 seconds of irrigation event, begin irrigation
  // If rtc.now() of next irrigation event - rtc.now() < 8, then stay on, otherwise sleep
  open_valve(1);
  open_valve(2);
  open_valve(3);
  open_valve(4);
  Serial.println(F("No irrigation events occuring in the near future, going back to sleep"));
  set_alarms();
  Low_Power_Sleep();
}

void new_event_prompt(){
  Serial.println(F("Event Scheduling Menu Options:"));
  Serial.println(F("1 - Schedule a new window to permit irrigation."));
  Serial.println(F("2 - Schedule a new window to deny irrigation."));
  Serial.println(F("3 - Schedule a new irrigation window."))

  int choice;

  //window current_time_window = {};
  //Serial.println(F("Enter the time window in the following format: D HH:MM:SS - D HH:MM:SS"));
  long prompt_timeout = millis() + 10000;
  while (millis() < prompt_timeout) {
    if (Serial.available() > 0) {
      choice = Serial.parseInt();  //get user input
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }

  String window_start, window_end;
  Serial.println(F("Windows should be entered in the following format: DD HH MM SS - DD HH MM SS"));
  Serial.print(F("Enter the start of the window: "));
  window_start = Serial.readString();
  Serial.print(F("Enter the end of the window: "));
  window_end = Serial.readString();

  // Construct the window_endpoint objects
  event_window_endpoint window_start_endpoint;
  event_window_endpoint window_completion_endpoint;
  int start_day, end_day, start_hour, end_hour, start_minute, end_minute, start_second, end_second;

  start_endpoint.day = start_day;
  start_endpoint.hour = start_hour;
  start_endpoint.second = start_second;

  window_completion_endpoint.day = end_day;
  window_completion_endpoint.hour = end_hour;
  window_completion_endpoint.second = end_second;


  Serial.println(F(""));
  
  event_window()
  schedule_new_event(choice);
  



  // Prompt for start, read in string, split string, convert pieces into ints, store in struct
  // Do same for end
  
}

void schedule_new_event(int event_type, event_window_endpoint& start, event_window_endpoint& end){
  // createa new event using th3e endpoints
  // add the event to the respective vector
  event_window new_event();

  if (check_overlap(new_event)){
    return;
  }
  if (event_type == 1){
    eeprom_object.permitted_windows[num] = new_event;
  }
  else if (event_type == 2){
    eeprom_object.denied_windows[num] = new_event;
  }
  else{
    eeprom_object.irrigatin_events[num] = new_event;
  }

}

bool check_overlap(event_window){
  // Loop through window vector and check if seconds_from_start_of_week of event_window is between start and end of something in the vector
  // events array is in eeprom object
  for (int i = 0; i < eeprom_object.num_events; i++){
    if (event_window.seconds_interval[0] < eeprom_object.events[i].seconds_interval[1] && event_window.seconds_interval[1] > eeprom_object.events[i].seconds_interval[0]){
      return true;
    }
  }
  return false;
}

// If an upcoming event is found, return the integer index within the events array, otherwise return -1
int check_for_events(){
  // Get the current time
  // Loop through the windows vector
  // See if the current time + 8 lies within a window, since windows are disjoin, it can only be in at most one
  // Windows should be disjoint, it doesn't make sense to have two overlapping windows for the same group, only one thing at once - either irrigating or not

}


void set_alarms(){
  sei();
  wdt_disable();                         // turn off watchdog timer From ArduinoSoilH2O
  rtc.disable32K();                      // Turn off 32kHz output
  pinMode(RTC_Interrupt, INPUT_PULLUP);  // Making it so, that the alarm will trigger an interrupt

  //Schedule an alarm
  attachInterrupt(digitalPinToInterrupt(RTC_Interrupt), on_alarm, FALLING);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);  //stop oscillating signals at sqw pin
  rtc.disableAlarm(2);              //remove if using alarm2...
  if (!rtc.setAlarm1(
        rtc.now() + TimeSpan(8), DS3231_A1_Second  // i.e. tells it to match seconds to value in timespan + now...
        )) {
    Serial.println(F("Error, alarm wasn't set!"));
  } else {
    Serial.println(F("Alarm is set!"));
  }
}

void open_valve(int WM_group_num){
  // For future reference: if the input and output pins were saved in an array, we could avoid if else statements and allow for greater scalability
  if (WM_group_num == 1){
    digitalWrite(in1, HIGH);                                //open the respective relay pin, removing power to the pump
    delay(1000);
    digitalWrite(in1, LOW);                                //open the respective relay pin, removing power to the pump
  }
  else if (WM_group_num == 2){
    digitalWrite(in2, HIGH);                                //open the respective relay pin, removing power to the pump
    delay(1000);
    digitalWrite(in2, LOW);                                //open the respective relay pin, removing power to the pump
  }
  else if (WM_group_num == 3){
    digitalWrite(in3, HIGH);                                //open the respective relay pin, removing power to the pump
    delay(1000);
    digitalWrite(in3, LOW);                                //open the respective relay pin, removing power to the pump
  }
  else{
    digitalWrite(in4, HIGH);                                //open the respective relay pin, removing power to the pump
    delay(1000);
    digitalWrite(in4, LOW);                                //open the respective relay pin, removing power to the pump
  }
}

void on_alarm(){
  Serial.println(F("Alarm occured"));
}

void Low_Power_Sleep() {
  Serial.println(F("Going to sleep..."));
  // Serial.print(F("Current Measurement Interval: "));
  // Serial.print(eeprom_object.ALARM_1_Interval);
  // Serial.print(F("  Minutes. Next measurement to occur at:  "));
  // DateTime now = rtc.now();                           //needed to get unix time in next line
  // uint32_t current_unix_epoch_time = now.unixtime();  //get current unix epoch time
  // local_time(current_unix_epoch_time + (eeprom_object.ALARM_1_Interval * 60));
  // Serial.print(local_time_irr_update);
  // Serial.println();

  Serial.end();  //disable serial communication
  delay(10);
  digitalWrite(15, LOW);  //Turn off the LED pin (15 on moteino mega)
  delay(10);
  driver.sleep();  //Put radio to sleep
  delay(10);
  analogComp_off();  //turn off analog comparator
  delay(10);
  ADC_off();  //turn off ADC
  delay(10);
  JTAG_off();  //disable On-Chip Debug system
  delay(10);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //Power down completely on sleep
  delay(10);
  sleep_enable();
  delay(10);
  setRTCInterrupt();  //trying new rtclib update
  delay(100);
  sleep_mode();  //Puts MEGA to sleep

  //This happens after wake up Interrupt-----
  ADC_on();
  Serial.begin(9600);
  SPI.begin();
}

//------------- Power Savers ---------------------

void analogComp_off() {  //Turn off analog comparator
  ACSR &= ~(1 << 7);
}

void analogComp_on() {  //Turn on analog comparator
  ACSR |= (1 << 7);
}

void ADC_off() {  //Turn off ADC  --  this saves about 113 uA
  ADCSRA &= ~(1 << 7);
}

void ADC_on() {  //Turn on ADC
  ADCSRA |= (1 << 7);
}

void JTAG_off() {  //We don't ever need to turn it on for this application  //Must be executed twice within 4 clock cycles to disable JTAG!
  cli();
  MCUCR |= (1 << 7);
  MCUCR |= (1 << 7);
  sei();
}

void check_for_irrigation(){
  DateTime now = rtc.now();
  // get current time, look at day, hour, second, minute, second
  // compare to intervals
  uint16_t day = now.day();
  uint16_t hour = now.hour();
  uint16_t minute = now.minute();
  uint16_t second = now.second();

  // Can get epoch time, seconds from Jan 1, 1970, work modulo number of seconds in a week

  // Can convert day, hour, minute, second to seconds since start of week (Monday 12 AM?)
  // second + minute*60 + hour*60*60 + day*24*60*60

  // loop through the array/vector of time windows. Look at the start seconds and end seconds

  // if seconds_since_start_of_week >= time_windows[]
  
  // IF want interval from one week to spill over to the next, split the interval, go from start - MOnday 12:00 AM and then Monday 12:00 AM to end.
  
  
  
  // Format: YYYY MM DD HH MM SS
  // Read in as a string, split on spaces, convert each piece to int32
  // create 2 DateTime objects
  // Constructor: DateTime(YYYY, MM, DD, HH, MM, SS, MS)
  // uint32_t current_unix_epoch_time = now.unixtime();



  
  

}

