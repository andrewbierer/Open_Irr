//-----Libraries--------------------------------------------------
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
#include "predefined_valves.h"


//-----Assign Pins-------------------------------------------------
#define LED 15         //LED pin on Moteino Mega board
#define DS18B20_pin 1  //Data line for Ds18b20, note all ds18b20 sensors are on a common data line / pullup resistor

/*
Plan to remove macros for in1, in2, etc. and replace with new int array
*/

int io_pins[] = {12, 13, 14, 3};

#define in1 12  //The four pins on the Moteino-Mega that can be used as an low-level output
#define in2 13
#define in3 14
#define in4 3

// ins = [12, 13, 14, 3]

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

#define NUM_SECONDS_IN_WEEK 7*24*60*60

//-----Declare Global Variables-------------------------------------------

char filename[] = "000_Data.txt";  //sd card file name, 000 to be replaced by IDnum

char irrigation_events_data[] = "irr_data.txt";
char permitted_events_data[] = "per_data.txt";
char den_events_data[] = "den_data.txt";

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

// Fixme: Plan to replace with an array
int wm_group_means[4];
// int WM_group1_mean;  //Integer to hold WM_group means. Group 1 to 4 means will be attached to a corresponding relay in pin, 1-4
// int WM_group2_mean;
// int WM_group3_mean;
// int WM_group4_mean;

int wm_grace_window = 10;  //Define the tolerance window (+ or - ,in kpa) for removing an individual sensor from the calculation of the group mean. Only triggers if pdiff from raw mean is >= 20.
bool force_irr = false;
bool new_irr_event = false;  //set true to write eeprom.object to moteino eeprom, thereby saving the record of the event

char local_time_irr_update[numChars]{ '\0' };  //Empty character array to dump local time stamp from unix time

// Fixme: Clearing all strings to release memory previously, but now want to utilize ArduinoJson
String data = "";
String header = "";
String WM_string = "";
String irrigation_prompt_string = "";

String temperature_string = "";
float Temp;

// Fixme: ArduinoJson might be an easier/more reliable way to handle this
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

char temp_json_data[10000];


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
  // Planned Change: plan to remove this since it's not needed
  // uint8_t IDnum;                        //numeric board identifier 0-255
  uint8_t gatewayID;                    //gatewayID for radio networking
  char projectID[numChars] = { '\0' };  //Project identifier

  // Planned Change: header not needed anymore since we are using JSON
  // boolean firstTime = true;  //flag for first time for writing to sdcard module

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

  /*
  Planned Change
  In the future, we can add a const called NUM_GROUPS for the array sizing
  */
  uint8_t WM_groups[4][16];
  // uint8_t WM_group1[16]{ 0 };  //Array to hold mux channel location of WM sensors to average.
  // uint8_t WM_group2[16]{ 0 };
  // uint8_t WM_group3[16]{ 0 };
  // uint8_t WM_group4[16]{ 0 };

  /*
  Planned Change
  */
  uint8_t num_ds18b20;         //Define the number of ds18b20 temperature sensors present
  uint8_t ds18b20_sensor_addresses[16][8];
  // uint8_t ds18b20_sensor0[8];  //Define locations to store ds18b20_sensor_addresses
  // uint8_t ds18b20_sensor1[8];
  // uint8_t ds18b20_sensor2[8];
  // uint8_t ds18b20_sensor3[8];
  // uint8_t ds18b20_sensor4[8];
  // uint8_t ds18b20_sensor5[8];
  // uint8_t ds18b20_sensor6[8];
  // uint8_t ds18b20_sensor7[8];
  // uint8_t ds18b20_sensor8[8];
  // uint8_t ds18b20_sensor9[8];
  // uint8_t ds18b20_sensor10[8];
  // uint8_t ds18b20_sensor11[8];
  // uint8_t ds18b20_sensor12[8];
  // uint8_t ds18b20_sensor13[8];
  // uint8_t ds18b20_sensor14[8];
  // uint8_t ds18b20_sensor15[8];

  uint8_t ALARM_1_Interval = 1;  //Set the interval for alarm 1 (wake and run routine), default is 1

  /*
  Planned Change
  */
  uint8_t n_channels_per_wm_group[4];
  // uint8_t n_channels_wm_group1;  //Integer to hold # of channels utilized in each WM_group mean
  // uint8_t n_channels_wm_group2;
  // uint8_t n_channels_wm_group3;
  // uint8_t n_channels_wm_group4;

  bool demo_mode;  //controls trouble shooting demo loop or the real loop

  bool toggle_radio;  //can turn off radio transmission when there is no gateway utilized

  uint32_t last_irr_unix_time[4]{};  // to store the unix time of the last irrigation event for each group.

  bool run_notes;  //to control printing of extraneous runtime data during the loop.

  bool latchingValve;

  error_log_struct error_log;  //nested array element with proper structure for error_log

  // Initally all -1, if an event is upcoming for group i, the 
  int current_events[4];

  int num_to_update = 0;

  // 2D array with match fields, matchFields[0][i] are the match fields for hours, matchFields[1][i] the match fields for minutes, and matchFields[2][i] are the match fields for seconds
  // had to change uint16_t to int to avoid type errors with assignment using indata
  uint8_t matchFields[3][8];    //arbitrarily length 24? how many match fields are reasonable to expect?
  uint8_t numFields[3]; // Stores the number of fields for seconds, minutes, and hours respectively

  uint16_t num_events = 0;

  // The queue would store the IDs of events
  int events_queue[50];
  int events_queue_size = 0;
  uint16_t current_event_reference_number = 0;

  //valve valves[10];
  int current_valve_id = 0;
  int num_valves = 0;
  int valve_group_id_lookup[4];


};

eeprom_struct eeprom_object = {};  //Declare an object with the eeprom_struct structure, access objects as eeprom_object."element of struct without quotes"

unsigned long group_millis[4];

enum event_state {
  IDLE,
  WAITING,
  IRRIGATING
};

event_state curr_state;

// new states: IDLE, WAITING, IRRIGATING, RADIO, 

event_state group_states[4];

bool group_is_done[4];

// Planned Change: New variables for compile_json

int channel;
int WM1_CB = 0;  //Holder for WM sensor value in CB/kPa, direction 1
double WM1_Resistance = 0;
int global_WM_group_num;
int global_WM_group_mean;

String global_last_irr_starting_time;
String global_last_irr_ending_time;

float global_temp_values[16];


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

// Radio events, static events that need to be shceduled
// Avoid scheduling certain events at the same time

// Might be able to track state of valve, when we pulse and irrigation event not ever, can't schedule another event

// Find times to next pulse for each of the non latching valves that are also irrigating: t1, t2, etc.
// Find the smallest


// If the smallest is within 200ms, then exit current function and pulse quickly
// void pulse_non_latching_valves(){
//   // Loop through all the active valves

//   // See if valve is non latching

//   // If time now is less than x ms from next pulse, pulse

// }

// Stores the correct valve for group i (value is the index of the valve in valves)
// int valveLookupTable[4];

// One option - 
// Another option - file stuff


// Initially timeEvaluationConsideration
struct event{
  int event_reference_number;
  // The irrigation groups that the event is scheduled for
  bool groups[4]; // Makes sense to be here since if we have an array in eeprom, it's more difficult to update indices when removing
  //int group;
  // Could use pointers or references instead
  bool recurring;              //false = singular
  // bool permit;                 //false = deny
  uint8_t event_type;           //0=measurement, 1=irrigation, 2=permit window, 3=deny window....
  
  //uint16_t matchFields[24];    //arbitrarily length 24? how many match fields are reasonable to expect?

  DateTime* span[2];  //DateTime classes for start and end dates/times for a particular event

  event(uint16_t event_reference_number, bool groups[4], bool recurring, uint8_t event_type, DateTime* span[2]): event_reference_number(event_reference_number), recurring(recurring), event_type(event_type){
    for (int i = 0; i < 4; i++){
      this->groups[i] = groups[i];
    }
    this->span[0] = span[0];
    this->span[1] = span[1]; 
  }

  event(): recurring(false), event_type(-1){}

  void print(){
    Serial.print(F("Group(s): "));
    for (int i = 0; i < 4; i++){
      if (groups[i]){
        Serial.print(i+1);
        Serial.print(" ");
      }
    }
    Serial.println("");
    Serial.print(F("Recurring: "));
    if (recurring){
      Serial.println("Y");
    }
    else{
      Serial.println("N");
    }
    Serial.print(F("Event Type: "));
    Serial.println(event_type);
    if (recurring){
      print_recurring_event_span();
    }
    else{
      print_singular_event_span();
    }
    Serial.println();
    Serial.println();
  }

  void print_singular_event_span(){
    Serial.println(F("Start"));
    Serial.print(F("Year: "));
    Serial.println(span[0]->year());
    Serial.print(F("Month: "));
    Serial.println(span[0]->month());
    Serial.print(F("Day: "));
    Serial.println(span[0]->day());
    Serial.print(F("Hour: "));
    Serial.println(span[0]->hour());
    Serial.print(F("Minute: "));
    Serial.println(span[0]->minute());
    Serial.print(F("Second: "));
    Serial.println(span[0]->second());
    
    Serial.println(F("End"));
    Serial.print(F("Year: "));
    Serial.println(span[1]->year());
    Serial.print(F("Month: "));
    Serial.println(span[1]->month());
    Serial.print(F("Day: "));
    Serial.println(span[1]->day());
    Serial.print(F("Hour: "));
    Serial.println(span[1]->hour());
    Serial.print(F("Minute: "));
    Serial.println(span[1]->minute());
    Serial.print(F("Second: "));
    Serial.println(span[1]->second());
  }

  void print_recurring_event_span(){
    Serial.println(F("Start"));
    Serial.print(F("Day: "));
    Serial.println(span[0]->dayOfTheWeek());
    Serial.print(F("Hour: "));
    Serial.println(span[0]->hour());
    Serial.print(F("Minute: "));
    Serial.println(span[0]->minute());
    Serial.print(F("Second: "));
    Serial.println(span[0]->second());
    
    Serial.println(F("End"));
    Serial.print(F("Day: "));
    Serial.println(span[1]->dayOfTheWeek());
    Serial.print(F("Hour: "));
    Serial.println(span[1]->hour());
    Serial.print(F("Minute: "));
    Serial.println(span[1]->minute());
    Serial.print(F("Second: "));
    Serial.println(span[1]->second());
  }

};

event* events[500];

int global_num_events = 0;

void set_alarms(int interval_seconds){
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
        rtc.now() + TimeSpan(interval_seconds), DS3231_A1_Second  // i.e. tells it to match seconds to value in timespan + now...
        )) {
    Serial.println(F("Error, alarm wasn't set!"));
  } else {
    Serial.println(F("Alarm is set!"));
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

// void check_for_irrigation(){
//   DateTime now = rtc.now();
//   // get current time, look at day, hour, second, minute, second
//   // compare to intervals
//   uint16_t day = now.day();
//   uint16_t hour = now.hour();
//   uint16_t minute = now.minute();
//   uint16_t second = now.second();

//   // Can get epoch time, seconds from Jan 1, 1970, work modulo number of seconds in a week

//   // Can convert day, hour, minute, second to seconds since start of week (Monday 12 AM?)
//   // second + minute*60 + hour*60*60 + day*24*60*60

//   // loop through the array/vector of time windows. Look at the start seconds and end seconds

//   // if seconds_since_start_of_week >= time_windows[]
  
//   // IF want interval from one week to spill over to the next, split the interval, go from start - MOnday 12:00 AM and then Monday 12:00 AM to end.
  
  
  
//   // Format: YYYY MM DD HH MM SS
//   // Read in as a string, split on spaces, convert each piece to int32
//   // create 2 DateTime objects
//   // Constructor: DateTime(YYYY, MM, DD, HH, MM, SS, MS)
//   // uint32_t current_unix_epoch_time = now.unixtime();
// }

//-----Get User input as integer-----------------------------------------------
void get_integer_input() {
  menutimeout = millis() + 60000;  // time to wait for user to input something, was 10 secs
  int sign = 1;                    // for handling negatives
  indata = 0;                      // initialize
  while (millis() < menutimeout)   // wait for user to input something
  {
    if (Serial.available() > 0)  // something came in to serial buffer
    {
      delay(100);                         // give time for everything to come in
      numincoming = Serial.available();   // number of incoming bytes (1 byte = 1 HEX (ASCII) character or 8bits of binary)
      for (i = 1; i <= numincoming; i++)  // read in everything
      {
        incoming[i] = Serial.read();                 // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)  // ignore carriage return & line feed
        {
        } else if (incoming[i] == '-') {                      //if a minus is read in
          sign = -1;                                          //update sign to neagative 1
        } else if (incoming[i] >= '0' && incoming[i] <= '9')  // otherwise
        {
          input = incoming[i] - '0';            // convert ASCII value to ??numerical equivalent?? decimal.
          indata = indata * 10 + input * sign;  // assemble to get total value if sequence of numbers
        }
      }
      break;  // exit before menutimeout
    }
  }
  Serial.println(indata);  //return entered value for visual user feedback
  delay(10);
}

//-----Get User Input for Character variables etc.-----------------------------------

void charinput() {
  memset(charInput, 0, sizeof(charInput));
  delay(50);
  long timeout;
  timeout = millis() + 60000;  //length of time to wait for user input, was 10sec
  byte numincoming;            //for # of bytes incoming

  while (millis() < timeout) {
    if (Serial.available() > 0) {
      delay(100);
      numincoming = Serial.available();
      for (byte i = 0; i <= numincoming; i++) {
        incomingChar[i] = Serial.read();
        if (incomingChar[i] == 13 || incomingChar[i] == 10) {
        } else {
          charInput[i] = incomingChar[i];
        }
      }
      charInput[numincoming] = 0;
      break;
    }
  }
  Serial.println(charInput);
}

void updateEEPROM() {
  // FixMe: Include the comparator for EEPROM to not overwrite too frequently if number of events don't change
  // Use strcmp
  eeprom_address = 0;                         //clear eeprom_address
  EEPROM.put(eeprom_address, eeprom_object);  //update chip EEPROM if there are any changes from what was saved...
  eeprom_address = 0;                         //clear eeprom_address
  Serial.println(F("EEPROM updated"));
}



//-----Store data on micro sd card----------------------------------

void writeFileSD(const char *path, const char *message) {


  Serial.print(F("writeFileSD path: "));  //limited to 13 characters including extension
  Serial.println(path);

  File file = SD.open(path, FILE_WRITE);  // This works when the name of the error log saved was changed.... we might be running into filename length restrictions here, but not sure why not on the esp32...
  delay(10);
  if (!file) {
    Serial.println(F("- failed to open file for writing"));
    return;
  }
  if (file.println(message)) {
    Serial.println(F("- file written"));
  } else {
    Serial.println(F("- write failed"));
  }
  file.close();
}

void set_radio(){
  //-----Set initial radio settings
  delay(10);
  driver.setFrequency(915.0);
  delay(10);
  driver.setTxPower(20, false);
  delay(10);
  manager.setThisAddress(eeprom_object.nodeID);  //Set the current address for radio communication in the manager
  delay(10);
  manager.setTimeout(2000);  //set timeout period where if an ACK not recieved it will retransmit message Default is 200ms, this will vary based on transmission packet length
  delay(10);
}

void check_rtc(){
  //-----RTC setup-----
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC, routine hang up"));
    eeprom_object.error_log.rtc_struct.rtc_begin_failure = true; // saved
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
}

//------Calc battery voltage (0- ~ 10V) with voltage divider circuit------------- Closed/opened by mosfet with digital input to prevent battery drain.
float calcbattV() {
  pinMode(pin_mBatt, OUTPUT);  //Set digital pin as an output to activate the circuit
  pinMode(pin_battV, INPUT);   //Set the other pin as input to read the voltage

  digitalWrite(pin_mBatt, HIGH);  //Turn the gate of the MOSFET HIGH, closing the circuit
  delayMicroseconds(500);         //delay time for stabilization

  uint16_t vInt;                 //The raw integer returned by analogRead
  vInt = analogRead(pin_battV);  //read vInt on analog pin connected where the resistors in the divider circuit meet.
  digitalWrite(pin_mBatt, LOW);  //Turn the gate of the MOSFET LOW, deactivating the circuit

  float vout = 0.00;
  float vin = 0.00;
  float R1 = 10000;  //1st Resistor in divider circuit taking load from battery
  float R2 = 4700;   //2nd Resistor in divider circuit leading to board GND

  vout = (vInt * ADC_REF_VOLTAGE) / ADC_RESOLUTION;  //voltage modified by divider circuit, voltage at dividing point
  vin = vout / (R2 / (R1 + R2));                     //Actual voltage coming into the divider circuit, i.e. battery level
  return vin;
}

void latchingValveModuleReturnToIdle() {
  //default/idle condition to save on current consumption
 
  //##SINGLE##
  // digitalWrite(in1, LOW);  //Pin states during IDLE state
  // digitalWrite(in2, LOW);
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, LOW);
  // Serial.println(F("Default/Idle condition to save on current consumption."));
 
  //Multiple WORKS
  for (int i = 0; i < 4; i++){
    digitalWrite(io_pins[i], LOW);  //Pin states during IDLE state
  }
  Serial.println(F("Default/Idle condition to save on current consumption."));
}

void setup() {
  // Fixme: Think about only beginning serial if device is connected
  Serial.begin(9600);

  Wire.begin();  //enable I2C bus for rtc
  SPI.begin();

  //-----Pin settings-----

  pinMode(SD_CS, OUTPUT);  //CS pin for sd card

  /*
  The new program uses the io_pins array as opposed to in1, in2, etc.
  */

  for (int i = 0; i < 4; i++){
    pinMode(io_pins[i], OUTPUT);
  }

  latchingValveModuleReturnToIdle();

  pinMode(WM_path1, OUTPUT);       //WM Sensor Vs or GND
  pinMode(mux_enable, OUTPUT);     //enable or disable the multiplexor, the EN pin on multiplexor
  pinMode(WM_path2, OUTPUT);       //WM Sensor Vs or GND
  digitalWrite(mux_enable, HIGH);  //Disable the mux on setup

  pinMode(pin_mBatt, OUTPUT);
  digitalWrite(pin_mBatt, LOW);  //Leave the battery voltage circuit open to avoid battery drain

  //-----

  battV = calcbattV();  // Get board battery level on startup

  //----- check chip EEPROM for stored data and place into "eeprom_object" with the structure of "eeprom_struct" declared earlier

  EEPROM.get(eeprom_address, eeprom_object);                                //eeprom_address may be redundant if only writing one eeprom object (i.e. it would always begin at position 0)
  int datasize_groups[4];

  /*
  Planned Change
  */

  for (int i = 0; i < 4; i++){
    datasize_groups[i] = sizeof(eeprom_object.WM_groups[i]) / sizeof(uint8_t);  //Hold the element length of the WM_group arrays, needs to be after eeprom_objects are read.
  }

  // int datasize_group1 = sizeof(eeprom_object.WM_group1) / sizeof(uint8_t);  //Hold the element length of the WM_group arrays, needs to be after eeprom_objects are read.
  // int datasize_group2 = sizeof(eeprom_object.WM_group2) / sizeof(uint8_t);
  // int datasize_group3 = sizeof(eeprom_object.WM_group3) / sizeof(uint8_t);
  // int datasize_group4 = sizeof(eeprom_object.WM_group4) / sizeof(uint8_t);

  //----- Check SD card
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD card not present or card failure."));
    eeprom_object.error_log.sd_struct.card_begin_failure = true;
    eeprom_object.error_log.write_log = true;
  }

  //Flash chip routine here instead of sdcard?
  /*

  */

  set_radio();
  check_rtc();

  // SD.remove("events.txt");
  // writeFileSD("events.txt", "Hello World");

  // // Prints the file contents, useful for debugging
  // myfile = SD.open("events.txt", FILE_READ);
  // while (myfile.available()) {  // read file and print to Serial COM port, Note this will be slow with alot of data due to chip limitations. A desktop with a chip reader is nearly instantaneous.
  //   Serial.write(myfile.read());
  // }
  // myfile.close();

  //test_read_and_write_events_data();

  read_events_data();


  events_menu();
  write_events_data();
  updateEEPROM();

  
  Serial.println("Initialization Completed");
  
  
  setRTCInterrupt();
  set_alarms(8);

  // Ready to incorporate into main menu modulo some small bugs
  //events_menu();
  // The number of reads/writes on eeprom is limited, so might want to think of other ways to store information like number of events
  // The realtime clock might have some memory
  updateEEPROM();
  
  //menu();
  // events menu in open_irr, continue on valve_menu and this file
}

void loop() {
  Serial.println(F("Woke up and checking for upcoming irrigation events"));
  
  
  // Read in current windows for permit, deny, and irrigation
  
  // Reading in the data from JSON
  // Print once events data is read successfully
  read_events_data();

  // Comarator to see if JSON has changed (possibly to cut down read/writes)
  // reading is free on chip but writing has a cost
  // read and write only if events have changed
  // Remove events that have been done and see if new events are there and write them
  //fill_old_events_data();

  // Now we have the events array filled with predefined events

  // // If within 8 seconds of irrigation event, begin irrigation
  // Planned Change: rename it to check_measurement_match_fields

  // If we have a match field, we need to add a new singular event to events array
  // Before the next time the device is off, we want these singular events processed
  
  events_loop();

  // while (1){
  //   Serial.println(F("Program hanging"));
  // }

  // // // If rtc.now() of next irrigation event - rtc.now() < 8, then stay on, otherwise sleep
  // // open_valve(1); // saved
  // // open_valve(2);
  // // open_valve(3);
  // // open_valve(4);
  // //Serial.println(F("No irrigation events occuring in the near future, going back to sleep"));
  // Write current windows for permit, deny and irrigation
  //events_menu();

  //write_data_to_sd();
  //set_alarm_1_interval();
  // We could compare old EEPROM data to new data using character array like we did for the events data
  //updateEEPROM();
  Low_Power_Sleep();
}

void write_data_to_sd(){
  write_events_data();
  write_valve_data();
}

void read_data_from_sd(){
  read_valve_data();

}

void readRTC() {
  DateTime now = rtc.now();
  secs = now.second();
  mins = now.minute();
  hrs = now.hour();
  days = now.day();
  mnths = now.month();
  yrs = now.year();
}


// FIXME: Refactor to avoid global strings
//-----Print user interpretable time from unix time-----------------------------------------
void local_time(uint32_t unix_time) {

  byte hold_min;
  byte hold_sec;

  String temp = "";
  /* //If you just wanted the time printed to serial monitor...
     Serial.print(month(unix_time));
    Serial.print(F("/"));
    Serial.print(day(unix_time));
    Serial.print(F("/"));
    Serial.print(year(unix_time));
    Serial.print(F("  "));

    Serial.print(hour(unix_time));
    Serial.print(F(":"));
    hold_min = minute(unix_time);
    if (hold_min < 10) {
      Serial.print('0');
    }
    Serial.print(hold_min);
    //Serial.print(minute(unix_time));
    Serial.print(F(":"));
    hold_sec = second(unix_time);
    if (hold_sec < 10) {
      Serial.print('0');
    }
    Serial.print(hold_sec);
    //Serial.print(second(unix_time));
  */


  byte month_i = month(unix_time);
  byte day_i = day(unix_time);
  int year_i = year(unix_time);
  byte hour_i = hour(unix_time);
  byte minute_i = minute(unix_time);
  byte second_i = second(unix_time);

  temp += month_i;
  temp += '/';
  temp += day_i;
  temp += '/';
  temp += year_i;
  temp += ' ';
  temp += hour_i;
  temp += ':';
  if (minute_i < 10) {
    temp += '0';
  }
  temp += minute_i;
  temp += ':';
  if (second_i < 10) {
    temp += '0';
  }
  temp += second_i;
  temp.toCharArray(local_time_irr_update, numChars);  //Update the global irr_update variable
}

void print_board_info(){
  //print out board information-----
  Serial.println();
  Serial.println(F("Open_Irr: soil water tension management system"));
  Serial.print(F("Project ID:  "));

  for (uint8_t x = 0; x < numChars; x++) {
    if (eeprom_object.projectID[x] != '\0') {
      Serial.print(eeprom_object.projectID[x]);
    } else {
      break;  //testing
    }
  }
  Serial.println();

  // Planned Change: This will be removed since IDnum will be removed
  // Serial.print(F("Board ID: "));
  // Serial.println(eeprom_object.IDnum);

  Serial.print(F("Node ID: "));
  Serial.println(eeprom_object.nodeID);
  Serial.print(F("Gatway ID:  "));
  Serial.println(eeprom_object.gatewayID);

  Serial.print(F("Data file name: "));
  Serial.println(filename);

  readRTC();
  Serial.print(F("Current date: "));
  Serial.print(mnths);
  Serial.print('-');
  Serial.print(days);
  Serial.print('-');
  Serial.println(yrs);

  Serial.print(F("Current time:  "));
  Serial.print(hrs);
  Serial.print(':');
  if (mins < 10) {
    Serial.print('0');
  }
  Serial.print(mins);
  Serial.print(':');
  if (secs < 10) {
    Serial.print('0');
  }
  Serial.println(secs);

  Serial.print(F("Current Battery Voltage:  "));
  battV == calcbattV();
  delay(10);
  Serial.print(battV);
  Serial.println(F("V"));

  Serial.print(F("Current Measurement Interval: "));
  Serial.print(eeprom_object.ALARM_1_Interval);
  Serial.print(F("  Minutes. Next measurement to occur at:  "));
  DateTime now = rtc.now();                           //needed to get unix time in next line
  uint32_t current_unix_epoch_time = now.unixtime();  //get current unix epoch time
  local_time(current_unix_epoch_time + (eeprom_object.ALARM_1_Interval * 60));
  Serial.print(local_time_irr_update);
  Serial.println();

  Serial.print(F("Number of WaterMark sensors currently installed:  "));
  Serial.println(eeprom_object.num_WM);
  Serial.print(F("Include Resistance Values in output:  "));
  Serial.println(eeprom_object.include_resistance);
  Serial.println(F("Current Sensor Grouping for Means:  "));

  for (int i = 0; i < 4; i++){
    Serial.print(F("Group"));
    Serial.println(i+1);
    Serial.print(F(": "));
    for (int j = 0; j < 16; j++) {
      if (eeprom_object.WM_groups[i][j] >= 0 && eeprom_object.WM_groups[i][j] != 255) {
        Serial.print(eeprom_object.WM_groups[i][j]);
        Serial.print(F("  "));
      }
    }
    Serial.println();
  }
  
  Serial.print(F("Number of temperature sensors connected:  "));
  Serial.println(eeprom_object.num_ds18b20);

  Serial.print(F("Water Manager Status: "));
  Serial.println(eeprom_object.is_water_manager_on);
  Serial.println();

  for (int q = 0; q < 4; q++) {
    Serial.print(F("Group "));
    Serial.println(q + 1);

    Serial.print(F("Matric potential threshold for irrigation events: "));
    Serial.println(eeprom_object.group_irr_thresholds[q]);

    Serial.print(F("Time of last irrigation event: "));
    local_time(eeprom_object.last_irr_unix_time[q]);  //convert unix timestamp to local time (stored in global var), then print it.
    Serial.println(local_time_irr_update);

    Serial.print(F("Minimum time between irrigation events (in minutes): "));
    Serial.println(eeprom_object.min_time_btwn_irr[q]);

    // Probably add the unit
    Serial.print(F("Duration of irrigation events: "));
    Serial.println(eeprom_object.irr_period[q]);

    Serial.println();
  }

  Serial.print(F("Radio Transmission is:  "));

  if (eeprom_object.toggle_radio) {
    Serial.println(F("ON."));
  } else {
    Serial.println(F("OFF."));
  }

  Serial.print(F("Troubleshooting/Demo mode is: "));
  if (eeprom_object.demo_mode) {
    Serial.println(F("ON."));
  } else {
    Serial.println(F("OFF."));
  }
}

void print_menu_options(){
  Serial.println(F("Menu Actions:"));                                                   // Define menu Actions--------------------------------------------
  Serial.println(F("   c  <--  Set clock"));                                            // "99" Set RTC clock
  Serial.println(F("   i  <--  Set ID numbers"));                                       // "105" set IDnum, nodeID, gatewayID
  Serial.println(F("   a  <--  Set Alarm (measurement) Interval"));                     // "97" Set the alarm intervals for the RTC
  Serial.println(F("   g  <--  Toggle Radio Transmissions (ON/OFF)"));                  // "103" Enable or disable LoRa Radio transmisison
  Serial.println(F("   h  <--  Toggle Troubleshooting (continuous) mode"));             // "104" Enter Troubleshooting continuous loop
  Serial.println(F("   b  <--  Identify connected ds18b20 sensors"));                   // "98" menu routine for iteratively connecting temperature sensors and saving them to eeprom
  Serial.println(F("   t  <--  Test measurements"));                                    // "116" take test measurements from sensors -> not currently used
  Serial.println(F("   s  <--  Switch Water Manager"));                                 // "115" Toggle/Switch Water Manager routine
  Serial.println(F("   w  <--  Define water threshold values and times"));              // "119" Define threshold to trigger an event
  Serial.println(F("   n  <--  Specify number of Watermark sensors and Mean groups"));  // "110" Set number of WM sensors installed & specify groups based on channel position of multiplexor? Could also write funciton to subtract 1 from numbers specified so that actual terminal numbers can be used.
  Serial.println(F("   o  <--  Specification of resistors for WM circuit"));            // "111" Define resistor information
  Serial.println(F("   p  <--  Set output HIGH on pinouts 1 to 4"));                    // "112" For priming/testing irrigation equipment
  //  Serial.println(F("   r  <--  Download range of data"));                         // "114" set beginning date to download, not functional
  Serial.println(F("   f  <--  Display microSD card information"));  // "102" Get sdcard information, non functional
  Serial.println(F("   d  <--  Download all data"));                 // "100" get all data to serial port
  Serial.println(F("   e  <--  Erase all data"));                    // "101" erase all data
  //  Serial.println(F("   m  <--  Repeat menu"));                                    // "109" repeat menu, not functional
  Serial.println(F("   x  <--  Exit"));  // "120" exit
}

void identify_temperature_sensors(){
  Serial.println(F("Iterative identification of ds18b20 temperature sensors."));
  get_integer_input();  //otherwise the first input is always 0?
  Serial.println(F("If you would like to continue, press 1. To return to main menu, press any other character."));
  get_integer_input();
  // do not specify int user_resp = indata; // for some reason it does not allow menu access, define the integer then change it.
  int user_resp;
  delay(100);
  user_resp = indata;
  delay(100);
  if (user_resp != 1) {
    menu();
  } else {
    Serial.println(F("Continuing with iterative identificaiton of ds18b20 temperature sensors..."));
    delay(1000);
    Serial.println(F("How many ds18b20 are being connected?"));
    get_integer_input();
    eeprom_object.num_ds18b20 = indata;

    if (eeprom_object.num_ds18b20 == 0) {
      Serial.println(F("As 0 external temperature sensors are being connected, temperature correction will be made using the Real Time Clock inside the environmental enclosure."));
      menu();
    }

    Serial.print(F("User must complete identification of "));
    Serial.print(eeprom_object.num_ds18b20);
    Serial.println(F(" ds18b20 temperature sensors."));
    Serial.println(F("Prepare to plug in sensors..."));
    delay(2000);
    Identify_1WireDevices();
  }
}

void set_clock(){
  Serial.println(F("Set clock:  "));
  get_integer_input();  //otherwise the first input is always 0?
  Serial.print(F("  input month: "));
  get_integer_input();
  mnths = indata;
  Serial.print(F("  input day:    "));
  get_integer_input();
  days = indata;
  Serial.print(F("  input year:   "));
  get_integer_input();
  yrs = indata;
  Serial.print(F("  input hour:   "));
  get_integer_input();
  hrs = indata;
  Serial.print(F("  input minute: "));
  get_integer_input();
  mins = indata;
  Serial.print(F(" input second: "));
  get_integer_input();
  secs = indata;

  rtc.adjust(DateTime(yrs, mnths, days, hrs, mins, secs));
  delay(50);
}

void pump_priming_prompt(){
  get_integer_input();  //otherwise the first input is always 0?
  Serial.flush();
  Serial.println(F("Are you sure you want to cycle the pumps?"));
  Serial.println(F("Make sure the lines are oriented where outflow is desired!"));
  Serial.println(F("Type YES to confirm priming of pumps"));

  charinput();
  if (charInput[0]) {
    char answer[4]{ 0 };
    byte i = 0;
    for (int i = 0; i < 3; i++) {
      if (charInput[i] != 0) {
        answer[i] = charInput[i];
      }
    }
    Serial.print(F("Answer: "));
    Serial.println(answer);
    if (strcmp(answer, "YES") == 0) {
      Serial.println(F("Priming Pumps..."));

      digitalWrite(in1, HIGH);  //provide power to relay/switch on respective pin, relays are configured active when HIGH. pumps should be normally open & circuit closed when powered
      delay(10000);
      digitalWrite(in1, LOW);  //power off
      delay(2000);
      digitalWrite(in2, HIGH);
      delay(10000);
      digitalWrite(in2, LOW);
      delay(2000);
      digitalWrite(in3, HIGH);
      delay(10000);
      digitalWrite(in3, LOW);
      delay(2000);
      digitalWrite(in4, HIGH);
      delay(10000);
      digitalWrite(in4, LOW);
      Serial.println(F("Pump priming routine completed."));
    } else {
      Serial.println(F("Answer was not YES, pump priming failed to initialize."));
    }
  }
}

void radio_prompt(){
  Serial.println(F("Would you like to enable radio transmisison?"));
  get_integer_input();  //otherwise first is always 0?
  Serial.println(F("Type 1 to enable radio transmissions or 0 to disable radio transmissions."));
  get_integer_input();
  eeprom_object.toggle_radio = indata;

  if (indata != 1) {
    Serial.println(F("Radio transmissions DISABLED."));
    driver.sleep();  //Puts radio to sleep.
  } else {
    Serial.println(F("Radio transmissions ENABLED."));
    RH_RF95 driver(4, 2);
    RHReliableDatagram manager(driver, radioID);  //Set up the radio manager
    if (!driver.init()) {
      Serial.println(F("Radio initialization failed."));
    }
    if (driver.init()) {
      Serial.println(F("Radio initilization successful."));
    }
  }
}

void troubleshooting_prompt(){
  Serial.println(F("Would you like to enter troublesooting mode?"));
  Serial.println(F("Troubleshooting mode does not save data to sdcard but DOES send radio transmissions."));
  Serial.println();
  Serial.println(F("You may also toggle the printing of extra runtime notes to help with troubleshooting."));
  Serial.println(F("This can be done in Troubleshooting mode or during normal operation."));
  get_integer_input();  //otherwise first is always 0?
  delay(2000);

  Serial.println(F("Would you like to enable runtime notes?"));
  Serial.println(F("Enter 1 to enable runtime notes or 0 to disable runtime notes."));
  Serial.print(F("Current runtime notes Setting:  "));
  if (eeprom_object.run_notes) {
    Serial.println(F("ON."));
  } else {
    Serial.println(F("OFF."));
  }
  get_integer_input();

  if (indata == 1 || indata == 0) {
    eeprom_object.run_notes = indata;
  } else {
    Serial.println(F("Invalid Entry, returning to main menu."));
    menu();
  }
  delay(2000);

  Serial.println(F("If you would like to enable troubleshooting mode enter 1, otherwise enter 0 to enable regular operation."));
  Serial.println(F("Enter any other key to return to the main menu."));
  get_integer_input();

  if (indata == 1 || indata == 0) {
    eeprom_object.demo_mode = indata;
  } else {
    menu();
  }

  if (eeprom_object.demo_mode) {  //if demo_mode is true (1)...
    Serial.println(F("Troubleshooting mode enabled."));
    delay(1000);
  } else {
    Serial.println(F("Regular operation enabled."));
    delay(1000);
  }
}

void erase_sd(){
  Serial.println(F("Erase data on sd card..."));
  Serial.print(F("Currently Writing to: "));
  Serial.println(filename);
  Serial.print(F("Do you want to delete: "));
  Serial.print(filename);
  Serial.println(F("? Type YES to confirm deletion of this file."));

  get_integer_input();  //Otherwise first input is always 0?
  Serial.flush();
  charinput();
  if (charInput[0]) {
    char erase[4]{ 0 };
    byte i = 0;
    for (int i = 0; i < 3; i++) {
      if (charInput[i] != 0) {
        erase[i] = charInput[i];
      }
    }
    Serial.print(F("erase:  "));
    Serial.println(erase);
    if (strcmp(erase, "YES") == 0) {
      SD.remove(filename);
      Serial.println(F("Data File deleted"));
      // Planned Change: This will be removed since firstTime will be removed
      // eeprom_object.firstTime = true;  //Return firsttime to true so header is printed when first writing to file
    } else {
      Serial.println(F("If condition not satisfied"));
    }
  }
}

void set_project_id(){
  Serial.print(F(" Project ID:     "));  // get projectID up to numChars length
  Serial.flush();
  charinput();
  if (charInput[0]) {
    byte i = 0;
    for (int i = 0; i < numChars; i++) {
      if (charInput[i] != 0 || charInput[i] != '\0') {
        eeprom_object.projectID[i] = charInput[i];
      } else {
        eeprom_object.projectID[i] = '\0';
        break;
      }
    }
  }
}

// Planned Change: This will be removed since BoardID will be removed
// void set_board_id(){
//   Serial.print(F(" Board ID:     "));  // get BoardID
//   Serial.flush();
//   get_integer_input();  // decode user input
//   eeprom_object.IDnum = indata;
// }

void set_node_id(){
  Serial.print(F(" Node ID:  "));  // get nodeID
  Serial.flush();
  get_integer_input();  // decode user input
  eeprom_object.nodeID = indata;
}

void set_gateway_id(){
  Serial.print((" Gateway ID:  "));  // get GatewayID
  Serial.flush();
  get_integer_input();  // decode user input
  eeprom_object.gatewayID = indata;
}

//-----Read DS18B20 Temperature sensors-------------------------------------------            //do not plug in backwards or board will heat!!!
float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius, not using the actual return value but updating the temperature_string

  sensors.begin();  //initiate the library

  delay(100);
  sensors.setResolution(DS18B20_Address, 9);  //Specify the resolution of the device, in bits (9 to 12). Higher resolution takes more time, 12bit can be up to 750ms

  delay(100);  //The second call to sensors.begin(); below did not work for me 04/04/22

  Serial.println(F("Requesting Temperatures..."));  //Print temperatures by address, each sensor address is defined before setup().
  sensors.requestTemperatures();

  delay(1000);

  Serial.print(F("Locating devices..."));  // locate devices on the bus with getDeviceCount, there are known errors with getDeviceCount returning 0....
  Serial.print(F("Found "));
  int num_ds18b20 = (sensors.getDeviceCount());  //store # of sensors detected, there are known errors with getDeviceCount returning 0....
  delay(100);
  Serial.print(num_ds18b20);
  Serial.println(F(" devices."));

  Serial.print(F("Power = Parasite:  "));
  Serial.println(sensors.readPowerSupply(DS18B20_Address));  //report parasite power condition (should be 0 with our wiring.)
  delay(100);
  Serial.print(("Parasite power is: "));  // report parasite power condition
  if (sensors.isParasitePowerMode()) {
    Serial.println(F("ON"));
  } else {
    Serial.println(F("OFF"));
  }

  delay(10);

  //Reference code to store the addresses in eeprom. This seems by-far the best solution through a menu() window to identify sensors....
  //< https: //forum.arduino.cc/t/storage-of-ds18b20-address-in-eeprom/934477/19>
  //< https: //www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html>
  // Serial.println(F("Begin test of configuring string from array"));
  // Serial.println(eeprom_object.num_ds18b20);
  float sum_temp = 0;  //for mean
  float mean_temp = 0;
  float single_temp = 0;  //for error detection
  float rtcTemp = 0;

  // Planned Change
  // Replace this array with the 2D array in eeprom
  //The top level array (an array of arrays here) must be a pointer.
  //uint8_t *ds18b20_array[]{ eeprom_object.ds18b20_sensor0, eeprom_object.ds18b20_sensor1, eeprom_object.ds18b20_sensor2, eeprom_object.ds18b20_sensor3, eeprom_object.ds18b20_sensor4, eeprom_object.ds18b20_sensor5, eeprom_object.ds18b20_sensor6, eeprom_object.ds18b20_sensor7, eeprom_object.ds18b20_sensor8, eeprom_object.ds18b20_sensor9, eeprom_object.ds18b20_sensor10, eeprom_object.ds18b20_sensor11, eeprom_object.ds18b20_sensor12, eeprom_object.ds18b20_sensor13, eeprom_object.ds18b20_sensor14, eeprom_object.ds18b20_sensor15 };

  rtcTemp = rtc.getTemperature();  //from the ds3232 library -> replaced with RTClib.h solution which already does the bitshifting

  //Loop through...
  for (int i = 0; i < (eeprom_object.num_ds18b20); i++) {
    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.print(F(" Address: "));
    //printAddress(ds18b20_array[i]);
    printAddress(eeprom_object.ds18b20_sensor_addresses[i]);
    Serial.print(F(" Temperature C: "));
    single_temp = sensors.getTempC(eeprom_object.ds18b20_sensor_addresses[i]);
    delay(10);
    Serial.print(single_temp);
    Serial.println();

    if (single_temp == -127.00) {
      eeprom_object.error_log.ds18b20_struct.ds_unit_error[i] = true;
      eeprom_object.error_log.ds18b20_struct.ds_unit_error_code[i] = 111;  // "o" for open
      eeprom_object.error_log.write_log = true;
    }
    if (single_temp == 85.00) {
      eeprom_object.error_log.ds18b20_struct.ds_unit_error[i] = true;
      eeprom_object.error_log.ds18b20_struct.ds_unit_error_code[i] = 114;  // "r" for reset during conversion
      eeprom_object.error_log.write_log = true;
    }
    if (single_temp == -98.00) {
      eeprom_object.error_log.ds18b20_struct.ds_unit_error[i] = true;
      eeprom_object.error_log.ds18b20_struct.ds_unit_error_code[i] = 117;  // "u" for unknown error
      eeprom_object.error_log.write_log = true;
    }

    if (single_temp == -127.00 || single_temp == -98.00 || single_temp == 85.00) {  //Check for known fault codes
      if (eeprom_object.run_notes == true) {
        Serial.print(F("Error on temperature sensor:  "));
        Serial.print(i);
        Serial.print(F("  With Value: "));
        Serial.println(single_temp);
        Serial.println(F("Replacing this sensors temperature with that of the RTC."));
      }
      single_temp = rtcTemp;
    }

    //Removed on 06/15/2023 as some nodes may be in the sun while sensors are in the soil leading to a large difference in temperature. Let the error codes handle replacement.
    // if (((abs(rtcTemp - single_temp) / ((rtcTemp + single_temp) / 2)) > 20) || ((single_temp - rtcTemp) >= 10) || ((single_temp - rtcTemp) <= -10)) {  //if Percent difference between the sensor temp and the RTC temp is higher than 20%, use the rtcTemp from the RTC. 2022/05/23 Added + or - 10 from rtcTemp as an additional criteria.
    //   Serial.print(F("Percent Difference threshold (20%) from RTC reference temp exceeded on Sensor "));
    //   Serial.print(i);
    //   Serial.print(F(" With Value: "));
    //   Serial.println(single_temp);
    //   Serial.println(F("Replacing this sensors temperature with that of the RTC."));
    //   single_temp = rtcTemp;
    // }

    sum_temp += single_temp;
    delay(20);
    mean_temp = (sum_temp / eeprom_object.num_ds18b20);

    //IF 0 external DS18b20 temperature sensors are connected, temperature corrections will be made using the rtcTemp & this will be reported.
    if (eeprom_object.num_ds18b20 == 0) {
      single_temp = rtcTemp;
      mean_temp = rtcTemp;
    }


    temperature_string += 'T';
    temperature_string += (i);
    temperature_string += ',';
    temperature_string += single_temp;
    global_temp_values[i] = single_temp;
    delay(10);
    temperature_string += ',';
  }

  if (eeprom_object.run_notes == true) {
    Serial.print(F("Sum of temperature sensors: "));
    Serial.println(sum_temp);
    Serial.print(F("Number of temperature sensors:  "));
    Serial.println(eeprom_object.num_ds18b20);
    Serial.println(F("End test of configuring string from array"));
  }

  Serial.print(F("Mean temperature:  "));
  Serial.println(mean_temp);
  return mean_temp;
}

////
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.print(F(" "));
}

void compile_json(){
  // Option 1: Use global JSON buffer
  // jsonBuffer.clear();
  // JsonObject obj = jsonBuffer.createNestedObject("data");

  // Option 2: Use local JSON document and object
  StaticJsonDocument<10000> data;
  JsonObject obj = data.createNestedObject();

  // Related functions: read_watermark(), getTemp(), WM_Irrigation_prompt()
  // Old terminology of "header", "WM_String" would go away and we work solely off of the key, value pairs
  // Try to separate reading and computing statistics
  
  // header contains node ID, voltage of battery, and date and time

  // Planned Change: This will be removed since IDnum will be removed
  // obj["nodeID"] = eeprom_object.IDnum;

  battV = calcbattV();
  Serial.print(F("Battery Pack Voltage:  "));
  Serial.println(battV);
  obj["voltage"] = battV;

  obj["months"] = mnths;
  obj["days"] = days;
  obj["years"] = yrs;
  obj["hours"] = hrs;
  obj["minutes"] = mins;
  obj["seconds"] = secs;
  
  delay(20);

  // irrigation prompt string: irrigation group number, mean (kpa) (sensor reading), start of last irrigation event, stop of last irigation event,
  obj["WM_group_num"] = global_WM_group_num;
  obj["WM_group_mean"] = global_WM_group_mean;
  obj["last_irr_starting_time"] = global_last_irr_starting_time;
  obj["last_irr_ending_time"] = global_last_irr_ending_time;

  // temperature string: temperature group  and reading (temperature sensor number and value)
  // In read_watermark(), 
  // example of data: 110,7.46,8/27/2023 11:50:29,0,-67,1,-110,2,0,3,-184,4,0,5,-137,6,0,7,0,8,-81,9,-73,10,0,11,-100,12,0,G1,-100,8/27/2023 11:50:48,G2,-999,8/27/2023 10:48:18,G3,-999,8/27/2023 10:48:50,G4,0,5/3/2101 1:45:12,T0,29.50
  
  JsonArray temperature_readings = obj.createNestedArray("temperature_readings");
  // FIXME: finish temperature sensor stuff
  for (int i = 0; i < 16; i++){
    temperature_readings.add(global_temp_values[i]);
  }

  // example keys: IDnum, months, days, years
}

//---
// Planned Change: Will this be removed as we add the new JSON methods to save data
// void saveData() {

//   if (!SD.begin(SD_CS)) {
//     Serial.println(F("SD card not present or card failure."));
//     eeprom_object.error_log.sd_struct.card_begin_failure = true;
//     eeprom_object.error_log.write_log = true;
//   }

//   delay(50);

//   myfile = SD.open(filename, FILE_WRITE);  //Open file for writing
//   delay(100);

//   if (myfile) {                             //If myfile is available...
//     if (eeprom_object.firstTime == true) {  //Print this header if first time writing to file
//       myfile.println();
//       myfile.println(F("Data values are stored as follows:"));
//       myfile.println(F("Node ID, Node battery voltage (V), DateTime"));
//       myfile.println(F("Following DateTime the mux channel position and sensor value (CB/kpa) for all watermark sensors installed are printed (Channel, Value, Channel, Value)"));
//       myfile.println(F("IF user specified for reporting of raw resistance values, format will be (Channel, kPa, resistance, Channel, kPa, Resistance), etc."));
//       myfile.println(F("Following the raw watermark sensor data, the water management group means are reported as (G(1-4), Mean, DateTime of last irrigation)"));
//       myfile.println(F("After watermark group means, DS18b20 temperature sensor data is reported as (T(# of sensor), Value (C))"));
//       myfile.println();

//       eeprom_object.firstTime = false;  //Only include this header when first writing to file
//     }

//     data.remove(data.length() - 1, 1);  //Remove the character that indicated the end of a transmission "]"
//     delay(10);
//     data.remove(0, 4);  //Remove the Transmission information "[~~~". Zero indexed
//     delay(10);
//     myfile.println(data);  //Save data string to file
//     delay(200);
//     myfile.close();  //Must close file
//     delay(200);
//     Serial.println(F("Data Written Successfully."));
//   } else {
//     Serial.println(F("Error opening file."));  //Error if myfile is not available
//     eeprom_object.error_log.sd_struct.open_file_failure = true;
//     eeprom_object.error_log.write_log = true;
//   }
// }

// FIXME: Instead of populating WM_String string, populate JSON object
//-----Read Watermark Sensors---------------------------------------
void read_watermark() {
  //Declare vars
  const int num_Iter = 3;              //Define number of iterations. Each is actually two reads of the sensor (both directions)
  long TempC = Temp;                   //Pull average temperature sensor from ds18b20 for time being until each temperature sensor can be attributed to a single or group of watermark sensors
  const long open_resistance = 35000;  //For throwing fault 255, default was 35000
  const long short_resistance = 200;   //For throwing fault 240, default was 200, modified was 45
  const long mux_resistance = 140;     //The "ON" resistance of two CD74HC4067 Multiplexors (70 ohms each)
  const long short_CB = 240;           //The Fault Code for WM sensor terminal short in wiring
  const long open_CB = 255;            //The Fault Code for WM sensor open circuit or missing sensor / sensor not present
  double Calib_Resistance;             //Holder for any calibration resistor present
  double SenV10K = 0;                  //For calibration resistor
  int number_WM_Cal_resistor_offset;   //Will be equal to eeprom_object.num_WM unless a calibration resistor is used, it will be incremented by 1

  if (eeprom_object.calibration_resistor_present) {
    number_WM_Cal_resistor_offset = eeprom_object.num_WM;
    number_WM_Cal_resistor_offset++;
  } else {
    number_WM_Cal_resistor_offset = eeprom_object.num_WM;
  }


  int WM_case;     //for describing what happens to the watermark readings based on the resistance
  int WM1_CB = 0;  //Holder for WM sensor value in CB/kPa, direction 1
  int WM2_CB = 0;  //Holder for WM sensor value in CB/kPa, direction 2

  double SenVWM1 = 0;  //VWM is the "stand off voltage"?, taken here to mean the average voltage which was recorded in direction 1.
  double SenVWM2 = 0;  //VWM is the "stand off voltage"?, taken here to mean the average voltage which was recorded in direction 2.

  double ARead_path1 = 0;  //The raw analog read value obtained in direction 1.
  double ARead_path2 = 0;  //The raw analog read value obtained in direction 2.

  float SupplyV = 3.3;  //If using a different logic board (3.3V vs. 5V etc) moteino mega is ~3.3v. //In the future it may be desirable to store energy in a capacitor instead of relying on instant moteino board power.

  //Code-----
  ARead_path1 = 0;  //Reset these values on the loop
  ARead_path2 = 0;

  if (eeprom_object.calibration_resistor_present == true) {  //Read the calibration resistor IF included. There is no polarity swap needed here
    for (int j = 0; j < num_Iter; j++) {                     //the num_Iter controls the number of successive read loops that is averaged.
      digitalWrite(mux_enable, LOW);                         //enable the MUX (The EN Pin on CD74HC4067)
      delay(10);
      mux_1.channel(eeprom_object.cal_resistor_loc);  //address the MUX where calibration resistor is installed
      delay(10);
      digitalWrite(WM_path2, LOW);                    //Set pin as gnd
      digitalWrite(WM_path1, HIGH);                   //Set pin as Vs
      delay(0.09);                                    //wait 90 micro seconds and take sensor read
      ARead_path1 += analogRead(WM_analog_read_pin);  //read the analog pin the multiplexor is connected to (Moteino Mega 10bit ADC)
      digitalWrite(WM_path1, LOW);                    //set the excitation voltage to OFF/LOW
      delay(100);                                     //0.1 second delay before moving to next channel or switching MUX
    }
    digitalWrite(mux_enable, HIGH);                                                                                                             //Disable mulitplexor
    SenV10K = ((ARead_path1 / 1024.0f) * SupplyV) / (num_Iter);                                                                                 //get the average of the number of readings and convert to volts, 1024 here for 10bit ADC
    Calib_Resistance = eeprom_object.cal_resistor_val / ((eeprom_object.fixed_resistor_val * (SupplyV - SenV10K) / SenV10K) - mux_resistance);  //generate a calibration factor.
    Serial.print(F("SenV10K:  "));
    Serial.println(SenV10K);
    Serial.print(F("eeprom_object.cal_resistor_loc: "));
    Serial.println(eeprom_object.cal_resistor_loc);
    Serial.print(F("Calibration Factor from fixed Resistor: "));
    Serial.println(Calib_Resistance);
    Serial.print(F("Raw Sum of Analog Reads:  "));
    Serial.println(ARead_path1);
    delay(100);  //0.1 second wait before moving to next channel or switching MUX
  } else {
    Serial.println(F("Calibration Factor defaulting to 1. Insert and specify a fixed resistor for calculating a calibration factor."));
    Calib_Resistance = 1;            //if not actually calculated, assume 1
    digitalWrite(mux_enable, HIGH);  //Disable mux pair
  }
  for (int i = 0; i < number_WM_Cal_resistor_offset; i++) {  //Assuming num_WM starts at mux channel 0 and increments, i.e. if num_WM = 4, i=0,1,2,3
    ARead_path1 = 0;                                //restore values to 0 for successive WM readings
    ARead_path2 = 0;

    //8/2/2023 Need to account for when a Calibration Resistor is installed. The for statement above used to read i < eeprom_object.num_WM


    for (int n = 0; n < num_Iter; n++) {  //take num_Iter readings from mux channel i
      digitalWrite(mux_enable, LOW);      // enable the MUX
      delay(100);
      mux_1.channel(i);  //Select mux channel from i in for loop referenceing num_WM above
      delay(10);
      digitalWrite(WM_path1, HIGH);                   //set signal pin of mux 1 (pin7) high
      delay(0.09);                                    //Wait 90 microseconds
      ARead_path1 += analogRead(WM_analog_read_pin);  //read the signal from this direction and add to running total
      digitalWrite(WM_path1, LOW);                    //set pin low

      //Polarity SWAP
      delay(100);

      digitalWrite(WM_path2, HIGH);                   //set pin 11 high this time
      delay(0.09);                                    //Wait 90 microseconds
      ARead_path2 += analogRead(WM_analog_read_pin);  //read the signal from the opposite direction and add to running total
      digitalWrite(WM_path2, LOW);                    //set pin low
    }
    SenVWM1 = ((ARead_path1 / 1024.0f) * SupplyV) / (num_Iter);  //get the average of the readings in the first direction and convert to volts | 1024 because 10bit adc
    SenVWM2 = ((ARead_path2 / 1024.0f) * SupplyV) / (num_Iter);  //get the average of the readings in the second direction and convert to volts

    double WM1_ResistanceA = ((eeprom_object.fixed_resistor_val * (SupplyV - SenVWM1) / SenVWM1) - mux_resistance);  //do the voltage divider math, using the the known resistor in the circuit
    double WM1_ResistanceB = eeprom_object.fixed_resistor_val * SenVWM2 / (SupplyV - SenVWM2) - mux_resistance;      //voltage divider math, opposite direction

    double WM1_Resistance = ((WM1_ResistanceA + WM1_ResistanceB) / 2) * Calib_Resistance;  //Average of the directions and apply calibration factor
    digitalWrite(mux_enable, HIGH);                                                        //Disable mux pair
    delay(100);

    //Troubleshooting consideration
    if (eeprom_object.run_notes) {
      Serial.print(F("ARead_path1:  "));
      Serial.println(ARead_path1);
      Serial.print(F("ARead_path2:  "));
      Serial.println(ARead_path2);
      Serial.print(F("SenVWM1:  "));
      Serial.println(SenVWM1);
      Serial.print(F("SenVWM2:  "));
      Serial.println(SenVWM2);
      Serial.print(F("WM1_ResistanceA:  "));
      Serial.println(WM1_ResistanceA);
      Serial.print(F("WM1_ResistanceB:  "));
      Serial.println(WM1_ResistanceB);
      Serial.print(F("WM_Resistance:  "));
      Serial.println(WM1_Resistance);
    }


    //-------Conversion of Resistance to centibars/kilopascals, 6 cases plus faults, equations from manufacturer
    if (WM1_Resistance > 550.00) {
      if (WM1_Resistance > 8000.00) {
        WM_case = 1;
      } else if (WM1_Resistance > 1000.00) {
        WM_case = 2;
      } else {
        WM_case = 3;
      }
    } else {
      if (WM1_Resistance > 300.00) {
        WM_case = 4;
      }
      if (WM1_Resistance < 300.00 && WM1_Resistance >= short_resistance) {
        WM_case = 5;
      }
    }
    if (WM1_Resistance >= open_resistance) {
      WM_case = 6;
    }
    switch (WM_case) {
      case 1:
        WM1_CB = -2.246f - 5.239f * (WM1_Resistance / 1000.00f) * (1.0f + .018f * (TempC - 24.00f)) - .06756f * (WM1_Resistance / 1000.00f) * (WM1_Resistance / 1000.00f) * ((1.00f + 0.018f * (TempC - 24.00f)) * (1.00f + 0.018f * (TempC - 24.00f)));
        break;
      case 2:
        WM1_CB = (-3.213f * (WM1_Resistance / 1000.00f) - 4.093f) / (1.0f - 0.009733f * (WM1_Resistance / 1000.00f) - 0.01205f * (TempC));
        break;
      case 3:
        WM1_CB = ((WM1_Resistance / 1000.00f) * 23.156f - 12.736f) * (1.0f + 0.018f * (TempC - 24.00f));
        break;
      case 4:
        WM1_CB = 0.00;
        break;
      case 5:
        WM1_CB = short_CB;  //240 is a fault code for sensor terminal short
        eeprom_object.error_log.wm_struct.mux_channel_error[i] = true;
        eeprom_object.error_log.wm_struct.mux_channel_error_code[i] = 115;  // "s" for short
        eeprom_object.error_log.write_log = true;
        break;
      case 6:
        WM1_CB = open_CB;  //255 is a fault code for open circuit or sensor not present, also observed in new dry sensors
        eeprom_object.error_log.wm_struct.mux_channel_error[i] = true;
        eeprom_object.error_log.wm_struct.mux_channel_error_code[i] = 111;  //"o" for open
        eeprom_object.error_log.write_log = true;
        break;
      default:
        WM1_CB = 0;  //the default if resistance does not fall within case 1 to 6
        eeprom_object.error_log.wm_struct.mux_channel_error[i] = true;
        eeprom_object.error_log.wm_struct.mux_channel_error_code[i] = 100;  // "d" for default
        eeprom_object.error_log.write_log = true;
        break;
    }

    //July 2023, we are observing some sensors returning positive kPa values (should be impossible by falling into one of the above cases, but tuning seems to be required for the constants)
    if (WM1_CB > 0) {
      WM1_CB = 0;
      eeprom_object.error_log.wm_struct.mux_channel_error[i] = true;
      eeprom_object.error_log.wm_struct.mux_channel_error_code[i] = 112;  // "p" for positive
      eeprom_object.error_log.write_log = true;
    }

    //print out data of interest
    Serial.print(F("Channel "));
    Serial.print(i);
    Serial.print(F("  Resistance Ohms: "));
    Serial.print(WM1_Resistance);
    Serial.print(F("  kPa:  "));
    Serial.println(WM1_CB);

    //Aggregate data to WM_string
    //IF statement for if raw resistance values are desired too...
    if (eeprom_object.include_resistance == true) {
      WM_string += (i);
      WM_string += ',';
      WM_string += (WM1_Resistance);
      WM_string += ',';
      WM_string += (WM1_CB);
      WM_string += ',';
    } else {
      WM_string += (i);
      WM_string += ',';
      WM_string += (WM1_CB);
      WM_string += ',';
    }
  }
}

// Planned Change: This will be removed as the new JSON methods of saving data are added
// //---------- Clear buffer---------------
// void clearBuffer() {
//   WM_string = "";
//   temperature_string = "";
//   irrigation_prompt_string = "";
//   data = "";
//   delay(100);
// }

//-----WM Group means handling------------------                                 //This portion could probably be substantially optimized...

int WM_group_means(uint8_t WM_group_num[], int datasize, uint8_t water_threshold_group) {

  // Work In Progress: Commented out to allow for compilation
  // force_irr = false;  //reset force_irr
  // int set_water_threshold;

  // if (water_threshold_group == 1) {
  //   set_water_threshold = eeprom_object.group_irr_thresholds[0];
  // } else if (water_threshold_group == 2) {
  //   set_water_threshold = eeprom_object.group_irr_thresholds[1];
  // } else if (water_threshold_group == 3) {
  //   set_water_threshold = eeprom_object.group_irr_thresholds[2];
  // } else if (water_threshold_group == 4) {
  //   set_water_threshold = eeprom_object.group_irr_thresholds[3];
  // } else {
  //   Serial.println(F("Water threshold undefined, define it in the menu."));
  // }


  // char WM_array[WM_string.length() + 1];              //arrays are zero-indexed (start at position 0)
  // WM_string.toCharArray(WM_array, sizeof(WM_array));  //convert the WM_string to a character array

  // char *pointer_array[sizeof(WM_array)];  //create an empty character (pointer_array) the size of the character array
  // char *pointer = NULL;                   //declare an empty pointer variable
  // byte index = 0;                         //Declare an index to increment starting at 0 since arrays are 0 indexed

  // pointer = strtok(WM_array, ",");  //Looking withing WM_array, a comma is a delimiter for separation, declare what pointer points to
  // Serial.print(F("pointer:  "));
  // Serial.println(pointer);
  // while (pointer != NULL) {
  //   pointer_array[index] = pointer;  //within pointer_array declare the pointer (0-inf) and the value pointed to (pointer)
  //   index++;
  //   pointer = strtok(NULL, ",");  //end the while loop
  // }

  // int sum = 0;  //declare holders for sum, count
  // int count = 0;
  // int error_code_count = 0;
  // //uint8_t raw_group_mean = 0;
  // float raw_group_mean = 0.00;
  // int buff_sum = 0;
  // int buff_count = 0;
  // int count_outliers = 0;  //Start at 0
  // int expected_sensor_count = 0;

  // if (water_threshold_group == 1) {
  //   Serial.print(F("eeprom_object.WM_group1:  "));
  //   for (int r = 0; r < 16; r++) {
  //     if (eeprom_object.WM_group1[r] != 255 && eeprom_object.WM_group1[r] != -1) {
  //       Serial.print(eeprom_object.WM_group1[r]);
  //     }
  //     if (eeprom_object.WM_group1[r] >= 0) {
  //       expected_sensor_count++;
  //     }
  //   }
  //   Serial.println();
  // } else if (water_threshold_group == 2) {
  //   Serial.print(F("eeprom_object.WM_group2:  "));
  //   for (int r = 0; r < 16; r++) {
  //     if (eeprom_object.WM_group2[r] != 255 && eeprom_object.WM_group2[r] != -1) {
  //       Serial.print(eeprom_object.WM_group2[r]);
  //     }
  //     if (eeprom_object.WM_group2[r] >= 0) {
  //       expected_sensor_count++;
  //     }
  //   }
  //   Serial.println();
  // } else if (water_threshold_group == 3) {
  //   Serial.print(F("eeprom_object.WM_group3:  "));
  //   for (int r = 0; r < 16; r++) {
  //     if (eeprom_object.WM_group3[r] != 255 && eeprom_object.WM_group3[r] != -1) {
  //       Serial.print(eeprom_object.WM_group3[r]);
  //     }
  //     if (eeprom_object.WM_group3[r] >= 0) {
  //       expected_sensor_count++;
  //     }
  //   }
  //   Serial.println();

  // } else if (water_threshold_group == 4) {
  //   Serial.print(F("eeprom_object.WM_group4:  "));
  //   for (int r = 0; r < 16; r++) {
  //     if (eeprom_object.WM_group4[r] != 255 && eeprom_object.WM_group4[r] != -1) {
  //       Serial.print(eeprom_object.WM_group4[r]);
  //     }
  //     if (eeprom_object.WM_group4[r] >= 0) {
  //       expected_sensor_count++;
  //     }
  //   }
  //   Serial.println();
  // } else {
  //   Serial.println(F("Water threshold undefined, define it in the menu."));
  // }



  // for (int i = 0; i < datasize; i++) {                               //for the declared channel group,
  //   int q = WM_group_num[i];                                         //q mirrors each element of channel group up to the globally specified datasize
  //   if (q != ',' && q != ' ' && q != '-' && q != "-1" && q <= 16) {  //if q is not a comma or a space, -1, or some random number above 16...
  //     //  q = (q * 2) + 1; //q is the location of the data of the desired channel, because channel of mux and arrays begin at 0, the value of channel 0 will be on element 1 of the array. formula is x*2+1                                                                                 //This is of course different if resistance values are included in the string.
  //     if (eeprom_object.include_resistance == true) {
  //       q = (q * 3) + 2;  //if we are including resistances in the wm_string, the formula is x*3+2
  //     } else {
  //       q = (q * 2) + 1;
  //     }
  //     int wm_val;
  //     wm_val = atoi(pointer_array[q]);
  //     //for troubleshooting
  //     Serial.print(F("wm_val: "));  //for troubleshooting
  //     Serial.println(wm_val);

  //     if (wm_val != 255 && wm_val != 240 && wm_val != 0 && wm_val < 0) {  //If not throwing a fault code calculate an initial mean
  //       sum += abs(wm_val);
  //       count++;
  //     } else {
  //       if (wm_val == 255 || wm_val == 240 || wm_val > 0) {  // 255=open circuit, 240=short circuit
  //         error_code_count++;
  //       }
  //     }
  //     if (WM_group_num[i] != -1 && WM_group_num[i] <= 16) {
  //       if (wm_val == 255 || wm_val == 240 || wm_val == 0) {
  //         Serial.print(F("Node: "));           //If a fault code was one of the data requested in the group
  //         Serial.print(eeprom_object.nodeID);  //Serial.print the nodeID, channel (algorithim is (x-1) / 2), and value of the fault
  //         Serial.print(F("  Fault code on channel:  "));
  //         Serial.print(WM_group_num[i]);
  //         Serial.print(F("  with value: "));
  //         Serial.print(wm_val);
  //         Serial.println(F(".  Value not included in Group mean."));
  //       }
  //     }
  //   }
  // }

  // //For troubleshooting
  // if (eeprom_object.run_notes) {
  //   Serial.print(F("Sum:  "));
  //   Serial.println(sum);
  //   Serial.print(F("Count:  "));
  //   Serial.println(count);
  //   Serial.print(F("Expected Sensor Count:  "));
  //   Serial.println(expected_sensor_count);
  //   Serial.print(F("Error Code Count: "));
  //   Serial.println(error_code_count);
  // }

  // if (count == 0.00) {  // none of the wm_val are counted...
  //   raw_group_mean = 0.00;
  // } else {
  //   raw_group_mean = (sum / count) * -1;  //calculate the raw_mean of the of the desired channels, negative 1 here as kpa is in tension
  // }
  // Serial.print(F("Raw group mean: "));
  // Serial.println(raw_group_mean);


  // //Run through calculation again, if percent difference of each sensor is < 20% from the raw_group_mean (all sensors as long as not fault code) include in the calculation of the buffered_mean that gets output & triggers irr events.
  // // 04/25/2022 Note that pdiff calc is working but that small differences in kpa result in large percent differences due to log scale...
  // // Need to consider a secondary flag. for example a "grace window" -> if the pdiff is > 20, check grace window. -> grace window = 10 (could be changed) -> if this sensors kpa is +10 or -10 from the raw_group_mean OR the set water threshold, include in calc of buffered mean, else exclude from buffered mean.

  // for (int i = 0; i < datasize; i++) {                               //for the declared channel group,
  //   int q = WM_group_num[i];                                         //q mirrors each element of channel group up to the globally specified datasize
  //   if (q != ',' && q != ' ' && q != '-' && q != "-1" && q <= 16) {  //if q is not a comma or a space, -1, or some random number above 16...
  //     if (eeprom_object.include_resistance == true) {                //q is the location of the data of the desired channel, because channel of mux and arrays begin at 0, the value of channel 0 will be on element 1 of the array. formula is x*2+1
  //       q = (q * 3) + 2;                                             //if we are including resistances in the wm_string, the formula is x*3+2
  //     } else {
  //       q = (q * 2) + 1;
  //     }
  //     int wm_val;
  //     wm_val = atoi(pointer_array[q]);

  //     ////// Another Question needs addressed. What if the mean is such that each individual sensor is >= 20% different from the raw_mean? ->> It needs to trigger the irrigation event anyway.
  //     if (wm_val != 255 && wm_val != 256 && wm_val != 240 && wm_val != 0 && WM_group_num[i] != -1) {
  //       if (abs((abs(raw_group_mean - wm_val) / ((raw_group_mean + wm_val) / 2.0)) * 100) < 20.0) {  //if the sensor reading is less than 20% different from the raw_group_mean, include in calc of buffered_group_mean
  //         Serial.print(F("Channel: "));
  //         Serial.print(WM_group_num[i]);
  //         Serial.print(F("  value:  "));
  //         Serial.print(wm_val);
  //         Serial.print(F(", Pdiff from raw mean = "));
  //         Serial.println(abs(((abs(raw_group_mean - wm_val)) / ((raw_group_mean + wm_val) / 2.0)) * 100));
  //         buff_sum += abs(wm_val);
  //         buff_count++;

  //       } else if ((wm_val < set_water_threshold - wm_grace_window || wm_val > set_water_threshold + wm_grace_window) && (wm_val < raw_group_mean - wm_grace_window || wm_val > raw_group_mean + wm_grace_window)) {  //check grace window against raw group mean and the user defined water threshold, wm_grace_window defined globally.
  //         Serial.print(F("Channel: "));
  //         Serial.print(WM_group_num[i]);
  //         Serial.print(F("  value:  "));
  //         Serial.print(wm_val);
  //         Serial.print(F(", Pdiff from raw mean = "));
  //         Serial.print(abs(((abs(raw_group_mean - wm_val)) / ((raw_group_mean + wm_val) / 2.0)) * 100));
  //         Serial.print(F("  detected as an outlier, (>20 pdiff & more than +- 10kpa from raw_group_mean AND the threshold set point)"));
  //         Serial.println(F("  This channel is not counted in buffered group mean."));

  //         Serial.print(F("Acceptable range based on raw group mean: "));
  //         Serial.print(raw_group_mean - wm_grace_window);
  //         Serial.print(F(" to "));
  //         Serial.print(raw_group_mean + wm_grace_window);
  //         Serial.println(F(" kPa"));

  //         Serial.print(F("Acceptable range based on threshold set point: "));
  //         Serial.print(set_water_threshold - wm_grace_window);
  //         Serial.print(F(" to "));
  //         Serial.print(set_water_threshold + wm_grace_window);
  //         Serial.println(F(" kPa"));

  //         count_outliers++;  //increment outlier count.
  //       } else {
  //         Serial.print(F("Channel: "));
  //         Serial.print(WM_group_num[i]);
  //         Serial.print(F("  value:  "));
  //         Serial.print(wm_val);
  //         Serial.print(F("  detected as a POSSIBLE outlier, (>20 pdiff from raw_group_mean,"));
  //         Serial.print(F(" Pdiff from raw mean = "));
  //         Serial.print(abs(((abs(raw_group_mean - wm_val)) / ((raw_group_mean + wm_val) / 2.0)) * 100));
  //         Serial.println(F(")"));
  //         buff_sum += abs(wm_val);  //include in buffered_mean calc anyway if + - 10 from raw mean or set water threshold
  //         buff_count++;
  //       }
  //     }
  //   }
  // }
  // Serial.print(F("Total Outliers: "));
  // Serial.println(count_outliers);

  // Serial.print(F("Sensors Considered (count): "));
  // Serial.println(buff_count);

  // if (error_code_count > (0.25 * expected_sensor_count) && expected_sensor_count > 2) {  // if there are 3 or more sensors expected and the error code count is greater than or equal to 1/4 of the expected sensor count.....force the event
  //   Serial.println(F("More than 1/4 the expected sensors of the group are throwing an error code."));
  //   Serial.println(F("Forcing an irrigation event."));
  //   force_irr = true;
  //   eeprom_object.error_log.irr_struct.forced_group[water_threshold_group - 1] = true;      //-1 as 0 indexed
  //   eeprom_object.error_log.irr_struct.forced_group_code[water_threshold_group - 1] = 101;  // "e" for error
  // }

  // if ((count_outliers >= buff_count) && count > 3) {                                        //force irrigation event if half or more of the buff_count are outliers if there are 3 or more sensors connected.
  //   eeprom_object.error_log.irr_struct.forced_group[water_threshold_group - 1] = true;      //-1 as 0 indexed
  //   eeprom_object.error_log.irr_struct.forced_group_code[water_threshold_group - 1] = 111;  // "o" for outlier

  //   Serial.println(F("Number of outliers >= one-half of non-error sensors AND the number of non-error sensors is > 3. Check and see whether to force irrigation event or encountering error."));
  //   Serial.print(F("Raw group mean is equal to: "));
  //   Serial.println(raw_group_mean);
  //   if ((raw_group_mean <= 0 && raw_group_mean >= -239) && (raw_group_mean < set_water_threshold)) {
  //     Serial.print(F("The raw group mean is reasonable and is lower than the set water threshold for the group."));
  //     Serial.println(F("  Proceed with forced irrigation event, note this result is possible with poor connection of ALL WM sensors in the group."));
  //     force_irr = true;
  //   } else {
  //     Serial.println(F("The raw group mean is not within the measurement range of WM sensors OR is higher than the set water threshold for the group."));
  //     Serial.println(F("Force irrigation event prevented."));
  //     eeprom_object.error_log.irr_struct.forced_group_code[water_threshold_group - 1] = 104;  // "h" for higher than set threshold
  //   }
  // }

  // Serial.print(F("buff_sum:  "));
  // Serial.println(buff_sum);
  // Serial.print(F("buff_count:  "));
  // Serial.println(buff_count);
  // //uint8_t buffered_group_mean = 0;
  // float buffered_group_mean;
  // if (buff_count == 0) {
  //   buffered_group_mean = 0.00;
  // } else {
  //   buffered_group_mean = (buff_sum / buff_count) * -1;  //calculate the mean of the of the desired channels, negative 1 here as kpa is in tension
  // }
  // Serial.print(F("Buffered Group mean: "));
  // Serial.println(buffered_group_mean);

  // if (force_irr) {
  //   Serial.println("Force irrigation event detected.");  //Give indication you are forcing the irrigation event
  //   buffered_group_mean = -999;
  // }

  // return buffered_group_mean;  //return the requested group_mean
  return 1;
}

//-----Compile Data-------------------------------------------------
void compile() {

  battV = calcbattV();
  Serial.print(F("Battery Pack Voltage:  "));
  Serial.println(battV);

  data = "";
  header = "";
  delay(20);

  header += '[';  //Add in String start indicator '[' and spacers "~~~" for packet # as in RadioString library
  header += '~';
  header += '~';
  header += '~';

  // Planned Change: IDnum has been removed from the EEPROM
  // header += eeprom_object.IDnum;
  header += ',';

  if (battV <= lowBatt) {
    header += "Low Battery Voltage: ";
    eeprom_object.error_log.pwr_struct.bat_low = true;
    eeprom_object.error_log.write_log = true;
    battLow = true;  // will be depreciated
  }
  header += battV;

  header += ',';
  header += mnths;
  header += '/';
  header += days;
  header += '/';
  header += yrs;
  header += ' ';
  header += hrs;
  header += ':';
  if (mins < 10) header += "0";
  header += mins;
  header += ':';
  if (secs < 10) header += "0";
  header += secs;
  header += ',';
  delay(20);

  data = header + WM_string + irrigation_prompt_string + temperature_string;  //Add in responses from sensors

  delay(50);

  data.remove(data.length() - 1, 1);  //remove last comma
  delay(10);
  data.concat(']');  //Add in an indicator for the end of the data string

  // End of data string
  delay(10);
  Serial.println(data);
  delay(50);
}



bool check_whether_address_seen(byte addr[8], byte addresses[16][8], int num_sensors_passed){
  bool address_seen = false;
  for (int i = 0; i < num_sensors_passed; i++){
    if (memcmp(addr, addresses[i], 8) == 0){
      address_seen = true;
    }
  }

  return address_seen;
}


// FIXME: Refactor
//-----Identification of ds18b20 addresses-----------
void Identify_1WireDevices() {
  byte addr[8];
  byte addresses[16][8];
  // byte addr0[8];
  // byte addr1[8];
  // byte addr2[8];
  // byte addr3[8];
  // byte addr4[8];
  // byte addr5[8];
  // byte addr6[8];
  // byte addr7[8];
  // byte addr8[8];
  // byte addr9[8];
  // byte addr10[8];
  // byte addr11[8];
  // byte addr12[8];
  // byte addr13[8];
  // byte addr14[8];
  // byte addr15[8];

  int num_sensors_passed = 0;
  // int intSensorsPassed = 0;

  Serial.println(F("Looking for 1-wire devices..."));
  delay(1000);
  Serial.println(F("Connect sensor 1..."));
  do {
    while (oneWire.search(addr)) {
      if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.print(F("CRC is not valid!\n"));
        Serial.println();
        Serial.print(F("CRC:  "));
        Serial.print(OneWire::crc8(addr, 7));
        Serial.print(F("addr: "));
        Serial.println(addr[7]);
        delay(100);
        break;
      }
      if (addr[0] != 0x28) {
        Serial.println(" Not a DS18b20");  //The least significant byte of the DS18b20 should be 0x28
        break;
      }

      // FIXME: Hasn't been tested yet
      if (!check_whether_address_seen(addr, addresses, num_sensors_passed)) {
            memcpy(addresses[num_sensors_passed], addr, 8);  //Store the address in the next available location
            num_sensors_passed++;
            Serial.print("Assigned to " + String(num_sensors_passed - 1) + ": ");
            printAddress(addresses[num_sensors_passed]);
            Serial.println();
            Serial.println("Sensors passed = " + String(num_sensors_passed) + "\n");
            Serial.println("Add sensor  " + String(num_sensors_passed + 1));
      } 


      // Above here just checks first element of CRC
      // switch (intSensorsPassed) {
      //   case 0:
      //     memcpy(addr0, addr, 8);  //Store the first address in addr0
      //     intSensorsPassed++;
      //     Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //     printAddress(addr0);
      //     Serial.println();
      //     Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //     Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     break;

      //   case 1:
      //     //check if the address was earlier identified
      //     if (!check_whether_address_seen(addr, addresses, intSensorsPassed)) {
      //       memcpy(addresses[num_sensors_passed], addr, 8);  //Store the addess in the next available location
      //       num_sensors_passed++;
      //       Serial.print("Assigned to " + String(num_sensors_passed - 1) + ": ");
      //       printAddress(addresses[num_sensors_passed]);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(num_sensors_passed) + "\n");
      //       Serial.println("Add sensor  " + String(num_sensors_passed + 1));
      //       // memcpy(addr1, addr, 8);  //Store the addess in addr1
      //       // intSensorsPassed++;
      //       // Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       // printAddress(addr1);
      //       // Serial.println();
      //       // Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       // Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 2:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0)) {
      //       memcpy(addr2, addr, 8);  //Store the addess in addr2
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr2);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 3:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0)) {
      //       memcpy(addr3, addr, 8);  //Store the addess in addr3
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr1);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 4:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0)) {
      //       memcpy(addr4, addr, 8);  //Store the addess in addr4
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr4);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 5:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0)) {
      //       memcpy(addr5, addr, 8);  //Store the addess in addr5
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr5);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 6:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0)) {
      //       memcpy(addr6, addr, 8);  //Store the addess in addr6
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr6);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 7:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0)) {
      //       memcpy(addr7, addr, 8);  //Store the addess in addr7
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr7);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 8:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0)) {
      //       memcpy(addr8, addr, 8);  //Store the addess in addr8
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr8);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 9:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0)) {
      //       memcpy(addr9, addr, 8);  //Store the addess in addr9
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr9);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 10:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0)) {
      //       memcpy(addr10, addr, 8);  //Store the addess in addr10
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr10);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 11:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0)) {
      //       memcpy(addr11, addr, 8);  //Store the addess in addr11
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr11);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 12:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0)) {
      //       memcpy(addr12, addr, 8);  //Store the addess in addr12
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr12);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 13:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0) && (memcmp(addr, addr12, 8) != 0)) {
      //       memcpy(addr13, addr, 8);  //Store the addess in addr13
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr13);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 14:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0) && (memcmp(addr, addr12, 8) != 0) && (memcmp(addr, addr13, 8) != 0)) {
      //       memcpy(addr14, addr, 8);  //Store the addess in addr14
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr14);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      //   case 15:
      //     //check if the address was earlier identified
      //     if ((memcmp(addr, addr0, 8) != 0) && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0) && (memcmp(addr, addr12, 8) != 0) && (memcmp(addr, addr13, 8) != 0) && (memcmp(addr, addr14, 8) != 0)) {
      //       memcpy(addr15, addr, 8);  //Store the addess in addr15
      //       intSensorsPassed++;
      //       Serial.print("Assigned to " + String(intSensorsPassed - 1) + ": ");
      //       printAddress(addr15);
      //       Serial.println();
      //       Serial.println("Sensors passed = " + String(intSensorsPassed) + "\n");
      //       Serial.println("Add sensor  " + String(intSensorsPassed + 1));
      //     }  //End if the new address obtained is the same as the last iteration of intSensorsPassed
      //     break;

      // }  // End switch case
    }    // End while sensors Passed
    delay(500);
  } while (num_sensors_passed < eeprom_object.num_ds18b20);  //This would hang up the program until this number is passed...
                                                           //Store the detected addresses in EEPROM
  
  
  // Planned Change: new 2D arrays for addresses
  for (int z = 0; z < 8; z++) {
    for (int i = 0; i < 16; i++){
      eeprom_object.ds18b20_sensor_addresses[i][z] = addresses[i][z]; 
    }
    // eeprom_object.ds18b20_sensor0[z] = addr0[z];
    // eeprom_object.ds18b20_sensor1[z] = addr1[z];
    // eeprom_object.ds18b20_sensor2[z] = addr2[z];
    // eeprom_object.ds18b20_sensor3[z] = addr3[z];
    // eeprom_object.ds18b20_sensor4[z] = addr4[z];
    // eeprom_object.ds18b20_sensor5[z] = addr5[z];
    // eeprom_object.ds18b20_sensor6[z] = addr6[z];
    // eeprom_object.ds18b20_sensor7[z] = addr7[z];
    // eeprom_object.ds18b20_sensor8[z] = addr8[z];
    // eeprom_object.ds18b20_sensor9[z] = addr9[z];
    // eeprom_object.ds18b20_sensor10[z] = addr10[z];
    // eeprom_object.ds18b20_sensor11[z] = addr11[z];
    // eeprom_object.ds18b20_sensor12[z] = addr12[z];
    // eeprom_object.ds18b20_sensor13[z] = addr13[z];
    // eeprom_object.ds18b20_sensor14[z] = addr14[z];
    // eeprom_object.ds18b20_sensor15[z] = addr15[z];
  }
  Serial.println(F("Iterative addressing of ds18b20 sensors complete."));
}

bool configuration_changed = false;
// Planned Change: improve saving functionality, possibly use a boolean

//-----Menu Routine----------------------------------------------------
void menu() {

  if (Serial.available() > 0) {
    Serial.read();  //clear serial input buffer
  }

  // Planned Change: IDnum has been removed from EEPROM
  // itoa(eeprom_object.IDnum, a, 10);  //convert IDnum to character array

  // if (eeprom_object.IDnum < 10) {  // for naming filename
  //   filename[0] = '0';             // put into filename[] array
  //   filename[1] = '0';
  //   filename[2] = a[0];
  // } else if (eeprom_object.IDnum < 100) {
  //   filename[0] = '0';  // put into filename[] array
  //   filename[1] = a[0];
  //   filename[2] = a[1];
  // } else {
  //   filename[0] = a[0];  // put into filename[] array
  //   filename[1] = a[1];
  //   filename[2] = a[2];
  // }

  print_board_info();
  print_menu_options();

  menutimeout = millis() + 60000;  //wait for user input, was 10 sec but 60 sec is better for new users
  while (millis() < menutimeout) {

    menuinput = 120;  //will default to ascii 120 "x"
    if (Serial.available() > 0) {
      menuinput = Serial.read();  //get user input
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }

  switch (menuinput) {  //switch cases for menu input, Note the case# corresponds to input in ASCII format
    case 98:            //"b" for iterating connection of ds18b20 temperature sensors
      identify_temperature_sensors();
      menu();
      break;

    case 99:  // "c" for set clock-----------------------------------------------
      set_clock();
      menu();
      break;

    case 112:               // "p" for prime pumps--------------------------------
      pump_priming_prompt();
      menu();
      break;

    case 102:  // "f" for checking files on sdcard. Non functional
      Serial.println(F("Getting sdcard information..."));
      sdCheck();
      menu();
      break;

    case 103:  // "g" for enabling or disabling Radio Transmission.
      radio_prompt();
      menu();
      break;

    case 104:  // "h" for enabling or disabling troubleshooting/demo continuous loop.
      troubleshooting_prompt();
      menu();
      break;

    case 105:  // "i" for set ID numbers-----------------------------------------

      Serial.println(F("Set network ID numbers (ProjectID, BoardID, NodeID, GatewayID):"));  // set ProjectID, BoardIDnum, nodeID, and gatewayID numbers
      get_integer_input();                                                                   //otherwise the first input is always 0?

      set_project_id();

      //
      // set_board_id();

      set_node_id();

      set_gateway_id();

      manager.setThisAddress(eeprom_object.nodeID);  // Set Radio Address

      menu();  // go back to menu
      break;

    case 97:  // "a" for Setting RTC Alarms for measurement interval
      Serial.println(F("Define measurement interval in minutes."));
      Serial.flush();
      get_integer_input();  //otherwise the first input is always 0?
      Serial.println();
      Serial.print(F("Current Measurement Interval (Minutes):  "));
      Serial.println(eeprom_object.ALARM_1_Interval);
      Serial.println();
      get_integer_input();
      eeprom_object.ALARM_1_Interval = indata;
      Serial.print(F("Measurement Interval set to:  "));
      Serial.println(eeprom_object.ALARM_1_Interval);
      delay(20);
      menu();
      break;

    case 115:  // "s" for switch water_manager----------------------------------
      Serial.println(F("Switch Water Manager: "));
      get_integer_input();  // otherwise the first input is always 0?
      Serial.println();
      Serial.println(F("Enable Water Manager?"));
      Serial.print(F("Current Status: "));
      Serial.println(eeprom_object.is_water_manager_on);  // returns 0 for false and 1 for true
      get_integer_input();
      eeprom_object.is_water_manager_on = indata;
      delay(20);
      menu();
      break;

    case 116:  // "t" for test measurements-------------------------------------
      Serial.println(F("Test measurements:"));
      get_integer_input();  //otherwise the first input is always 0?
      Serial.println();
      test_measurements();
      delay(10);
      menu();
      break;

    case 119:                             // "w" for setting water threshold levels and min time between irrigation events and reporting of resistance values
      get_integer_input();                //otherwise the first input is always 0?
      water_management_group_settings();  //void function

      delay(20);
      Serial.println(F("Do you want to include raw resistance values (for all sensors/groups) in the data string? Type 1 for true, Type 0 for false"));  // specify if resistance is desired in data string. true = 1 false = 0
      Serial.println();
      Serial.print(F("Current Value: "));
      Serial.print(eeprom_object.include_resistance);
      get_integer_input();
      eeprom_object.include_resistance = indata;
      menu();
      break;

    case 100:  // "d" for Download all data in sd card----------------------------
      Serial.println(F("Download all data:"));
      delay(100);

      myfile = SD.open(filename);  // open file

      if (myfile) {

        while (myfile.available()) {  // read file and print to Serial COM port, Note this will be slow with alot of data due to chip limitations. A desktop with a chip reader is nearly instantaneous.
          Serial.write(myfile.read());
        }
        myfile.close();

      } else {
        Serial.println(F("Error opening file"));
        // eeprom_object.error_log->sd_struct.open_file_failure = true;
        eeprom_object.error_log.sd_struct.open_file_failure = true;
      }
      delay(10000);  //Give time for user to copy data...
      menu();
      break;

    case 101:  // "e" for erase data on sd card-------------------------------------------
      erase_sd();
      menu();
      break;


    case 111:               //"o" for calibration / fixed resistor settings----------------------------
      get_integer_input();  //Otherwise first input is always 0?
      Serial.flush();       //unneeded?
      Serial.println(F("Define calibration and fixed resistor settings..."));
      Serial.println(F("Is a calibration resistor present?, 1 = true OR 0 = false"));
      Serial.print(F("Current value:  "));
      Serial.println(eeprom_object.calibration_resistor_present);
      get_integer_input();
      eeprom_object.calibration_resistor_present = indata;

      Serial.println(F("Muliplexor channel location of the calibration resistor. 0 to 15"));
      Serial.print(F("Current value:  "));
      Serial.println(eeprom_object.cal_resistor_loc);
      get_integer_input();
      eeprom_object.cal_resistor_loc = indata;

      Serial.println(F("Value of the calibration resistor in ohms. A float"));
      Serial.print(F("Current value:  "));
      Serial.println(eeprom_object.cal_resistor_val);
      get_integer_input();
      eeprom_object.cal_resistor_val = indata;

      Serial.println(F("The value of the FIXED resistor in ohms. A float"));
      Serial.print(F("Current value:  "));
      Serial.println(eeprom_object.fixed_resistor_val);
      get_integer_input();
      eeprom_object.fixed_resistor_val = indata;

      menu();
      break;

    case 110:  //"n" for sepecifying number of WM sensors & grouping | 11/18/2021 untested, not sure if will work
      // Work in Progress: Commented out to allow for compilation
      {
        // get_integer_input();  //otherwise the first input is always 0?
        // Serial.println(F("Specify number of WaterMark sensors and averaging instructions."));
        // Serial.println(F("If you would like to continue, type 1. Press any other key to return to main menu."));
        // get_integer_input();
        // if (indata != 1) {
        //   delay(1000);
        //   Serial.println(F("Returning to main menu."));
        //   menu();
        //   break;
        // } else {
        //   Serial.println(F("Clearing Existing Grouping Data."));
        //   for (int z = 0; z < 16; z++) {
        //     // FIXME: Update to use new 2D array version 
        //     // eeprom_object.WM_group1[z] = -1;
        //     // eeprom_object.WM_group2[z] = -1;
        //     // eeprom_object.WM_group3[z] = -1;
        //     // eeprom_object.WM_group4[z] = -1;
        //   }
        // }
        // delay(1000);
        // Serial.println(F("Proceed with WaterMark sensor specification."));
        // Serial.println(F("Number of WaterMark sensors. Must be installed in sequential order 1-16"));
        // Serial.print(F("Current value:  "));
        // Serial.println(eeprom_object.num_WM);
        // get_integer_input();
        // eeprom_object.num_WM = indata;

        // Serial.println(F("Determine averaging instructions for the WaterMark sensors."));
        // Serial.println(F("Four groups of watermark sensors can be defined based on their channel position of the Multiplexor."));
        // Serial.println(F("Specify 0 for no averaging routine."));
        // Serial.println(F("Note that the water management routine toggles relays based on group averages."));

        // Serial.print(F("Number of channels to average, Group1: "));  //This works great! repeat for each group 1-4
        // get_integer_input();
        // int num_sensors = indata;
        // Serial.print(F("Number sensors: "));
        // Serial.println(num_sensors);
        // int num_sensors_group1 = num_sensors;
        // // Work in Progress: Commented out to allow for compilation
        // // eeprom_object.n_channels_wm_group1 = num_sensors;

        // //Attempt a less memory intensive method------
        // for (int i = 0; i < num_sensors; i++) {
        //   if (num_sensors == 0) {
        //     break;  //If 0 sensors are selected, break out of for loop
        //   }
        //   int sensor_num = i + 1;
        //   Serial.print(F("Specify sensor number "));
        //   Serial.print(sensor_num);
        //   Serial.println(F("  in group1: "));
        //   get_integer_input();
        //   // Work in Progress: Commented out to allow for compilation
        //   eeprom_object.WM_group1[i] = indata;
        // }

        // Serial.print(F("Number of channels to average, Group2: "));
        // get_integer_input();
        // num_sensors = indata;
        // Serial.print(F("Number sensors: "));
        // Serial.println(num_sensors);
        // int num_sensors_group2 = num_sensors;
        // eeprom_object.n_channels_wm_group2 = num_sensors;
        // for (int i = 0; i < num_sensors; i++) {
        //   if (num_sensors == 0) {
        //     break;
        //   }
        //   int sensor_num = i + 1;
        //   Serial.print(F("Specify sensor number "));
        //   Serial.print(sensor_num);
        //   Serial.println(F("  in group2: "));
        //   get_integer_input();
        //   eeprom_object.WM_group2[i] = indata;
        // }

        // Serial.print(F("Number of channels to average, Group3: "));
        // get_integer_input();
        // num_sensors = indata;
        // Serial.print(F("Number sensors: "));
        // Serial.println(num_sensors);
        // int num_sensors_group3 = num_sensors;
        // eeprom_object.n_channels_wm_group3 = num_sensors;
        // for (int i = 0; i < num_sensors; i++) {
        //   if (num_sensors == 0) {
        //     break;
        //   }
        //   int sensor_num = i + 1;
        //   Serial.print(F("Specify sensor number "));
        //   Serial.print(sensor_num);
        //   Serial.println(F("  in group3: "));
        //   get_integer_input();
        //   eeprom_object.WM_group3[i] = indata;
        // }

        // Serial.print(F("Number of channels to average, Group4: "));
        // get_integer_input();
        // num_sensors = indata;
        // Serial.print(F("Number sensors: "));
        // Serial.println(num_sensors);
        // int num_sensors_group4 = num_sensors;
        // eeprom_object.n_channels_wm_group4 = num_sensors;
        // for (int i = 0; i < num_sensors; i++) {
        //   if (num_sensors == 0) {
        //     break;
        //   }
        //   int sensor_num = i + 1;
        //   Serial.print(F("Specify sensor number "));
        //   Serial.print(sensor_num);
        //   Serial.println(F("  in group4: "));
        //   get_integer_input();
        //   eeprom_object.WM_group4[i] = indata;
        // }

        // Serial.println(F("Current Grouping Settings."));
        // Serial.print(F("Group1: "));
        // for (int i = 0; i < num_sensors_group1; i++) {
        //   Serial.print(eeprom_object.WM_group1[i]);
        //   Serial.print(F("  "));
        // }
        // Serial.println();
        // Serial.print(F("Group2: "));
        // for (int i = 0; i < num_sensors_group2; i++) {
        //   Serial.print(eeprom_object.WM_group2[i]);
        //   Serial.print(F("  "));
        // }
        // Serial.println();
        // Serial.print(F("Group3: "));
        // for (int i = 0; i < num_sensors_group3; i++) {
        //   Serial.print(eeprom_object.WM_group3[i]);
        //   Serial.print(F("  "));
        // }
        // Serial.println();
        // Serial.print(F("Group4: "));
        // for (int i = 0; i < num_sensors_group4; i++) {
        //   Serial.print(eeprom_object.WM_group4[i]);
        //   Serial.print(F("  "));
        // }
        // menu();
        // break;
      }
    case 120:  //"x" for Exit ---------------------------------------------------
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;


    default:  //Define default case, exit menu if no valid user input
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;
  }
  eeprom_address = 0;                         //clear eeprom_address
  EEPROM.put(eeprom_address, eeprom_object);  //store new settings (eeprom_object with structure eeprom_struct) to chip EEPROM
  eeprom_address = 0;                         //clear eeprom_address
  Set_ALARM_1_Interval();
  delay(100);
}

//-----Return sdCard information----------------------------------------- Nonfunctional

void sdCheck() {
  Sd2Card card;  // set up variables using the SD utility library functions
  SdVolume volume;
  SdFile root;
  // print the type of card
  Serial.println();
  Serial.print(F("Card type:         "));
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println(F("SD1"));
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println(F("SD2"));
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println(F("SDHC"));
      break;
    default:
      Serial.println(F("Unknown"));
  }
  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
  }

  Serial.print(F("Clusters:          "));
  Serial.println(volume.clusterCount());
  Serial.print(F("Blocks x Cluster:  "));
  Serial.println(volume.blocksPerCluster());

  Serial.print(F("Total Blocks:      "));
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();
  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print(F("Volume type is:    FAT"));
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
  volumesize *= volume.clusterCount();     // we'll have a lot of clusters
  volumesize /= 2;                         // SD card blocks are always 512 bytes (2 blocks are 1KB

  Serial.print(F("Volume size (Kb):  "));
  Serial.println(volumesize);
  Serial.print(F("Volume size (Mb):  "));
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print(F("Volume size (Gb):  "));
  Serial.println((float)volumesize / 1024.0);

  Serial.println(F("\nFiles found on the card (name, date and size in bytes): "));
  root.openRoot(volume);
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
}

//-----Test Measurements function-------------------------------------------------------------
void test_measurements() {
  readRTC();  //Read the RTC
  delay(50);
  Temp = getTemp();  //Read DS18B20 temperature sensors
  delay(50);
  read_watermark();  //Read the WaterMark Sensors
  delay(50);
  compile();  //Format data from subroutines into one string for transmission
}

void water_management_group_settings() {
  Serial.println(F("Which water management group would you like to change settings for? (1 to 4)"));

  get_integer_input();

  int fn_group;
  fn_group = indata;

  if (fn_group != 1 && fn_group != 2 && fn_group != 3 && fn_group != 4) {
    Serial.println(F("Invalid entry. Returning to main menu."));
    menu();
    return;
  }

  Serial.print(F("Defining water management settings for group #: "));
  Serial.println(fn_group);

  Serial.println(F("Current Settings: "));

  Serial.print(F("Matric potential threshold for irrigation:  "));
  Serial.println(eeprom_object.group_irr_thresholds[fn_group - 1]);  //-1 as 0 indexed
  Serial.print(F("Minimum time between irrigation events (minutes): "));
  Serial.println(eeprom_object.min_time_btwn_irr[fn_group - 1]);
  Serial.print(F("Duration of an irrigation event (seconds): "));
  Serial.println(eeprom_object.irr_period[fn_group - 1]);
  Serial.println();
  delay(1000);



  //Need to add option to define in minutes or seconds... 3/21/2023

  Serial.print(F("Please define the matric potential threshold (in kPa) which will trigger an irrigation event for group #: "));
  Serial.println(fn_group);
  get_integer_input();
  int user_threshold;
  user_threshold = indata;
  Serial.println();

  Serial.print(F("Please define the minimum time between irrigation events (in minutes) for group #: "));
  Serial.println(fn_group);
  get_integer_input();
  int32_t min_btwn;
  min_btwn = indata;
  Serial.println();

  delay(100);
  Serial.print(F("Please define the duration of the irrigation events (in seconds) for group #: "));
  Serial.println(fn_group);
  get_integer_input();
  int32_t sec_irr;
  sec_irr = indata;
  Serial.println();

  /*

  delay(100);
  Serial.print(F("Please define prohibited windows for irrigation events in group #: "));
  Serial.println(fn_group);
  (please write me :^))

  delay(100);
  Serial.print(F("Please define permitted windows for irrigation events in group #: "));
  Serial.println(fn_group);
  (please write me :^))
  */

  eeprom_object.group_irr_thresholds[fn_group - 1] = user_threshold;  // -1 due to 0 indexing of arrays
  eeprom_object.min_time_btwn_irr[fn_group - 1] = min_btwn;
  eeprom_object.irr_period[fn_group - 1] = sec_irr;
}

void onAlarm() {
  Serial.println("Alarm occured!");
}

//-----Set ALARM1 ---------------------------------
void Set_ALARM_1_Interval() {
  sei();
  wdt_disable();                         // turn off watchdog timer From ArduinoSoilH2O
  rtc.disable32K();                      // Turn off 32kHz output
  pinMode(RTC_Interrupt, INPUT_PULLUP);  // Making it so, that the alarm will trigger an interrupt

  //Schedule an alarm
  attachInterrupt(digitalPinToInterrupt(RTC_Interrupt), onAlarm, FALLING);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);  //stop oscillating signals at sqw pin
  rtc.disableAlarm(2);              //remove if using alarm2...
  if (!rtc.setAlarm1(
        rtc.now() + TimeSpan(0, 0, eeprom_object.ALARM_1_Interval, 0), DS3231_A1_Minute  // i.e. tells it to match minutes to value in timespan + now...
        )) {
    Serial.println(F("Error, alarm wasn't set!"));
  } else {
    Serial.println(F("Alarm is set!"));
  }
}

// Menu setting for irrigation sensor/timer based


// States: IRRIGATING_S(sensor based irrigation), IRRIGATING_M(measurement based irrigation), 
void events_loop(){
  
  check_for_singular_events();
  // Check match fields
  //check_for_sensor_based_irrigation();
  // Test a sensor measurement event everyday where seconds is 15
  // If sensor based irrigation needs to occur for certain group, schedule singular events to do so
  // Can have custom calcualtions for event span based on various factors
  // Add these events to queue

  // Check for scheduled events
  // Add to queue

  // Find earliest event in queue
  // process

  // If 

  //check_match_fields();

  // Function Name: 
  // First, check whether sensor based irrigation needs to be done based on the match fields
  // If it does, perform sensor based irrigation

  // If not, check for other events and perform them

  //sensor_based_irrigation();


  //check_for_measurement_events();



  // Check for timed events

  // Keep track of state






  // state is initally IDLE
  curr_state = IDLE;

  reset_events_queue();
  find_upcoming_events();

  // How to handle pulse type vavles with this loop????
  while (eeprom_object.events_queue_size > 0){
    // Timing of the pulse length, not every x ms
    // Check to make sure the valve is closed
    // Maybe attach another component that allows verififcant (set one of 4 output pins as input to verify valve state)
    //maintain_valves_open();


    // Prioritize scheduled irrigation events

    // Check the type of event and if irrigation, move to new queue
    for (int i = 0; i < eeprom_object.events_queue_size; i++){
      // Create a new function that finds an event by id/reference number and returns the pointer and input into this
      // get_event_by_id 
      //handle_event(get_event_by_id(eeprom_object.events_queue[i]));
    }
    // subfunctions will update num_events and we will keep track of events to remove
    // eventsQueue will be updated after the loop
    // If a function is non blocking it will move on to next iteration of loop
    check_for_singular_events();
    check_for_recurring_events();

    // Could try to error bounds using ms for radio or other precise events
    // Could use testcase and engineer some edge cases and regular cases and see whether the function works
    // Could also be measurement events

  }

  // Standard event on all valves 
  // Start one valve and can start another at same time and later time (e.g. one at noon and another 12:15)
  // Measuring events testcases (measuring event during irrigation period)
  // Radio( dummy function that prints to serial terminal that radio is occuring) transmissions during irrigation
  // Device is on and has to change something, 

  // Could use eeprom OR flash memory
  // RTC_DATA_ARR int events[50]
  // DS3231 RTC

  // More mini examples for that
  // Look into lookup tables and pulse type signaling(one signal to change state or wait for second state to change back for latching)
  // Valve type logic
  
  // Look into state diagrams


  // Start rtc, testcases, start working on state table
  
  //while (num_queue_events > 0):
    // go through events queue and if event is occuring now, call functions to handle based on event_type
    

    // radio functions and some other events may block, but not for long
    // for irrigation, do it nonblocking


}

int events_queue_size = 50;

// Populates eventsQueue with upcoming events
void find_upcoming_events(){
  // Get the current time
  DateTime now = rtc.now();
  
  for (int i = 0; i < eeprom_object.num_events; i++){
    if (events[i]->recurring){
      uint32_t current_unix_epoch_time = now.unixtime();  //get current unix epoch time
      uint32_t num_seconds_in_week = 604800;
      uint32_t shift = 259200;
      uint32_t num_seconds_since_week_start = (current_unix_epoch_time - shift) % num_seconds_in_week;

      uint32_t beginning_seconds_since_week_start = ( events[i]->span[0]->unixtime() - shift) % num_seconds_in_week;
      uint32_t ending_seconds_since_week_start = ( events[i]->span[1]->unixtime() - shift) % num_seconds_in_week;
      if (num_seconds_since_week_start >= beginning_seconds_since_week_start &&  num_seconds_since_week_start <= ending_seconds_since_week_start){
        // add to events queue
        // might want to add something to current time to find events very shortly in the future to ensure they don't get skipped
        // so probably current_time + some_small_offset
        // Work in Progress: Commented out to allow for compilation
        //eeprom_object.events_queue[events_queue_size] = *(events[i]->event_id);
        eeprom_object.events_queue_size++;
      }
    }
    else{
      // *(events[i]->span[0]) 
      // span is a DateTime array, and we have <= comparators we can compare 2 datetime objects
      // Making sure that the time now is in the range of the span
      // With arrow notation, we can access data elements of an objects using it's pointer
      if (now >= *(events[i]->span[0]) && now <= *(events[i]->span[1])){
        // Work in Progress: Commented out to allow for compilation
        //eeprom_object.events_queue[events_queue_size] = *(events[i]->event_id);
        eeprom_object.events_queue_size++;
      }
    }
  }
}

void reset_events_queue(){
  for (int i = 0; i < 50; i++){
    eeprom_object.events_queue[i] = -1;
  }
  eeprom_object.events_queue_size = 0;
}

// Work in Progress: Commented to allow for compilation
// int get_index_of_event_by_id(int id){
//   for (int i = 0; i < eeprom_object.num_events; i++){
//     if (events[i]->event_id == id){
//       return i;
//     }
//   }
// }

// example client on microcontroller, while still connected, stuck in client loop, still process time or other events or no?
// Consider 

// What states can I change to based on current state

// FIXME: Refactor and complete this function later
// void handle_event(event* e){
//   switch(e->event_type):
//     case 0:
//       // Check for radio events, can check state within state
//       // example: taking measurement and checking event time
//       perform_scheduled_irrigation(e);
//       break;
//     case 4:
//       //radio_event(e);
//       break;
// }




/*
Start of valve management code.
*/

void valve_menu(){
  Serial.println(F("Open_Irr Valve Menu..."));
  Serial.println();

  Serial.println(F("Make a selection."));
  Serial.println(F("1    <-     View Current Valves"));
  Serial.println(F("2    <-     Add New Valve"));
  Serial.println(F("3    <-     Remove Valve"));

  get_integer_input();  //from existing code using global int "indata"

  int choice = indata;
  if (choice == 1){
    print_all_valves();
  }
  else if (choice == 2){
    add_new_valve();
  }
  else if (choice == 3){
    remove_valve();
  }
}

void add_new_valve(){
  String model, link, power, valve_fittings, duty_cycle_type;
  int duty_cycle_ms, min_pressure_psi, max_pressure_psi;
  bool latching, idle_state_normally_closed, wiring_three_way;
  float resistance_ohms;
  
  JsonObject new_valve = predefined_valves_array.createNestedObject();
  new_valve["id"] = eeprom_object.current_valve_id;

  Serial.println(F("Enter the model of the valve: "));
  model = Serial.readString();
  new_valve["model"] = model;

  Serial.println(F("Enter the link of the valve: "));
  link = Serial.readString();
  new_valve["link"] = link;

  Serial.println(F("Enter the power of the valve in DC: "));
  power = Serial.readString();
  new_valve["power"] = power;

  Serial.println(F("Enter the fittings of the valve in DC: "));
  valve_fittings = Serial.readString();
  new_valve["valve_fittings"] = valve_fittings;

  Serial.println(F("If the valve is latching, enter 1, otherwise enter false: "));
  get_integer_input();
  if (indata == 1){
    latching = true;
  }
  else{
    latching = false;
  }
  new_valve["latching"] = latching;

  Serial.println(F("If the idle state is normally closed, enter 1, otherwise enter false: "));
  get_integer_input();
  if (indata == 1){
    idle_state_normally_closed = true;
  }
  else{
    idle_state_normally_closed = false;
  }
  new_valve["idle_state_normally_closed"] = idle_state_normally_closed;


  Serial.println(F("Enter the duty cycle type: "));
  duty_cycle_type = Serial.readString();
  new_valve["duty_cycle_type"] = duty_cycle_type;

  Serial.println(F("Enter the duty cycle time in ms: "));
  get_integer_input();
  duty_cycle_ms = indata;
  new_valve["duty_cycle_ms"] = duty_cycle_ms;

  Serial.println(F("If there is three way wiring, enter 1, otherwise enter 0: "));
  get_integer_input();
  if (indata == 1){
    wiring_three_way = true;
  }
  else{
    wiring_three_way = false;
  }
  new_valve["wiring_three_way"] = wiring_three_way;

  Serial.println(F("Enter the minimum pressure in PSI: "));
  get_integer_input();
  min_pressure_psi = indata;
  new_valve["pressure_min_psi"] =  min_pressure_psi;

  Serial.println(F("Enter the maximum pressure in PSI: "));
  get_integer_input();
  max_pressure_psi = indata;
  new_valve["pressure_max_psi"] = max_pressure_psi;

  // Need to use float input
  Serial.println(F("Enter the maximum pressure in PSI: "));
  get_integer_input();
  max_pressure_psi = indata;
  new_valve["resistance_ohms"] = resistance_ohms;
  
  eeprom_object.num_valves++;
  eeprom_object.current_valve_id++;
}

void remove_valve(){
  // https://arduinojson.org/v6/api/jsonarray/remove/
  // ArduinoJson::Remove causes memory leaks but this might be fixable using the garbage collector?

  Serial.println(F("Enter the index of the valve you want to remove (note predefined valves cannot be removed):"));
  get_integer_input();
  int choice = indata;

  predefined_valves_array.remove(choice);
  eeprom_object.num_valves--;
}

int get_valve_procedure(int valve_id){
  for (JsonObject o: predefined_valves_array){
    if (o["id"] == valve_id){
      serializeJsonPretty(o, Serial);
      //Serial.println(o["id"]);
      bool idle_state_normally_closed = o["idle_state_normally_closed"];
      bool latching = o["latching"];
      long duty_cycle_ms = o["duty_cycle_ms"];

      // Serial.println(idle_state_normally_closed);
      // Serial.println(latching);
      Serial.println(duty_cycle_ms);

      if (idle_state_normally_closed){
        if (latching){
          return 1;
        }
        else{
          return 2;
        }
      }
      else{
        if (latching){
          return 3;
        }
        else{
          return 4;
        }
      }
    }
  }  
}

// Saves valve procedure integers for all valves to EEPROM
void set_all_valve_integers(){
  for (JsonObject o: predefined_valves_array){
      bool idle_state_normally_closed = o["idle_state_normally_closed"];
      bool latching = o["latching"];
      int duty_cycle_ms = o["duty_cycle_ms"];
      int valve_id = o["valve_id"];
      if (idle_state_normally_closed){
        // Work in Progress: Commented out to allow for compilation
        // if (latching){
        //   eeprom_object.valve_procedure_integers[valve_id] = 1;
        // }
        // else{
        //   eeprom_object.valve_procedure_integers[valve_id] = 1;
        // }
      }
      else{
        // Work in Progress: Commented out to allow for compilation
        // if (latching){
        //   eeprom_object.valve_procedure_integers[valve_id] = 1;
        // }
        // else{
        //   eeprom_object.valve_procedure_integers[valve_id] = 1;
        // }
      }
  }
}

// void perform_valve_procedure(int procedure){
//   // idle state normally closed (circuit is closed) and latching
//   if (procedure == 1){
//     // can turn 
//   }
//   else if (procedure == 2){

//   }
//   else if (procedure == 3){

//   }
//   else if (procedure == 4){

//   }
//   else{
//     System.println("Invalid procedure");
//   }
// }

void print_all_valves(){
  serializeJsonPretty(predefined_valves_array, Serial);
}

void read_valve_data(){
  Serial.println(F("Reading in valve data from file."));
  StaticJsonDocument<10000> valveDoc;
  File valves_file = SD.open("valves.txt", FILE_READ);
  char buf[10000];
  valves_file.read(buf, 10000);
  DeserializationError error = deserializeJson(valveDoc, buf);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  for (JsonObject o: valveDoc["userDefinedValves"].as<JsonArray>()){
    JsonObject user_defined_valve = predefined_valves_array.createNestedObject();
    user_defined_valve["id"] = o["id"];
    user_defined_valve["model"] = o["model"];
    user_defined_valve["link"] = o["link"];
    user_defined_valve["power"] = o["power"];
    user_defined_valve["valve_fittings"] = o["valve_fittings"];
    user_defined_valve["latching"] = o["latching"];
    user_defined_valve["idle_state_normally_closed"] = o["idle_state_normally_closed"];
    user_defined_valve["duty_cycle_type"] = o["duty_cycle_type"];
    user_defined_valve["duty_cycle_ms"] = o["duty_cycle_ms"];
    user_defined_valve["wiring_three_way"] = o["wiring_three_way"];
    user_defined_valve["pressure_min_psi"] =  o["pressure_min_psi"];
    user_defined_valve["pressure_max_psi"] = o["pressure_max_psi"];
    user_defined_valve["resistance_ohms"] = o["resistance_ohms"];
  }
}

void write_valve_data(){
  // FixMe: If valve array hasn't changed, don't write anything to SD card
  Serial.println(F("Writing valve data to file."));
  StaticJsonDocument<10000> valveDoc;

  JsonArray user_defined_valves_array = valveDoc.createNestedArray("userDefinedValves");
  // Only user defined valves will be written to SD card
  // Start at inbdex 5 onwards for now, maybe change to a variable in eeprom if possible
  int i = 0;
  // Maybe rename predefined_valves_array as it stores both user and predefined valves
  num_predefined_valves;
  for (JsonObject o: predefined_valves_array){
    if (i >= num_predefined_valves()){
      JsonObject user_defined_valve = user_defined_valves_array.createNestedObject();
      user_defined_valve["id"] = o["id"];
      user_defined_valve["model"] = o["model"];
      user_defined_valve["link"] = o["link"];
      user_defined_valve["power"] = o["power"];
      user_defined_valve["valve_fittings"] = o["valve_fittings"];
      user_defined_valve["latching"] = o["latching"];
      user_defined_valve["idle_state_normally_closed"] = o["idle_state_normally_closed"];
      user_defined_valve["duty_cycle_type"] = o["duty_cycle_type"];
      user_defined_valve["duty_cycle_ms"] = o["duty_cycle_ms"];
      user_defined_valve["wiring_three_way"] = o["wiring_three_way"];
      user_defined_valve["pressure_min_psi"] =  o["pressure_min_psi"];
      user_defined_valve["pressure_max_psi"] = o["pressure_max_psi"];
      user_defined_valve["resistance_ohms"] = o["resistance_ohms"];
    }
    i++;
  }

  char json_array[10000];  // char array large enough
  Serial.print(F("Saving new Event as Json..."));
  // Serial.println();
  serializeJson(valveDoc, json_array);  //copy the info in the buffer to the array to use writeFile below
  //serializeJson(eventsLog, Serial);
  // //Serial.println(s);
  //writeFileSD("events.txt", json_array);  //filename limit of 13 chars
  
  SD.remove("valves.txt");
  writeFileSD("valves.txt", json_array);
}

/*
End of valve management code.
*/

/*
Start of event management code.
*/

//To be placed in menu() -> maybe rename to eventScheduleMenu?
void events_menu() {
  Serial.println(F("Open_Irr Event Scheduler Menu..."));
  Serial.println();

  Serial.println(F("Make a selection."));
  Serial.println(F("1    <-     View Current Schedule"));
  Serial.println(F("2    <-     Schedule New Event"));
  Serial.println(F("3    <-     Remove Event From Schedule"));
  Serial.println(F("4    <-     Remove All Events From Schedule"));
  Serial.println(F("5    <-     Clear Match Fields"));
  Serial.println(F("6    <-     Emergency Clear SD"));
  Serial.println(F("7    <-     Exit Events Menu"));

  

  delay(100);

  int menu_timeout = millis() + 10000; 
  int menu_input = -1;
  while (millis() < menu_timeout) {
    if (Serial.available() > 0) {
      get_integer_input();  //from existing code using global int "indata"
      menu_input = indata;
      break;
    }
  }
  if (menu_input == -1){
    return;
  }

  if (menu_input == 1 || menu_input == 3) {
    Serial.println(F("Scheduled Events: "));
    //print from timeEvaluation scheduled event Array (DateTime objects) with array indicies
    print_all_events();

    Serial.println(F("Measurement Match Fields: "));
    print_current_match_fields();

    //will also need to print from the oldEvent global instance to show the matchFields for measurement
    // TODO
    //print_measurement_interval_details();
    if (menu_input == 3) {
      Serial.println(F("Enter the index number of scheduled event to remove."));
      get_integer_input();
      remove_event(indata);
      Serial.println(F("Updated schedule"));
      print_all_events();
    }
  }
  else if (menu_input == 2) {
    event_scheduler_menu();
  }
  else if (menu_input == 4){
    remove_all_events();
  }
  else if (menu_input == 5){
    clear_match_fields();
  }
  else if (menu_input == 5){
    clear_sd();
  }
  else if (menu_input == 7){
    return;
  }
  events_menu();
  // FixMe: give user specified amount of time before exiting
  // If completed one option, the user is moved back to the top of the menu
  // Check Node
  // Default option is to allow users to go back through menu
}

void event_scheduler_menu(){
  Serial.println(F("Scheduling New Event..."));
  Serial.println();

  Serial.println(F("Select Event Type."));
  Serial.println(F("0     <-     Sensor Measurement"));
  Serial.println(F("1     <-     Timer-based Irrigation"));
  Serial.println(F("2     <-     Irrigation Permit window"));
  Serial.println(F("3     <-     Irrigation Deny window"));

  //Placeholder for other event types to schedule... Radio transmissions etc. possible...

  get_integer_input();

  int choice = indata;
  if (choice == 0) {
    measurement_event_scheduler_menu();
  }
  else if (choice == 1) {
    timer_based_scheduler_menu();
  }
  else if (choice == 2) {
    permit_deny_event_scheduler_menu();
  }
  // else if (choice == 3){
  //   deny_window_scheduler_menu();
  // }
  //Exit scope of permit/deny window (event type 2)
}

// void wakeup_routine(){
//   check_for_events();

// }



void measurement_event_scheduler_menu(){
  //Schedule Measurement Event (scheduling the sensor reading interval)
  Serial.println(F("Measurement type events are recurring by default."));

  //To schedule a Measurement, a Datetime struct must be evaluated, so we need to choose a matchField

  Serial.println(F("Select a DateTime Match Field Element for Scheduling Measurements."));
  Serial.println(F("Measurements will occur when DateTime elements pulled from the DS3231 RTC match the scheduled event time."));
  Serial.println(F("For example, to schedule measurements to occur every 15 minutes one would: "));
  Serial.println(F("  1) Select Mintues for the match field."));
  Serial.println(F("  2) Specify 4 match fields."));
  Serial.println(F("  3) Enter match fields one at a time (0,15,30,45)."));
  Serial.println(F("This results in measurements taken when the current time of the RTC has minute values of 0, 15, 30, and 45."));

  Serial.println();
  Serial.println(F("0     <-     Seconds"));
  Serial.println(F("1     <-     Minutes"));
  Serial.println(F("2     <-     Hours"));
  get_integer_input();

  //print_current_match_fields();

  int selectedMatchField = indata;  //0,1,2

  if (indata > 2 || indata < 0) {
    Serial.println(F("Invalid Entry."));
    //menu();
  }

  //user feedback
  if (selectedMatchField == 0) {
    Serial.println(F("Match Seconds Selected."));
  }
  else if (selectedMatchField == 1) {
    Serial.println(F("Match Crap Selected."));
  }
  else if (selectedMatchField == 2) { // saved
    Serial.println(F("Match Hours Selected."));
  }

  Serial.println(F("Enter Number of Match Field Entries to evaluate, 0 to 24."));
  get_integer_input();
  int num_match_fields = indata;

  //arbitrary limit?
  if (num_match_fields > 24 || num_match_fields == 0) {
    Serial.print(F("Invalid Entry."));
    //menu();
  }

  //enter integers as asked
  for (int i = 0; i < num_match_fields; i++) {
    Serial.print(F("Enter Value of Match Field Entry "));
    Serial.println(i + 1);
    get_integer_input();
    eeprom_object.matchFields[selectedMatchField][i] = indata;  //pass in the integer to match on


    // //check existing matchFields for updated values (to not write to eeprom needlessly)
    // if (newEvent.matchFields[i] != oldEvent.matchFields[i]) {
    //   //MatchFields have been updated
    //   updateMeasurementInterval = true;                   //set the global flag true
    //   oldEvent.matchFields[i] = newEvent.matchFields[i];  //pass the newEvent matchFields to the oldEvent matchFields
    // }
    // else {
    //   //No changes to matchFields
    //   //No save needed
    // }

    // TODO
    //Save the changes
    // if (updateMeasurementInterval) {
    //   //save oldEvent.matchFields to eeprom

    //   //Have to add that element to the eeprom struct
    //   eeprom_object.measurementMatchFields[i] = oldEvent.matchFields[i];
    // }
  }
  eeprom_object.numFields[selectedMatchField] = num_match_fields;

  Serial.println(F("Measurement Event Scheduling Completed."));

  //All matchFields Set.
  //Will need to check eeprom_object.measurementMatchFields[i] in timeEvaluation() function
  //Exit this submenu to exit to EEPROM.put(eeprom_address, eeprom_object); in the main menu() function.
}

void timer_based_scheduler_menu(){
  //Timer-based Irrigation Event
  Serial.println(F("Timer-based Irrigation Event Selected."));
  Serial.println();
  Serial.println(F("Timer-based Irrigation Events are scheduled as recurring events every week, unless a singular event is specified."));
  Serial.println(F("To schedule a singular event, enter 0, any other entry defaults to a recurring weekly event."));

  get_integer_input();

  if (indata == 0) {
    singular_event_scheduling_menu(1);
  }
  else {
    recurring_event_scheduler_menu(1);
    // if (indata < 60 && indata >= 0) {
    //   minute = indata;
    // }
    // else {
    //   Serial.println(F("Invalid Entry."));
    //   //menu();
    // }

    // Serial.println(F("Enter Irrigation Event Duration in Minutes"));
    // Serial.println();
    // get_integer_input();

    // if (indata > 0) {
    //   eventDurationMinutes = indata;
    //   //Pass the irrigation event duration somewhere to recall ->  this is currently (type long) eeprom_object.irr_period[4], one for each group...

    //   //may need refactored to store differently
    // }
  }

  Serial.println(F("Timer-based Irrigation Event Scheduling Completed."));
  print_all_events();

}

void permit_deny_event_scheduler_menu(){
    //Define irrigation permit/deny window
    int event_type;
    Serial.println(F("Irrigation Permit/Deny Windows Selected."));
    Serial.println();

    Serial.println(F("To define a permit window, enter 1, any other entry will result in a deny window."));
    get_integer_input();

    if (indata == 1) {
      Serial.println(F("Permit window selected."));
      event_type = 2;
    }
    else {
      Serial.println(F("Deny window selected."));
      event_type = 3;
    }
    Serial.println();

    Serial.println(F("Is the irrigation event permit/deny window recurring (regularly occuring at set times weekly) or singular?"));
    Serial.println(F("To schedule a singular permit/deny window, enter 0, any other entry defaults to a recurring weekly permit/deny window."));
    get_integer_input();

    if (indata == 0) {
      singular_event_scheduling_menu(event_type);
    }
    else {
      recurring_event_scheduler_menu(event_type);
    }
    Serial.println(F("Recurring Permit/Deny Window Addition Completed."));
}

void singular_event_scheduling_menu(int event_type){
  //Singular Event Schedule: configure start and end dateTime(s), Append the file storing DateTimes for evaluation
  Serial.println(F("Define a new singular event window."));
  Serial.println();
  
  Serial.println(F("Define Start and Stop dates for the new singular event window."));
  Serial.println();

  // May have to look into changing type from int to more specific type
  int years[2];
  int months[2];
  int days[2];
  int hours[2];
  int minutes[2];
  int seconds[2];

  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      Serial.println(F("Select Start date."));
    }
    else {
      Serial.println(F("Select End date."));
    }

    Serial.println(F("Enter Year."));
    get_integer_input();
    years[i] = indata;

    Serial.println(F("Enter Month."));
    get_integer_input();
    months[i] = indata;

    Serial.println(F("Enter Day of Month."));
    get_integer_input();
    days[i] = indata;

    Serial.println(F("Enter Hour of Day, considering a 24 hour clock (0 to 23). "));
    get_integer_input();
    hours[i] = indata;

    Serial.println(F("Enter Minute of Hour (0 to 59)."));
    get_integer_input();
    minutes[i] = indata;

    Serial.println(F("Enter Second of Minute (0 to 59)."));
    get_integer_input();
    seconds[i] = indata; //saved
  }

  DateTime* span[2];
  span[0] = new DateTime(years[0], months[0], days[0], hours[0], minutes[0], seconds[0]);
  span[1] = new DateTime(years[1], months[1], days[1], hours[1], minutes[1], seconds[1]);

  Serial.println(F("How many irrigation zones should consider this new event?"));
  Serial.println();
  get_integer_input();
  int numZones = indata;

  if (numZones >= 5 || numZones <= 0) {
    Serial.println(F("Invalid Entry."));
    //menu();
    return;
  }

  // fixed size for now, but might want to change later
  int zones[500];

  bool groups[4];
  for (int i = 0; i < 4; i++){
    groups[i] = false;
  }

  for (int i = 0; i < numZones; i++) {
    Serial.print(F("Enter irrigation zone receiving this new singular event."));
    get_integer_input();
    groups[indata-1] = true;
    //perhaps store an integer array in eeprom for each irrigation zone containing the index number of DateTime scheduling objects stored in a file?.
    //saved
  }

  schedule_new_event(groups, false, event_type, span);
  Serial.println(F("Singular window addition(s) completed."));
  //print_all_events();

}

void recurring_event_scheduler_menu(int event_type){
  uint8_t day, hour, minute, valve;
  uint16_t eventDurationMinutes;
  //create a new DateTime instance and populate with user provided information, fill the remaining information with the current values of the RTC

  //store that event in the timeEvaluation file

  //poll user for: day of week, hour of day, minute of hour
  //note that year, month, and second do not need to be provided by user and are not used...
  // seconds can be initialized to 0
  Serial.println(F("Define new Recurring Event."));
  Serial.println();
  Serial.println(F("Choose Event Day of Week."));
  Serial.println();
  Serial.println(F("0     <-     Sunday"));
  Serial.println(F("1     <-     Monday"));
  Serial.println(F("2     <-     Tuesday"));
  Serial.println(F("3     <-     Wednesday"));
  Serial.println(F("4     <-     Thursday"));
  Serial.println(F("5     <-     Friday"));
  Serial.println(F("6     <-     Saturday"));

  get_integer_input();

  if (indata < 7 && indata >= 0) {
    day = indata;
  }
  else {
    Serial.println(F("Invalid Entry."));
    //menu();
  }

  Serial.println(F("Enter Irrigation Event hour of day, considering a 24 hour clock (0 to 23)."));
  Serial.println();
  get_integer_input();

  if (indata < 24 && indata >= 0) {
    hour = indata;
  }
  else {
    Serial.println(F("Invalid Entry."));
    //menu();
  }

  Serial.println(F("Enter Event minute of hour (0 to 59)."));
  Serial.println();

  get_integer_input();

  // May have to look into changing type from int to more specific type
  int years[2];
  int months[2];
  int days[2];
  int hours[2];
  int minutes[2];
  int seconds[2];

  Serial.println(F("Define Start and Stop dates for the first iteration of the new recurring event."));
  Serial.println(F("The event will repeat weekly on a window matching the day of the week, hour, minute, and second of the initial event window."));
  //Serial.println(F("For example, say it is 8/16/24 and you want to schedule a recurring event."))
  Serial.println();


  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      Serial.println(F("Select Start date."));
    } else {
      Serial.println(F("Select End date."));
    }

    Serial.println(F("Enter Year."));
    get_integer_input();
    years[i] = indata;

    Serial.println(F("Enter Month."));
    get_integer_input();
    months[i] = indata;

    Serial.println(F("Enter Day of Month."));
    get_integer_input();
    days[i] = indata;
    
    Serial.println(F("Enter Hour of Day, considering a 24 hour clock (0 to 23). "));
    get_integer_input();
    hours[i] = indata;

    Serial.println(F("Enter Minute of Hour (0 to 59)."));
    get_integer_input();
    minutes[i] = indata;

    Serial.println(F("Enter Second of Minute (0 to 59)."));
    get_integer_input();
    seconds[i] = indata; //saved
  }

  DateTime* span[2];
  span[0] = new DateTime(years[0], months[0], days[0], hours[0], minutes[0], seconds[0]);
  span[1] = new DateTime(years[1], months[1], days[1], hours[1], minutes[1], seconds[1]);

  Serial.println(F("How many irrigation zones should receive this new recurring event?"));
  Serial.println();
  get_integer_input();

  int numZones = indata;
  if (numZones < 5 && numZones > 0) {

  } else {
    Serial.println(F("Invalid Entry."));
    //menu();
  }

   bool groups[4];
  for (int i = 0; i < 4; i++){
    groups[i] = false;
  }

  for (int i = 0; i < numZones; i++) {
    Serial.println(F("Enter irrigation zone receiving this new recurring event."));
    get_integer_input();
    //perhaps store an integer array in eeprom for each irrigation zone containing the index number of DateTime scheduling objects stored in a file?.
    groups[indata-1] = true;
  }

  schedule_new_event(groups, true, event_type, span);

  //For later Evaluation....
  //calculate the number of seconds elapsed since the beginning of the week
  //the DateTime object will still be used to calculate the seconds elapsed since the beginning of that week (implying we can determine the unix time at the end of the week)
  //evaluate this number
}

// Functions for managing events
void schedule_new_event(bool groups[4], bool recurring, uint8_t event_type, DateTime* span[2]){
  //event new_event(eeprom_object.current_event_reference_number, groups, recurring, event_type, span);
  events[eeprom_object.num_events] = new event(eeprom_object.current_event_reference_number, groups, recurring, event_type, span);
  eeprom_object.num_events++;
  eeprom_object.current_event_reference_number++;
  
  Serial.println(eeprom_object.num_events);
  //print_all_events();
}

void remove_event(int index){
  delete events[index];
  for (int i = index; i < eeprom_object.num_events - 1; i++){
    events[i] = events[i+1];
  }
  events[eeprom_object.num_events-1] = nullptr;
  eeprom_object.num_events--;
}

void remove_all_events(){
  while (eeprom_object.num_events > 0){
    remove_event(eeprom_object.num_events-1);
  }
  delete[] events;
  SD.remove("events.txt");
}
 
 // saved
// Clear JSON buffer
// Put counter in loop to populate array, print size
// Clear other buffers

// Look into whether writing new file or not
// Can compare 2 buffers or boolean to see if change
void read_events_data(){
  Serial.println(F("Reading in events data from file."));
  StaticJsonDocument<1048> eventsLog;
  File events_file = SD.open("events.txt", FILE_READ);
  if (!events_file){
    Serial.println(F("File does not exist, nothing was read."));
    return;
  }
  char buf[1048];
  events_file.read(buf, 1048);
  events_file.close();
  // Serial.println(buf);
  DeserializationError error = deserializeJson(eventsLog, buf);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  deserializeJson(eventsLog, temp_json_data);
  JsonArray eventsArray = eventsLog["eventsArray"].as<JsonArray>(); //saved

  int i = 0;
  for (JsonObject o: eventsLog["eventsArray"].as<JsonArray>()){
    events[i] = new event();
    for (int j = 0; j < 4; j++){
      events[i]->groups[j] = o["groups"][j];
    }
    //events[i]->group = o["group"];
    events[i]->recurring = o["recurring"];
    events[i]->event_type = o["event_type"];
    uint32_t start = o["start_span_seconds"];
    uint32_t end = o["end_span_seconds"];
    events[i]->span[0] = new DateTime(start);
    events[i]->span[1] = new DateTime(end);
    i++;
  }
  // for (int i = 0; i < eeprom_object.num_events; i++){
  //   //Serial.println(eventsLog[i]["group"]); //saved
  //   events[i] = new event();
  //   events[i]->group = eventsArray[i]["group"];
  //   events[i]->recurring = eventsArray[i]["recurring"];
  //   events[i]->event_type = eventsArray[i]["event_type"];
  //   // event["start_span_seconds"] = events[i]->span[0]->unixtime();
  //   // event["end_span_seconds"] = events[i]->span[1]->unixtime();
  // }
}

// void fill_old_events_data(){
//   StaticJsonDocument<10000> eventsLog;
//   // jsonBuffer.clear() or eventsLog.clear()
//   JsonArray eventsArray = eventsLog.createNestedArray("eventsArray");
//   for (int i = 0; i < eeprom_object.num_events; i++){
//     //events[i]->print();
//     JsonObject event = eventsArray.createNestedObject();
//     JsonArray groups = event.createNestedArray("groups");
//     for (int j = 0; j < 4; j++){
//       groups.add(events[i]->group[j]);
//     }
//     event["recurring"] = events[i]->recurring;
//     event["event_type"] = events[i]->event_type;
//     event["start_span_seconds"] = events[i]->span[0]->unixtime();
//     event["end_span_seconds"] = events[i]->span[1]->unixtime();
//   }

//   Serial.print(F("Saving new Event as Json..."));
//   serializeJson(eventsLog, temp_json_data);  //copy the info in the buffer to the array to use writeFile below
// }

void write_events_data(){
  if (eeprom_object.num_events == 0){
    Serial.print(F("No events currently scheduled, file was not written."));
    return;
  }
  // FixMe: If events array hasn't changed, don't write anything to SD card
  SD.remove("events.txt");
  delay(10);
  StaticJsonDocument<1048> eventsLog;
  // jsonBuffer.clear() or eventsLog.clear()
  JsonArray eventsArray = eventsLog.createNestedArray("eventsArray");
  for (int i = 0; i < eeprom_object.num_events; i++){
    events[i]->print();
    JsonObject event = eventsArray.createNestedObject();
    JsonArray groups = event.createNestedArray("groups");
    for (int j = 0; j < 4; j++){
      groups.add(events[i]->groups[j]);
    }
    event["recurring"] = events[i]->recurring;
    event["event_type"] = events[i]->event_type;
    event["start_span_seconds"] = events[i]->span[0]->unixtime();
    event["end_span_seconds"] = events[i]->span[1]->unixtime();
  }
  char json_array[1048];  // char array large enough
  Serial.print(F("Saving new Event as Json..."));
  serializeJson(eventsLog, json_array);  //copy the info in the buffer to the array to use writeFile below
  if (!strcmp(json_array, temp_json_data)){
    Serial.println(F("No changes in events data, nothing printed to file"));
    return;
  }
  serializeJson(eventsLog, Serial);
  
  
  //File f = SD.open("events.txt", FILE_WRITE);
  //serializeJson(eventsLog, f);
  //writeFileSD("events.txt", "Hello World!");
  writeFileSD("events.txt", json_array);  //filename limit of 13 chars
  
  //eventsArray.printTo(Serial);
  //serializeJsonPretty(eventsLog, Serial);

  // Prints the file contents, useful for debugging
  // myfile = SD.open("events.txt", FILE_READ);
  // while (myfile.available()) {  // read file and print to Serial COM port, Note this will be slow with alot of data due to chip limitations. A desktop with a chip reader is nearly instantaneous.
  //   Serial.write(myfile.read());
  // }
  // myfile.close();
}

void test_read_and_write_events_data(){

  const char s[300] = "{\"eventsArray\":[{\"groups\":[1,0,0,0],\"recurring\":1,\"event_type\":1,\"start_span_seconds\":173815966,\"end_span_seconds\":1738025966},{\"groups\":[0,1,0,0],\"recurring\":1,\"event_type\":1,\"start_span_seconds\":173815966,\"end_span_seconds\":1738025966}]}\0";
  writeFileSD("events.txt", s);
}

void print_all_events(){
  Serial.print(F("There are currently "));
  Serial.print(eeprom_object.num_events);
  Serial.println(F(" events."));
  Serial.println(F("Printing all events."));
  for (int i = 0; i < eeprom_object.num_events; i++){
    events[i]->print();
  }
}

void print_events_by_group(int group){
  for (int i = 0; i < eeprom_object.num_events; i++){
    if (events[i]->groups[i]){
      events[i]->print();
    }
  }
}

int min_time_to_next_event(){
  // Get the current time
  DateTime now = rtc.now();                           //needed to get unix time in next line
  uint32_t current_unix_epoch_time = now.unixtime();  //get current unix epoch time
  // Serial.println(current_unix_epoch_time);

  uint32_t num_seconds_in_week = 604800;
  uint32_t shift = 259200;
  uint32_t num_seconds_since_week_start = (current_unix_epoch_time - shift) % num_seconds_in_week;
  //Serial.println(num_seconds_since_week_start);

  for (int i = 0; i < eeprom_object.num_events; i++){
    if (events[i]->recurring){
      uint32_t beginnging_seconds_since_week_start = ( events[i]->span[0]->unixtime() - shift) % num_seconds_in_week;
      uint32_t ending_seconds_since_week_start = ( events[i]->span[1]->unixtime() - shift) % num_seconds_in_week;
      if (num_seconds_since_week_start >= beginnging_seconds_since_week_start &&  num_seconds_since_week_start <= ending_seconds_since_week_start){
        Serial.print("Recurring event indicated for the following group(s): ");
        for (int j = 0; j < 4; j++){
          if (events[i]->groups[j]){
            Serial.print(j+1);
            Serial.print(" ");
          }
        }
      }
    }
    // The event is singular
    else{

    }
  }
}

void check_for_sensor_based_irrigation(){
  //Get the current hours, minute, seconds
  //Loop through the elements of the match array and see if anything matches
  Serial.println(F("Checking for upcoming measurement events."));
  DateTime* times[8];
  DateTime current_time = rtc.now();
  for (int i = 0; i < 8; i++){
    times[i] = new DateTime(current_time.unixtime() + i);
    //Serial.println(times[i]->timestamp());
  }

  for (int i = 0; i < 8; i++){
    if (check_match_fields(*times[i])){
      // The next measurement occurs in i < 8 seconds, wait that long and then begin measurement

      // Schedule a singular event starting in i seconds, calculate the span and add it to the event queue

      // Change schedule_new_event to return an integer: the event ID, store it in events_queue
      // trigger global boolean to read sensors






      
      //delay(i*1000);

      // Create a event i seconds from now and 

      //perform_sensor_based_irrigation();
      //test_nonblocking_irrigation();
      if (check_match_fields(*times[i])){
        Serial.print(F("Match found in "));
        Serial.print(i);
        Serial.println(F(" seconds."));
      }
    }
  }

  // for (int i = 0; i < 3; i++){ //saved
  //   // Might not want to loop up to 24, instead just loop to number of match fields for that specific time unit
  //   for (int j = 0; j < 24; j++){
  //     if (i == 0){
  //       if (matchFields[i][j] == hour){
  //         return true;
  //       }
  //     }
  //     else if (i == 1){
  //       if (matchFields[i][j] == minute){
  //         return true;
  //       }
  //     }
  //     else if (i == 2){
  //       if (matchFields[i][j] == second){ //saved
  //         return true;
  //       }
  //     }
  //   }
  // }
  return false;
}

//Checks if the DateTime matches any of the match fields
bool check_match_fields(DateTime d){
  int currentFields[3];
  currentFields[0] = d.second();
  currentFields[1] = d.minute();
  currentFields[2] = d.hour();

  for (int i = 0; i < 3; i++){
    for (int j = 0; j < eeprom_object.numFields[i]; j++){
      if (eeprom_object.matchFields[i][j] == currentFields[i]){

        // FixMe: Print out time from RTC and the match field
        Serial.println("Found match, showing comaprison");
        Serial.println(eeprom_object.matchFields[i][j]);
        Serial.println(currentFields[i]);
        return true; // saved
      }
    }
  }
}

void print_current_match_fields(){
  for (int i = 0; i < 3; i++){
    print_match_fields_by_unit_of_time(i);
  }
}

void clear_match_fields(){
  for (int i = 0; i < 3; i++){
    eeprom_object.numFields[i] = 0;
  }
}

// unit of time can be 0, 1, 2 - seconds, minutes, hours
void print_match_fields_by_unit_of_time(int unit_of_time){
  if (unit_of_time == 0){
    Serial.println(F("Printing Match Fields for the Seconds"));
  }
  else if (unit_of_time == 1){
    Serial.println(F("Printing Match Fields for the Minutes"));
  }
  else if (unit_of_time == 2){
    Serial.println(F("Printing Match Fields for the Hours"));
  }
  for (int i = 0; i < eeprom_object.numFields[unit_of_time]; i++){
    Serial.print(eeprom_object.matchFields[unit_of_time][i]);
    Serial.print(F(" "));
  }
  Serial.println("");
}

int check_for_recurring_events(){
  // Get the current time
  DateTime now = rtc.now();                           //needed to get unix time in next line
  uint32_t current_unix_epoch_time = now.unixtime();  //get current unix epoch time

  uint32_t num_seconds_in_week = 604800;
  uint32_t shift = 259200;
  uint32_t num_seconds_since_week_start = (current_unix_epoch_time - shift) % num_seconds_in_week;


  for (int i = 0; i < eeprom_object.num_events; i++){
    if (events[i]->recurring){
      uint32_t beginnging_seconds_since_week_start = ( events[i]->span[0]->unixtime() - shift) % num_seconds_in_week;
      uint32_t ending_seconds_since_week_start = ( events[i]->span[1]->unixtime() - shift) % num_seconds_in_week;
      if (num_seconds_since_week_start >= beginnging_seconds_since_week_start &&  num_seconds_since_week_start <= ending_seconds_since_week_start){
        Serial.print("Recurring event indicated for the following group(s): ");
        for (int j = 0; j < 4; j++){
          if (events[i]->groups[j]){
            Serial.print(j+1);
            Serial.print(" ");
          }
        }
      }
    }
  }
  // // Loop through the irrigation windows for each group
  // for (int i = 0; i < 4; i++){
  //   for (int j = 0; j < eeprom_object.num_irrigation_events[i]; j++){
  //   if (num_seconds_since_week_start + 8  >= eeprom_object.irrigation_events[i][j].seconds_interval[0] && num_seconds_since_week_start <= eeprom_object.irrigation_events[i][j].seconds_interval[1]){
  //     // Proceed with irrigation
  //     Serial.print(F("Irrigation indicated for group"));
  //     Serial.println(i);
  //     // call test_nonblocking_irrigation or it's equivalent in the node code
  //   }
  //   }
  //}
  
  // See if the current time + 8 lies within a window, since windows are disjoin, it can only be in at most one
  // Windows should be disjoint, it doesn't make sense to have two overlapping windows for the same group, only one thing at once - either irrigating or not
}

// If an upcoming event is found, return the integer index within the events array, otherwise return -1
int check_for_singular_events(){
  // Get the current time
  DateTime now = rtc.now();                           //needed to get unix time in next line
  // uint32_t current_unix_epoch_time = now.unixtime();  //get current unix epoch time
  // Apparently putting 5 in here will result in 8 seconds being added?
  TimeSpan t(5);
  Serial.print((now + t).timestamp());
  for (int i = 0; i < eeprom_object.num_events; i++){
    //Serial.print(events[i]->span[0].timestamp());
    if (now + t >= *(events[i]->span[0]) && now + t <= *(events[i]->span[1])){
      // event is found, just print for now, but eventually  proceed to nonblocking irrigation
      Serial.print("Upcoming singular event indicated for the following group(s): ");
      for (int j = 0; j < 4; j++){
        if (events[i]->groups[j]){
          Serial.print(j+1);
          Serial.print(" ");
        }
      }
    }
  }
  
  // Serial.println(current_unix_epoch_time);

  // uint32_t num_seconds_in_week = 604800;
  // uint32_t shift = 259200;
  // uint32_t num_seconds_since_week_start = (current_unix_epoch_time - shift) % num_seconds_in_week;
  // Serial.println(num_seconds_since_week_start);

  // // Loop through the irrigation windows for each group
  // for (int i = 0; i < 4; i++){
  //   for (int j = 0; j < eeprom_object.num_irrigation_events[i]; j++){
  //   if (num_seconds_since_week_start + 8  >= eeprom_object.irrigation_events[i][j].seconds_interval[0] && num_seconds_since_week_start <= eeprom_object.irrigation_events[i][j].seconds_interval[1]){
  //     // Proceed with irrigation
  //     Serial.print(F("Irrigation indicated for group"));
  //     Serial.println(i);
  //     // call test_nonblocking_irrigation or it's equivalent in the node code
  //   }
  //   }
  //}
  
  // See if the current time + 8 lies within a window, since windows are disjoin, it can only be in at most one
  // Windows should be disjoint, it doesn't make sense to have two overlapping windows for the same group, only one thing at once - either irrigating or not
}

void clear_sd(){
  SD.remove("events.txt");
}

/*
End of event code
*/

// Given a pointer to a scheduled irrigation event, this function performs nonblocking irrigation for the specified groups
// void perform_scheduled_irrigation(struct event* e){
//   Datetime now = rtc.now();
//   if (e->state == IDLE){
//     for (int i = 0; i < 4; i++){
//       if (groups[i]){
//         open_valve(i);
//       }
//     }



//     //e->event_millis = millis();
//     // global for each group
//     //opened_miliis = millius();
//     // have pin_states and change state 
//     // when the timer is up, set the pin low and return to default state for each of the valves
//     // helper function to pull default state of valve
//     // normally closed vs normally open: closed swithc connection made, singal going through,
//     // the valve is the barrier, different components control the valve such as sensros or switches
    
//     // close swithc is closing circuit, closing switch opens valves
//     // switch is normally open (the circuit is normally open) in this case
//     // if (groups[0]){
//     //   // say x is the pulse time

//     //   // On some solenoids, it permits irrigation and some it stops
//     //   digitalWrite(in1, HIGH);
//     //   // add some valve logic 
//     //   group_states[0] = IRRIGATING;
//     // }
//     // if (groups[1]){
//     //   digitalWrite(in1, HIGH);
//     //   group_states[1] = IRRIGATING;
//     // }
//     // if (groups[2]){
//     //   digitalWrite(in1, HIGH);
//     //   group_states[2] = IRRIGATING;
//     // }
//     // if (groups[3]){
//     //   digitalWrite(in1, HIGH);
//     //   group_states[3] = IRRIGATING;
//     // }
//   }
//   else if (e->state == IRRIGATING && now >= e->span[1]){
//     // The event is over
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);

//     uint32_t current_unixtime = rtc.now().unixtime();  //reset the time of last irrigation event for that group
//     eeprom_object.last_irr_unix_time[WM_group_num - 1] = rtc.now().unixtime();  //reset the time of last irrigation event for that group

//     for (int i = 0; i < 4; i++){
//       if (groups[i]){
//         eeprom_object.last_irr_unix_time[i] = current_unxtime;
//         new_irr_event = true;  //set boolean true to update eeprom

//         // Maybe change function name to print_local_time
//         local_time(eeprom_object.last_irr_unix_time[i]);
//         delay(10);

//         // Don't know if still want to do this for scheduled events
//         // irrigation_prompt_string += 'G';
//         // irrigation_prompt_string += i;  //Amend relevant data to string for storage/transmission
//         // irrigation_prompt_string += ',';


//         // irrigation_prompt_string += WM_group_mean;
//         // irrigation_prompt_string += ',';

//         // for (int i = 0; i <= numChars; i++) {
//         //   if (local_time_irr_update[i] != '\0') {
//         //     irrigation_prompt_string += local_time_irr_update[i];  //The global variable keeping track of last irr event
//         //   } else {
//         //     break;
//         //   }
//         // }
//         // irrigation_prompt_string += ',';
//       }
//     }  
              
              
              
//   }
  
  
// }

// Uses valve logic to open valve for specific group
void open_valve(int group){
  // Retreive valve based on group

  // Call open, close normally open latching, non latching based on type
}

// Maybe add another piece of hardware that solely pulses the valves. This piece of hardware can be activated by this function or something
void maintain_valves_open(){
  for (int i = 0; i < 4; i++){
    // Work in Progress: Commented out to allow for compilation
    // if (!eeprom_object.valves[i].latching){
    //   // see if duty cyclce time has passed and need to send anotehr pulse
    // }
  }
}


void irrigate(){
  bool test_mode = true;
  for (int i = 0; i < 4; i++){
    group_is_done[i] = false;
  }

  // Group 1 and 4 will have means below the threshold and have times that guarantee irrgiation

  while (!group_is_done[0] || !group_is_done[1] || !group_is_done[2] || !group_is_done[3]){
    WM_irrigation_prompt(1, eeprom_object.group_irr_thresholds[0] - 5, eeprom_object.group_irr_thresholds[0], eeprom_object.last_irr_unix_time[0] - eeprom_object.min_time_btwn_irr[0] * 60, false);
    // Work in Progress: Commented out to allow for compilation
    // WM_irrigation_prompt(2, WM_group2_mean, eeprom_object.group_irr_thresholds[1], eeprom_object.last_irr_unix_time[1], false);
    // WM_irrigation_prompt(3, WM_group3_mean, eeprom_object.group_irr_thresholds[2], eeprom_object.last_irr_unix_time[2], false);
    WM_irrigation_prompt(4, eeprom_object.group_irr_thresholds[0] - 2, eeprom_object.group_irr_thresholds[3], eeprom_object.last_irr_unix_time[3] - eeprom_object.min_time_btwn_irr[3] * 60, false);
  }

  Serial.println(F("Irrigation done"));
}

// Planned Change: event has start, stop, e.g. threshold: 30, then evaluation criteria,

// Integers to identify each criteria, parameter passed into this function

// Read sensors, evaluate readings based on our criteria, then write an event tailored to the criteria

// Removing actual irrigation from this function and shift to an evaluation function
// bool 

// shell function
void evaluate_irr_criteria_1(int WM_group_num, int WM_group_mean, int WM_group_water_threshold, uint32_t last_irr_time_for_group){
  bool irrigate = false;


  // evaluate criteria

  if (irrigate){
    // create new event
    // probably singular
    // calculate and define the event span
  }
}

// Prototype sketch for latching valve module 12/17/24

//New prompt for the 4 threshold groups of sensors.
uint32_t WM_irrigation_prompt(int WM_group_num, int WM_group_mean, int WM_group_water_threshold, uint32_t last_irr_time_for_group, bool test_mode) {
  if (WM_group_num >= 1 && WM_group_num <= 4){
    if (group_states[WM_group_num-1] == IRRIGATING){
      // The irrigation time has passed
      if (millis() - group_millis[WM_group_num - 1] >= eeprom_object.irr_period[WM_group_num - 1] * 1000){
            Serial.print(F("Group "));
            delay(50);
            Serial.print(WM_group_num);
            delay(50);
            Serial.println(F(" done irrigating."));
            delay(50);
            Serial.print(F("Irr duration in ms: "));
            delay(50);
            long irr_duration_ms = eeprom_object.irr_period[WM_group_num - 1] * 1000;
            Serial.println(irr_duration_ms);
            delay(50);
            if (!test_mode){
              // Work in Progress: Commented out to allow for compilation
              // if (latchingValve){
                
              // }
              // else{
              //   // Could try 
              //   if (WM_group_num == 1){
              //     digitalWrite(in1, LOW);                                //open the respective relay pin, removing power to the pump
              //   }
              //   else if (WM_group_num == 2){
              //     digitalWrite(in2, LOW);                                //open the respective relay pin, removing power to the pump
              //   }
              //   else if (WM_group_num == 3){
              //     digitalWrite(in3, LOW);                                //open the respective relay pin, removing power to the pump
              //   }
              //   else{
              //     digitalWrite(in4, LOW);                                //open the respective relay pin, removing power to the pump
              //   }
              // }
              
              eeprom_object.last_irr_unix_time[WM_group_num - 1] = rtc.now().unixtime();  //reset the time of last irrigation event for that group
              new_irr_event = true;  //set boolean true to update eeprom

              // Maybe change function name to print_local_time
              local_time(eeprom_object.last_irr_unix_time[0]);
              delay(10);

              irrigation_prompt_string += 'G';
              irrigation_prompt_string += WM_group_num;  //Amend relevant data to string for storage/transmission
              irrigation_prompt_string += ',';
              irrigation_prompt_string += WM_group_mean;
              irrigation_prompt_string += ',';

              for (int i = 0; i <= numChars; i++) {
                if (local_time_irr_update[i] != '\0') {
                  global_last_irr_ending_time += local_time_irr_update[i];  //The global variable keeping track of last irr event
                } else {
                  break;
                }
              }
              irrigation_prompt_string += ',';
            }
            else{
              Serial.println("CURRENTLY IN TEST MODE: pipe would have closed now.");
              delay(50);
            }
            group_states[WM_group_num-1] = IDLE;
            group_is_done[WM_group_num-1] = true;

      }
    }
    else {
      // Work in Progress: Commented out to allow for compilation
      // if (there is an event){
      //   start irrigation //saved
      // }
      // if sensors indicate the need for a watering event (for each group threshold)-----
      if (WM_group_mean < eeprom_object.group_irr_thresholds[WM_group_num-1]) {
        if (group_states[WM_group_num-1] == IDLE){
          Serial.print(F("Need for watering event indicated for sensor group: "));
          delay(50);
          Serial.print(WM_group_num);
          delay(50);
          Serial.print(F("  with a mean of: "));
          delay(50);
          Serial.println(WM_group_mean);
          delay(50);
        }
        DateTime now = rtc.now();
        uint32_t current_unix_epoch_time = now.unixtime();
                                                                      //get current unix epoch time, THIS IS IN SECONDS!!!
        if (current_unix_epoch_time - last_irr_time_for_group >= (eeprom_object.min_time_btwn_irr[WM_group_num - 1] * 60)) {  //IF the time since last irrigation event is greater than or equal to the minnimum time between irrigation events (minutes*60=seconds) Do not use (minutes*60*1000 = milliseconds) as UNIX time is represented as seconds.
          // 2022/03/22 Note that this will need changed in future if separate timing differences are specified for each group-----
          Serial.println(F("The minimum time since last irrigation event has been exceeded. Proceed with irrigation"));
          delay(50);

          local_time(eeprom_object.last_irr_unix_time[0]);
          delay(10);

          for (int i = 0; i <= numChars; i++) {
            if (local_time_irr_update[i] != '\0') {
              global_last_irr_starting_time += local_time_irr_update[i];  //The global variable keeping track of last irr event
            } else {
              break;
            }
          }

          if (test_mode){
            Serial.println(F("CURRENTLY IN TEST MODE: pipe would have opened now."));
            delay(50);
          }
          else{
            // Work in Progress: Commented out to allow for compilation
            // if (latchingValve){

            // }
            // else{
            //   if (WM_group_num == 1){
            //     digitalWrite(in1, HIGH);                                   //provide power to pump on relay on respective pin
            //   }
            //   else if (WM_group_num == 2){
            //     digitalWrite(in2, HIGH);                                   //provide power to pump on relay on respective pin
            //   }
            //   else if (WM_group_num == 3){
            //     digitalWrite(in3, HIGH);                                   //provide power to pump on relay on respective pin
            //   }
            //   else {
            //     digitalWrite(in4, HIGH);                                   //provide power to pump on relay on respective pin
            //   }
            // }

            
            Serial.println(F("Pipe opened"));
            delay(100);
            delay(50);
          }
          group_millis[WM_group_num - 1] = millis();
          group_states[WM_group_num-1] = IRRIGATING;
        }
        //add condition to not overwater??
        //irr_count ++;
        //like incrementing irr_count for throwing flag if X events take place in Y time?
        //Then do something? or send flag to gateway?
        else {
          Serial.print(F("Minimum Time between irrigation events not reached for Group: "));  //declare that the minimum time between irrigations has not elapsed for specified group
          delay(50);
          Serial.print(WM_group_num);
          delay(50);
          Serial.print(F("  with a mean of: "));
          delay(50);
          Serial.println(WM_group_mean);
          delay(50);
          group_states[WM_group_num-1] = WAITING;

          if (!test_mode){
            //Report the group #, group mean, and a unix timestamp of the last irrigation event to the irrigation_prompt_string that gets saved etc.
            irrigation_prompt_string += 'G';
            irrigation_prompt_string += WM_group_num;
            irrigation_prompt_string += ',';
            irrigation_prompt_string += WM_group_mean;
            irrigation_prompt_string += ',';
            //different here than in case above
            //if minimum time between irrigations has not been exceeded, return the time of last irrigation event for the group
            local_time(eeprom_object.last_irr_unix_time[WM_group_num]);
            delay(10);

            for (int i = 0; i <= numChars; i++) {
              if (local_time_irr_update[i] != '\0') {
                irrigation_prompt_string += local_time_irr_update[i];  //The global variable keeping track of last irr event
              } else {
                break;
              }
            }
            irrigation_prompt_string += ',';
          }
          group_is_done[WM_group_num-1] = true;          
        }
      }
      else {
        Serial.print(F("Group: "));
        delay(50);
        Serial.print(WM_group_num);
        delay(50);
        Serial.print(F("  Mean: "));
        delay(50);
        Serial.print(WM_group_mean);
        delay(50);
        Serial.print(F(", Threshold water content of "));
        delay(50);
        Serial.print(eeprom_object.group_irr_thresholds[0]);
        delay(50);
        Serial.println(F("  has not been exceeded."));
        delay(50);
        if (!test_mode){
          //New print routine to add to the irrigation_prompt_string even when water threshold has not been reached
          //and min time has not elapsed
          irrigation_prompt_string += 'G';           //This is looped through for each group
          irrigation_prompt_string += WM_group_num;  //Amend relevant data to string for storage/transmission
          irrigation_prompt_string += ',';
          irrigation_prompt_string += WM_group_mean;
          irrigation_prompt_string += ',';

          local_time(eeprom_object.last_irr_unix_time[WM_group_num-1]);
          delay(10);

          for (int i = 0; i <= numChars; i++) {
            if (local_time_irr_update[i] != '\0') {
              irrigation_prompt_string += local_time_irr_update[i];  //The global variable keeping track of last irr event
            } else {
              break;
            }
          }

          irrigation_prompt_string += ',';
        }
        group_is_done[WM_group_num-1] = true;
      }
    }
  }
  else {
    Serial.println(F("Undefined Group number..."));
    delay(50);
  }  
}

//[10:51 AM] Bierer, Andrew - REE-ARS
//template for thought process
void somethingWorkingPlease(){
 
  //Read from SD card and have json file in a dynamically allocated buffer to pull from using dot access
    //one sub function
 
  //Read our large structure from EEPROM (can be done before SD card i dont think it matters)
    //one sub function
 
//one function
  //Check if it is time to do something
    //Iteritively deal with each "event-reference #" in the SD card file
      //->Checking the clock and using the "timeEvaluation" function with matchfields.
        //For ANY  of the event-references in the sd-card file, is it time to do something - or wait to do something that is scheduled before next wake?
          //No -> go to short sleep
          //Yes -> Okay, which # is this? (lets call it "activeEvent") read eeprom information for
 
          // int equal to active Event
 
 
 
         /* struct valveEventSchedule{
 uint8_t outputValve[4]; //Array to hold valve number
 uint8_t valveEventReference[40]; //Array to hold the event reference(s) - a static declaration of 10 references to consider? would be hard to do this dynamically with a limited and fixed eeprom size.
//output pin being set high
} eventReference;
*/
 
//One function
        //Evaluate the sd-card event-reference # (that it is time to do something) against the stored eeprom event-reference
          //For valves
            //i = 0-4 for valves
            //For valve event-references (ten array positions for each valve)
              //if j == activeEvent
                // Do the something that is being scheduled
                  //sub-function to do that something...
}