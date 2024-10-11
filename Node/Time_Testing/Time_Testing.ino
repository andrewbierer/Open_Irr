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

  // Initally all -1, if an event is upcoming for group i, the 
  int current_events[4];

  int num_to_update = 0;

  // 2D array with match fields, matchFields[0][i] are the match fields for hours, matchFields[1][i] the match fields for minutes, and matchFields[2][i] are the match fields for seconds
  // had to change uint16_t to int to avoid type errors with assignment using indata
  uint8_t matchFields[3][8];    //arbitrarily length 24? how many match fields are reasonable to expect?
  uint8_t numFields[3]; // Stores the number of fields for seconds, minutes, and hours respectively

  uint16_t num_events = 0;

  uint16_t current_event_reference_number = 0;

  // struct valveEventSchedule{
  // //   uint8_t outputValve[4]; //Array to hold valve number
  // //   uint8_t valveEventReference[40]; //Array to hold the event reference(s) - a static declaration of 10 references to consider? would be hard to do this dynamically with a limited and fixed eeprom size.
  // //  //output pin being set high
  //   uint8_t num_events[4]{ 0 };
  //   uint16_t schedule[4][10];
  // } eventReference;
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

//Working on storing the time evaluation information

//dateTime class objects are stored as 9 bytes 4 for the date and 5 for the time...

//eeprom storage/access seems appropriate if cannot be stored on Stack RAM as # of reads on eeprom is ~unlimited with ~100,000 write/erase cycles

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

void setup() {
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

  read_events_data();
  eeprom_object.current_event_reference_number = 0; //saved

  Serial.println("Initialization Completed"); //saved


  // setRTCInterrupt();
  // Serial.println(eeprom_object.num_irrigation_events[0]);
  // // set_alarms();
  events_menu(); //saved
  //eeprom_object.num_irrigation_events[0] = 0;

  // // Serial.println(eeprom_object.num_to_update);
  // // eeprom_object.num_to_update++;
  // // // eeprom_object.latchingValve = false;
  // // // Serial.println(eeprom_object.latchingValve);
  // // // eeprom_object.latchingValve = true;
  // eeprom_object.num_irrigation_events[0]++;
  write_events_data(); //saved
  updateEEPROM(); //saved

}

void loop() {
  Serial.println(F("Woke up and checking for upcoming irrigation events."));
  // Read in current windows for permit, deny, and irrigation
  read_events_data();
  // Find the minimum amount of time to the next event, delay that long

  check_for_singular_events();

  check_for_recurring_events();
  // run the events_processing_loop
  // First, loop through all events, find ones occuring now and fill into array
  // Right now, we only consider sensor based irrigation and event based irrigation
  // while (more events)
  // Get current time
  // Find events occuring now or right before now
  // Fill indices in array and process one by one
  // 
  // Go through array one by one and process events
  // irrigation events 
  // have an array of current events, storing index numbers and fill it using function
  // 
  // // If within 8 seconds of irrigation event, begin irrigation
  //check_for_measurement_events();
  // // // If rtc.now() of next irrigation event - rtc.now() < 8, then stay on, otherwise sleep
  
  // //Serial.println(F("No irrigation events occuring in the near future, going back to sleep"));
  // Write current windows for permit, deny and irrigation
  //events_menu();
  write_events_data();
  set_alarms();
  updateEEPROM();
  Low_Power_Sleep();
}

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

  get_integer_input();  //from existing code using global int "indata"

  delay(100);

  if (indata == 1 || indata == 3) {
    Serial.println(F("Scheduled Events: "));
    //print from timeEvaluation scheduled event Array (DateTime objects) with array indicies
    print_all_events();

    Serial.println(F("Measurement Match Fields: "));
    print_current_match_fields();

    //will also need to print from the oldEvent global instance to show the matchFields for measurement
    // TODO
    //print_measurement_interval_details();
    if (indata == 3) {
      Serial.println(F("Enter the index number of scheduled event to remove."));
      get_integer_input();
      remove_event(indata);
      Serial.println(F("Updated schedule"));
      print_all_events();
    }
  }
  else if (indata == 2) {
    event_scheduler_menu();
  }
  else if (indata == 4){
    remove_all_events();
  }
  else if (indata == 5){
    clear_match_fields();
  }
  else if (indata == 5){
    clear_sd();
  }
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

  // for (int i = 0; i < 4; i++){
  //   if (groups[i]){
  //     eeprom_object.eventReference.schedule[eeprom_object.eventReference.] = 
  //   }
  // }
  
  Serial.println(eeprom_object.num_events);
  //global_num_events;
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
  StaticJsonDocument<10000> eventsLog;
  File events_file = SD.open("events.txt", FILE_READ);
  char buf[10000];
  events_file.read(buf, 10000);
  // Serial.println(buf);
  DeserializationError error = deserializeJson(eventsLog, buf);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  //JsonArray eventsArray = eventsLog["eventsArray"].as<JsonArray>(); //saved

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

void write_events_data(){
  StaticJsonDocument<10000> eventsLog;
  // jsonBuffer.clear() or eventsLog.clear()
  JsonArray eventsArray = eventsLog.createNestedArray("eventsArray");
  for (int i = 0; i < eeprom_object.num_events; i++){
    //events[i]->print();
    JsonObject event = eventsArray.createNestedObject();
    JsonArray groups = event.createNestedArray("groups");
    //event["groups"] = events[i]->groups;
    for (int j = 0; j < 4; j++){
      groups.add(events[i]->groups[j]);
    }
    event["recurring"] = events[i]->recurring;
    event["event_type"] = events[i]->event_type;
    event["start_span_seconds"] = events[i]->span[0]->unixtime();
    event["end_span_seconds"] = events[i]->span[1]->unixtime();
  }
  char json_array[10000];  // char array large enough
  Serial.print(F("Saving new Event as Json..."));
  // Serial.println();
  serializeJson(eventsLog, json_array);  //copy the info in the buffer to the array to use writeFile below
  //serializeJson(eventsLog, Serial);
  // //Serial.println(s);
  //writeFileSD("events.txt", json_array);  //filename limit of 13 chars
  
  SD.remove("events.txt");
  writeFileSD("events.txt", json_array);
  //eventsArray.printTo(Serial);
  //serializeJsonPretty(eventsLog, Serial);

  // Prints the file contents, useful for debugging
  // myfile = SD.open("events.txt", FILE_READ);
  // while (myfile.available()) {  // read file and print to Serial COM port, Note this will be slow with alot of data due to chip limitations. A desktop with a chip reader is nearly instantaneous.
  //   Serial.write(myfile.read());
  // }
  // myfile.close();
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

bool check_for_measurement_events(){
  //Get the current hours, minute, seconds
  //Loop through the elements of the match array and see if anything matches
  Serial.println(F("Checking for upcoming measurement events."));
  DateTime* times[8];
  DateTime currentTime = rtc.now();
  for (int i = 0; i < 8; i++){
    times[i] = new DateTime(currentTime.unixtime() + i);
    //Serial.println(times[i]->timestamp());
  }

  for (int i = 0; i < 8; i++){
    if (check_match_fields(*times[i])){
      // The next measurement occurs in i < 8 seconds, wait that long and then begin measurement
      delay(i*1000);
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

void clear_sd(){
  SD.remove("events.txt");
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
 
