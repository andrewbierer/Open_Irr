//-----License------------------------------------------------------------------------
/*
   MIT License

  Copyright (c) 2022 Andrew Bierer

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
//-----Device-------------------------------------------------------------------------
/* Node for Open_Irr soil water tension management system

     Purpose: Manage soil water tension by monitoring watermark 200ss sensors (& temperature) for temperature drift.
     Node sends data to a gateway for storage on sd card module. Node will initiate an irrigation
     event when a defined threshold in soil moisture tension is reached. Water management system will be toggleable and
     thresholds for irrigation, irrigation time, and time between irrigation events programable.

   Much thanks to the ArduinoSoilH2O project <https://github.com/ArduinoSoilH2O>
   which was partial inspiration for the present system.


     Version History:

  2022.09.19 -> First Version of record.
*/
//-----Some future considerations------------------------------------------------------
/*
   1  connect individual temp sensors to groups of WM sensor or number 0-15 like WM sensors...
   2  Adding on to the 4 channels of water thresholds... users should be able to set timing parameters for each irrigation group. Not necessary for the preliminary study taking place in Spring 2022
   3  Parse menu option for radio settings returning registers (transmission strength etc.)
   4  Integration of switch boards so that external relay modules are not needed for irrigation automation
   5  Revise Temperature routine to associate temp sensors with each watermark sensor
   6  Revise star topology to single bus topology for ds18b20 sensors
   7  Revise menu option to include 1 submenu for Watermark settings instead of several menu options (as in Gateway)
   8  Timing of irrigation events without use of delay() as in Gateway
*/
//-----Libraries--------------------------------------------------
#include<SPI.h>                   //SPI communication
#include<SD.h>                    //sd card functionality
#include<SPIFlash.h>              //Flash library for the Moteino Mega flash_CS pin is D23 on this board, Open_Irr is configured with a 4 MBit (0.5 Megabyte) Flash chip.
// Note: if other SPI devices are present, ensure their CS pins are pulled up or set HIGH
#include<Wire.h>                  //I2C functions for rtc
#include<OneWire.h>               //OneWire protocol for DS18B20
#include<DallasTemperature.h>     //To ease DS18b20 use
#include"RTClib.h"                //for RTC functions
#include<EEPROM.h>                //Built in EEPROM routines
#include<DS3232RTC.h>             //Precision (generic) RTC library for DS3232 rtc module, requires v 1.3 - do not update
#include<Streaming.h>             //C++ style output with << operator , maybe not needed with new rtc code?
#include<LowPower.h>              //Low power functionality
#include <avr/sleep.h>            //Sleep Functions
#include "avr/io.h"               //Register set and bit values for Atmel chips
#include "avr/interrupt.h"        //Interrupt functionality 
#include "avr/wdt.h"              //Watchdog timer
#include<CD74HC4067.h>            //For controlling Multiplexor
#include<RH_RF95.h>               //RadioHead library for the RF95 radio transceiver 
#include<RHReliableDatagram.h>    //Additional Radiohead library for ease of transmission, max packet length is 251??
#include<RadioString.h>           //Library to ease sending and receiving of large strings
#include <math.h>                 //Mathmatical functions

//-----Assign Pins-------------------------------------------------
#define LED 15                            //LED pin on Moteino Mega board
#define DS18B20_pin 1                     //Data line for Ds18b20, note all ds18b20 sensors are on a common data line / pullup resistor

#define in1 12                            //The four pins on the Moteino-Mega that can be used as an low-level output 
#define in2 13                            
#define in3 14                           
#define in4 3                             

#define MAX_PACKET_LEN (RH_RF95_MAX_MESSAGE_LEN - 1) //250
#define MAX_STRING_LEN (MAX_PACKET_LEN * (2^8))      //64000 bytes, one character in string/packet = 1 byte

#define WM_path1 26                                  //Pin for path1 of WM reading, to switch polarity
#define WM_path2 27                                  //Pin for path2 of WM reading, to switch polarity
#define WM_analog_read_pin A1                        //Analog Pin (A1) for reading the WM sensors
#define mux_enable 22                                //Pin for EN (enable) of the CD7HC4067 multiplexor
//mux select pins for s0,s1,s2,s3 are respecively digital pins 18,19,20,21
#define pin_mBatt 30                                 //Digital pin to activate battery voltage measurement circuit
#define pin_battV 31                                 //Analog pin to read battery voltage (31 ~ A7)
#define ADC_REF_VOLTAGE 3.3                          //The reference output of the moteino mega board
#define ADC_RESOLUTION  1024                         //The analog digital converter in moteino mega is 10 bit resolution
#define ADC_MAXVALUE  1023                           //1024 possibilities including 0 (0-1023) on 10 bit ADC 

#define SD_CS 28                                     //Chip select (CS) pin on sd card module

#define FLASH_SS 23                                  //Slave select (SS) pin for the flash chip on moteino mega -> uses SPI 

#define RTC_Interrupt 24                             //(A0) for RTC_SQW for interrupts


//-----Declare Global Variables-------------------------------------------

char filename[] = "000_Data.txt";        //sd card file name, 000 to be replaced by IDnum

uint16_t expectedDeviceID = 0xEF30;      //expected flash manufacturer ID of Moteino mega
//////////////////////////////////////////
// flash(SPI_CS, MANUFACTURER_ID)
//
// MANUFACTURER_ID - OPTIONAL, 0x1F44 for adesto(ex atmel) 4mbit flash
//                             0xEF30 for windbond 4mbit flash
//                             0xEF40 for windbond 64mbit flash
//////////////////////////////////////////

char a[4];                                             //char array for itoa function
byte i;                                                //for-loop counter


uint8_t radioID;                                       //Have to have a placeholder for radioID
char Data[RH_RF95_MAX_MESSAGE_LEN];                    //Placeholder to be filled with data to transmit
char Data2[RH_RF95_MAX_MESSAGE_LEN];
uint8_t int_buff[RH_RF95_MAX_MESSAGE_LEN];             //integer type array buffer
uint8_t length_buff;                                   //integer denoting the size of the buffer
uint8_t from_id;                                       //integer specifying the radio id that a transmission is coming from

uint16_t TRANSMIT_TIMEOUT = 2000;                      //Set the minimum retransmit timeout. If sendtoWait is waiting for an ack longer than this time (in milliseconds), it will retransmit the message. Defaults to 200ms.
uint8_t TRANSMIT_POWER = 20;                           //Set transmission power, 0-20,  defaults to 14
float RADIO_FREQUENCY = 915.0;                         //Set frequency of radio module
uint16_t ROUTINE_TIMEOUT = 2000;                       //Set delay at the end of the routine, too low and some packets will be missed, too high and you may also miss packets
uint8_t RETRY_NUM = 3;                                 //times to send packets without receiving ACK

float battV;                                           //battery voltage
float lowBatt = 4.0;                                   //low-battery limit
bool battLow = false;                                  //tracks if battery voltage falls below specified threshold

byte secs;                                             //time and date values, RTClib
byte mins;
byte hrs;
byte dow;
byte days;
byte mnths;
int yrs;
byte alarm_1_Mins;
tmElements_t tm;

boolean isMenuOn = false;
int menuinput;                                         //user input to menu prompt
long menutimeout;                                      //length of time to wait for user input
int indata;                                            //user input data
int input;                                             //for conversion of indata in getdata()
int numincoming;                                       //To indicate how many bytes are coming by user input
int incoming[7];                                       //Arrays are 0 indexed so this is enough room for 1 Binary character (1byte|8bits)
char charInput[200];                                   //for charinput function
char incomingChar[200];

const byte numChars = 32;                              //for reading in character user input

uint32_t last_irr_time_unix_epoch_group1;              //save the time as unix epoch time from RTClib for the groups.
uint32_t last_irr_time_unix_epoch_group2;
uint32_t last_irr_time_unix_epoch_group3;
uint32_t last_irr_time_unix_epoch_group4;

int WM_group1_mean;                                    //Integer to hold WM_group means. Group 1 to 4 means will be attached to a corresponding relay in pin, 1-4
int WM_group2_mean;
int WM_group3_mean;
int WM_group4_mean;
int wm_grace_window = 10;                              //Define the tolerance window (+ or - ,in kpa) for removing an individual sensor from the calculation of the group mean. Only triggers if pdiff from raw mean is >= 20.
bool force_irr = false;

char local_time_irr_update[numChars] {'\0'};           //Empty character array to dump local time stamp from unix time

String data = "";
String header = "";
String WM_string = "";
String irrigation_prompt_string = "";

String temperature_string = "";
float Temp;

//EEPROM structure method for storing and retrieving values from eeprom: relevant for variables defined in the menu
// Note that each EEPROM position can save only one byte of information, i.e. 8-bit numbers 0-255 and leading values (001) ARE NOT INTERPRETABLE
// but more than one position can be accessed using eeprom.put and eeprom.get with a defined structure
// this also enables saving different data types together
//ATMEL EEPROM LIFESPAN ~ 100,000 read/writes
//ATMEL FLASH LIFESPAN ~ 10,000 read/writes

int eeprom_address = 0;

struct eeprom_struct {
  uint8_t nodeID;                         //nodeID for radio networking
  uint8_t IDnum;                          //numeric board identifier 0-255
  uint8_t gatewayID;                      //gatewayID for radio networking
  char projectID[numChars] = {0};         //Project identifier 0-255

  boolean firstTime = true;               //flag for first time for writing to sdcard module

  boolean is_water_manager_on;            //flag to initiate drought stress management routine
  int water_threshold_1;                  //for defining water threshold for each group, 0-255, integer type
  int water_threshold_2;
  int water_threshold_3;
  int water_threshold_4;

  long min_time_btwn_irr;                //for defining minimum time between irrigation events, seconds
  long irr_period;                       //for defining the duration of an irrigation event, seconds
  boolean include_resistance;            //for specifying if resistance is to be included in written data string. true=1 false=0

  boolean calibration_resistor_present;  //Is a calibration resistor present?
  int cal_resistor_loc;                  //Channel location of the calibration resistor between the Multiplexors
  float cal_resistor_val;                //Value of the calibration resistor installed
  int fixed_resistor_val;                //The value of the fixed resistor attached in series to the sensor and ground(voltage divider circuit), unchanged schematic is 10K ohms
  int num_WM;                            //Define the number of watermark sensors present

  int WM_group1[16] {0};                 //Array to hold mux channel location of WM sensors to average.
  int WM_group2[16] {0};                 //can declare the memory space here for the averages.
  int WM_group3[16] {0};
  int WM_group4[16] {0};

  int num_ds18b20;                       //Define the number of ds18b20 temperature sensors present
  uint8_t ds18b20_sensor0[8];            //Define locations to store ds18b20_sensor_addresses
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

  int ALARM_1_Interval = 1;                //Set the interval for alarm 1 (wake and run routine), minutes

  int n_channels_wm_group1;                //Integer to hold # of channels utilized in each WM_group mean
  int n_channels_wm_group2;
  int n_channels_wm_group3;
  int n_channels_wm_group4;

  bool demo_mode;                          //controls trouble shooting demo loop or the real loop

  bool toggle_radio;                       //can turn off radio transmission when there is no gateway utilized
};

eeprom_struct eeprom_object = {};          //Declare an object with the eeprom_struct structure, access objects as eeprom_object."element of struct without quotes"


//-----Initialize-----------------------------------------------------
RTC_DS3231 rtc;

CD74HC4067 mux_1(18, 19, 20, 21);             //connect s0,s1,s2,s3 select pins on mux to specified digital pins
CD74HC4067 mux_2(18, 19, 20, 21);             //The same pins are connected to mux 2 for reading Watermark200ss

OneWire oneWire(DS18B20_pin);                 //The Data pin for the DS18b20 temp sensors
DallasTemperature sensors(&oneWire);          //Pass OneWire reference to Dallas Temperature

DeviceAddress DS18B20_Address;                //Array to hold sensor addresses

RH_RF95 driver(4, 2);
RadioString manager(driver, radioID);         //Set up the radio manager, with RadioString library

File myfile;                                  //make sd card file object

//FlashTools flash;                           //initialize flash memory

//_________________________________________________________________________________________________________________________________
void setup() {
  Serial.begin(9600);
  delay(100);
  Wire.begin();                               //enable I2C bus for rtc
  delay(100);
  SPI.begin();
  delay(300);

  //-----Pin settings-----

  pinMode(SD_CS, OUTPUT);                    //CS pin for sd card

  pinMode(in1, OUTPUT);                      //declare relay pins as outputs
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);                    //Four channel relay we want normally lOW and activated when HIGH. Bridge the COM & HIGH pins on the relay with provided jumper. Provide Vcc & GND from 5v wall plug. connect microcontroller pins to relay switches. Connect common ground between microcontroller and 5v wall plug.
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pinMode(WM_path1, OUTPUT);                  //WM Sensor Vs or GND
  pinMode(mux_enable, OUTPUT);                //enable or disable the multiplexor, the EN pin on multiplexor
  pinMode(WM_path2, OUTPUT);                  //WM Sensor Vs or GND
  digitalWrite(mux_enable, HIGH);             //Disable the mux on setup

  pinMode(pin_mBatt, OUTPUT);
  digitalWrite(pin_mBatt, LOW);               //Leave the battery voltage circuit open to avoid battery drain

  if (! SD.begin(SD_CS)) {
    Serial.println(F("SD card not present or card failure."));
  }

  //Flash chip routine here instead of sdcard?
  /*
      if (! SD.begin(SD_CS)) {
    Serial.println(F("Attempting to save data to Flash Memory instead..."));
    if (! flash.init(FLASH_SS, 0xEF40)) {
      Serial.println(F("Flash Memory initialization FAILED."));
      Serial.println(F("Erase Flash memory."));
      flash.chipErase();
    } else {
      Serial.print(F("Flash Memory initialization OK."));
    }
    Serial.println(F("2022_05_19 -> This function is not complete at this time."))
    }
  */

  //-----

  battV = calcbattV();                            // Get board battery level on startup

  //----- check chip EEPROM for stored data and place into "eeprom_object" with the structure of "eeprom_struct" declared earlier

  EEPROM.get(eeprom_address, eeprom_object);                              //eeprom_address may be redundant if only writing one eeprom object (i.e. it would always begin at position 0)
  int datasize_group1 = sizeof(eeprom_object.WM_group1) / sizeof(int);    //Hold the element length of the WM_group arrays, needs to be after eeprom_objects are read.
  int datasize_group2 = sizeof(eeprom_object.WM_group2) / sizeof(int);
  int datasize_group3 = sizeof(eeprom_object.WM_group3) / sizeof(int);
  int datasize_group4 = sizeof(eeprom_object.WM_group4) / sizeof(int);


  //-----Set initial radio settings
  delay(10);
  driver.setFrequency(915.0);
  delay(10);
  driver.setTxPower(20, false);
  delay(10);
  manager.setThisAddress(eeprom_object.nodeID);                           //Set the current address for radio communication in the manager
  delay(10);
  manager.setTimeout(2000);                                               //set timeout period where if an ACK not recieved it will retransmit message Default is 200ms, this will vary based on transmission packet length
  delay(1000);


  //-----RTC setup-----
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC, routine hang up"));
    while (1);
  }

  //Power Saving
  wdt_disable();                                  // turn off watchdog timer From ArduinoSoilH2O
  rtc.disable32K();                               // Turn off 32kHz output
  pinMode(RTC_Interrupt, INPUT_PULLUP);           // Making it so, that the alarm will trigger an interrupt

  //Schedule an alarm
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 0);   //Alarm_1  alarmtype,seconds,minutes,hours,daydate
  RTC.setAlarm(ALM2_MATCH_MINUTES, 0, 0, 0, 0);   //Alarm_2  alarmtype,seconds,minutes,hours,daydate
  RTC.alarm(ALARM_1);                             //turn off alarm 1
  RTC.alarm(ALARM_2);                             //turn off alarm 2
  RTC.alarmInterrupt(ALARM_1, true);              //turn on alarm 1 interrupt
  RTC.squareWave(SQWAVE_NONE);                    //Use SQW pin as interrupt generator, no output frequency
  readRTC();
  RTC.alarm(ALARM_1);
  Set_ALARM_1_Interval();

  //Second alarm here for timesync from gateway or other future consideration.
  //-----

  Serial.println("Initialization Completed");
  menu();
}
//______________________________________________________________________________________________________________________________________________________________________________________________

void loop() {
  //---Real loop----------------------------

  if (eeprom_object.demo_mode == false) {       //Check for troubleshooting/demo mode in the eeprom settings
    Low_Power_Sleep();                          //Enter Power saving routine & wait for alarm

    if (RTC.alarm(ALARM_1)) {                   //When alarm 1 goes off...
      Serial.println(F("Waking from sleep..."));
      readRTC();                                //read RTC
      delay(50);
      Temp = getTemp();                         //Read DS18B20 temperature sensors
      delay(50);
      read_watermark();                         //Read the WaterMark Sensors
      delay(50);
      water_manager();                          //Determine if an irrigation event is appropriate, perform if so
      delay(50);
      compile();                                //Format data from subroutines into one string for transmission
      delay(50);

      //Radio Transmission
      if (!manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.nodeID)) {
        Serial.println(F("Radio Failed"));
      }
      if (manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.nodeID)) {
        Serial.println(F("Radio initilization successful."));
      }
      if (eeprom_object.toggle_radio) {         //If Radio transmission is ENABLED (true)..
        manager.Send_String(data, eeprom_object.nodeID, eeprom_object.gatewayID); //RadioString transmit
      }

      delay(50);
      saveData();                               //Write the data string to the sdcard
      delay(50);
      check_menu();                             //Check to see if user is requesting access to menu ("menu")
      delay(50);
      clearBuffer();                            //Clear the buffer strings used throughout the sketch
      delay(50);
      Set_ALARM_1_Interval();                   //Re-set the alarm interval
      delay(50);                                //Return to top of loop i.e. Go back to sleep...
    }
  } else {
    Serial.println(F("Troubleshooting and Demo loop..."));
    //---Demo loop---------------------------
    readRTC();                      //Read the RTC
    delay(50);
    Temp = getTemp();               //Read DS18B20 temperature sensors
    delay(50);
    read_watermark();               //Read the WaterMark Sensors
    delay(50);
    water_manager();                //Determine if an irrigation event is appropriate, perform if so
    delay(50);
    compile();                      //Format data from subroutines into one string for transmission
    delay(50);

    //Radio Transmission
    if (!manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.nodeID)) {
      Serial.println(F("Radio Failed"));
    }
    if (manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.nodeID)) {
      Serial.println(F("Radio initilization successful."));
    }
    if (eeprom_object.toggle_radio) {         //If Radio transmission is ENABLED (true)..
      manager.Send_String(data, eeprom_object.nodeID, eeprom_object.gatewayID); //RadioString transmit
    }

    delay(50);
    //saveData();                             //Write the data string to the sdcard
    delay(50);
    check_menu();                             //Check to see if user is requesting access to menu ("menu")
    delay(50);
    clearBuffer();                            //Clear the buffer strings used throughout the sketch
    delay(50);
    //wake_interrupt();                       //Reset next alarm interrupt to wake after interval has been exceeded
    delay(50);
    //Low_Power_Sleep();                      //Return to sleep
    Serial.println();                         //Spacer for next loop

  }
}

//___________________________________________________________________________________________________________________________________
//-----Other Defined Functions------
//___________________________________________________________________________________________________________________________________

//-----Water manager------------------------------------------------
void water_manager() {

  if (eeprom_object.is_water_manager_on == true) {
    Serial.println(F("Water Manager ON"));
  } else {
    Serial.println(F("Water Manager OFF"));
  }

  //Calculate the means for the specified groups
  /*
     Troubleshooting consideration
       Serial.print(F("Printing group1 from eeprom_object: "));
    for (int i; i < (sizeof(eeprom_object.WM_group1) / sizeof(int)) ; i++) {
      int q = eeprom_object.WM_group1[i];
      Serial.print(q);
    }
    Serial.println();

  */


  WM_group1_mean = WM_group_means(eeprom_object.WM_group1, (sizeof(eeprom_object.WM_group1) / sizeof(int)), 1);
  Serial.print(F("WM Group 1 mean:  "));
  Serial.println(WM_group1_mean);
  Serial.print(F("eeprom group 1 mean:  "));
  for (int r = 0; r < 16; r++) {
    Serial.print(eeprom_object.WM_group1[r]);
    Serial.print(F(" "));
  }
  Serial.println();

  WM_group2_mean = WM_group_means(eeprom_object.WM_group2, (sizeof(eeprom_object.WM_group2) / sizeof(int)), 2);
  Serial.print(F("WM Group 2 mean:  "));
  Serial.println(WM_group2_mean);
  Serial.print(F("eeprom group 2 mean:  "));
  for (int r = 0; r < 16; r++) {
    Serial.print(eeprom_object.WM_group2[r]);
    Serial.print(F(" "));
  }
  Serial.println();

  WM_group3_mean = WM_group_means(eeprom_object.WM_group3, (sizeof(eeprom_object.WM_group3) / sizeof(int)), 3);
  Serial.print(F("WM Group 3 mean:  "));
  Serial.println(WM_group3_mean);
  Serial.print(F("eeprom group 3 mean:  "));
  for (int r = 0; r < 16; r++) {
    Serial.print(eeprom_object.WM_group3[r]);
    Serial.print(F(" "));
  }
  Serial.println();

  WM_group4_mean = WM_group_means(eeprom_object.WM_group4, (sizeof(eeprom_object.WM_group4) / sizeof(int)), 4);
  Serial.print(F("WM Group 4 mean:  "));
  Serial.println(WM_group4_mean);
  Serial.print(F("eeprom group 4 mean:  "));

  for (int r = 0; r < 16; r++) {
    Serial.print(eeprom_object.WM_group4[r]);
    Serial.print(F(" "));
  }
  Serial.println();


  if (eeprom_object.is_water_manager_on == true) {                                    //if water manager is active...
    //run the WM_irrigation_prompt routine
    //uint32_t WM_irrigation_prompt(int WM_group_num, int WM_group_mean, int WM_group_water_threshold, uint32_t last_irr_time_for_group) -->> format for new 4 threshold groups

    WM_irrigation_prompt(1, WM_group1_mean, eeprom_object.water_threshold_1, last_irr_time_unix_epoch_group1);
    WM_irrigation_prompt(2, WM_group2_mean, eeprom_object.water_threshold_2, last_irr_time_unix_epoch_group2);
    WM_irrigation_prompt(3, WM_group3_mean, eeprom_object.water_threshold_3, last_irr_time_unix_epoch_group3);
    WM_irrigation_prompt(4, WM_group4_mean, eeprom_object.water_threshold_4, last_irr_time_unix_epoch_group4);
  } else {
    Serial.println(F("Water Manager Disabled."));
  }
}

//-----Read Watermark Sensors---------------------------------------
void read_watermark() {
  //Declare vars
  const int num_Iter = 3;                   //Define number of iterations each is actually two reads of the sensor (both directions)
  //const long TempC = 24;                  //Default when no temperature sensor is attached, replace with sensor value when integrated
  long TempC = Temp;                        //Pull average temperature sensor from ds18b20 for time being until each temperature sensor can be attributed to a single or group of watermark sensors
  const long open_resistance = 35000;       //For throwing fault 255, default was 35000
  const long short_resistance = 200;        //For throwing fault 240, default was 200, modified was 45
  const long short_CB = 240;                //The Fault Code for WM sensor terminal short in wiring
  const long open_CB = 255;                 //The Fault Code for WM sensor open circuit or missing sensor / sensor not present
  float Calib_Resistance;                   //Holder for any calibration resistor present
  float SenV10K = 0;                        //For calibration resistor

  int WM_case;                              //for describing what happens to the watermark readings based on the resistance
  int WM1_CB = 0;                           //Holder for WM sensor value in CB/kPa, direction 1
  int WM2_CB = 0;                           //Holder for WM sensor value in CB/kPa, direction 2

  float SenVWM1 = 0;                        //VWM is the "stand off voltage"?, taken here to mean the average voltage which was recorded in direction 1.
  float SenVWM2 = 0;                        //VWM is the "stand off voltage"?, taken here to mean the average voltage which was recorded in direction 2.

  float ARead_path1 = 0;                    //The raw analog read value obtained in direction 1.
  float ARead_path2 = 0;                    //The raw analog read value obtained in direction 2.

  float SupplyV = 3.3;                      //If using a different logic board (3.3V vs. 5V etc) moteino mega is 3.3v

  //Code-----
  ARead_path1 = 0;                          //Reset these values on the loop
  ARead_path2 = 0;

  if (eeprom_object.calibration_resistor_present == true) {   //Read the calibration resistor IF included. There is no polarity swap needed here
    for (int j = 0; j < num_Iter; j++) {                      //the num_Iter controls the number of successive read loops that is averaged.
      digitalWrite(mux_enable, LOW);                          //enable the MUX (The EN Pin on CD74HC4067)
      delay(10);
      mux_1.channel(eeprom_object.cal_resistor_loc);          //address the MUX where calibration resistor is installed
      delay(10);
      digitalWrite(WM_path2, LOW);                            //Set pin as gnd
      digitalWrite(WM_path1, HIGH);                           //Set pin as Vs
      delay(0.09);                                            //wait 90 micro seconds and take sensor read
      ARead_path1 += analogRead(WM_analog_read_pin);          //read the analog pin the multiplexor is connected to (Moteino Mega 10bit ADC)
      digitalWrite(WM_path1, LOW);                            //set the excitation voltage to OFF/LOW
      delay(100);                                             //0.1 second delay before moving to next channel or switching MUX
    }
    digitalWrite(mux_enable, HIGH);                           //Disable mulitplexor
    SenV10K = ((ARead_path1 / 1024) * SupplyV) / (num_Iter);  //get the average of the number of readings and convert to volts, 1024 here for 10bit ADC
    Calib_Resistance = eeprom_object.cal_resistor_val / ((eeprom_object.fixed_resistor_val * (SupplyV - SenV10K) / SenV10K));    //generate a calibration factor
    Serial.print(F("SenV10K:  "));
    Serial.println(SenV10K);
    Serial.print(F("eeprom_object.cal_resistor_loc: "));
    Serial.println(eeprom_object.cal_resistor_loc);
    Serial.print(F("Calibration Factor from fixed Resistor: "));
    Serial.println(Calib_Resistance);
    Serial.print(F("Raw Sum of Analog Reads:  "));
    Serial.println(ARead_path1);
    delay(100);                                               //0.1 second wait before moving to next channel or switching MUX
  }
  else {
    Serial.println(F("Calibration Factor defaulting to 1. Insert and specify a fixed resistor for calculating a calibration factor."));
    Calib_Resistance = 1;                                     //if not actually calculated, assume 1
    digitalWrite(mux_enable, HIGH);                           //Disable mux pair
  }
  for (int i = 0; i < eeprom_object.num_WM; i++) {            //Assuming num_WM starts at mux channel 0 and increments, i.e. if num_WM = 4, i=0,1,2,3
    ARead_path1 = 0;                                          //restore values to 0 for successive WM readings
    ARead_path2 = 0;

    for (int n = 0; n < num_Iter;  n++) {                     //take num_Iter readings from mux channel i
      digitalWrite(mux_enable, LOW);                          // enable the MUX
      mux_1.channel(i);                                       //Select mux channel from i in for loop referenceing num_WM above
      delay(10);
      digitalWrite(WM_path1, HIGH);                           //set signal pin of mux 1 (pin7) high
      delay(0.09);
      ARead_path1 += analogRead(WM_analog_read_pin);          //read the signal from this direction and add to running total
      digitalWrite(WM_path1, LOW);                            //set pin low

      //Polarity SWAP
      delay(100);

      digitalWrite(WM_path2, HIGH);                           //set pin 11 high this time
      delay(0.09);
      ARead_path2 += analogRead(WM_analog_read_pin);          //read the signal from the opposite direction and add to running total
      digitalWrite(WM_path2, LOW);                            //set pin low

      /* Troubleshooting consideration
            //Serial.print(F("ARead_path1:  "));
            //Serial.println(ARead_path1);
            //Serial.print(F("ARead_path2:  "));
            //Serial.println(ARead_path2);
      */
    }
    SenVWM1 = ((ARead_path1 / 1024) * SupplyV) / (num_Iter); //get the average of the readings in the first direction and convert to volts | 1024 because 10bit adc
    SenVWM2 = ((ARead_path2 / 1024) * SupplyV) / (num_Iter); //get the average of the readings in the second direction and convert to volts
    /* Troubleshooting consideration
        //Serial.print(F("SenVWM1:  "));
        //Serial.println(SenVWM1);
        //Serial.print(F("SenVWM2:  "));
        //Serial.println(SenVWM2);
    */
    float WM1_ResistanceA = (eeprom_object.fixed_resistor_val * (SupplyV - SenVWM1) / SenVWM1); //do the voltage divider math, using the the known resistor in the circuit
    float WM1_ResistanceB = eeprom_object.fixed_resistor_val * SenVWM2 / (SupplyV - SenVWM2);   //voltage divider math, opposite direction
    float WM1_Resistance = ((WM1_ResistanceA + WM1_ResistanceB) / 2) * Calib_Resistance;        //Average of the directions and apply calibration factor
    digitalWrite(mux_enable, HIGH);                                                             //Disable mux pair
    delay(100);

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
      if (WM1_Resistance > 300.00)  {
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
        WM1_CB = -2.246 - 5.239 * (WM1_Resistance / 1000.00) * (1 + .018 * (TempC - 24.00)) - .06756 * (WM1_Resistance / 1000.00) * (WM1_Resistance / 1000.00) * ((1.00 + 0.018 * (TempC - 24.00)) * (1.00 + 0.018 * (TempC - 24.00)));
        break;
      case 2:
        WM1_CB = (-3.213 * (WM1_Resistance / 1000.00) - 4.093) / (1 - 0.009733 * (WM1_Resistance / 1000.00) - 0.01205 * (TempC)) ;
        break;
      case 3:
        WM1_CB = ((WM1_Resistance / 1000.00) * 23.156 - 12.736) * (1.00 + 0.018 * (TempC - 24.00));
        break;
      case 4:
        WM1_CB = 0.00;
        break;
      case 5:
        WM1_CB = short_CB;                                              //240 is a fault code for sensor terminal short
        break;
      case 6:
        WM1_CB = open_CB;                                               //255 is a fault code for open circuit or sensor not present, also observed in new dry sensors
        break;
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


//-----Read DS18B20 Temperature sensors-------------------------------------------            //do not plug in backwards or board will heat!!!
float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius, not using the actual return value but updating the temperature_string

  sensors.begin();                                                                            //initiate the library
  delay(100);
  sensors.setResolution(DS18B20_Address, 9);                                                  //Specify the resolution of the device, in bits (9 to 12). Higher resolution takes more time, 12bit can be up to 750ms

  delay(100);                                                                                 //The second call to sensors.begin(); below did not work for me 04/04/22

  Serial.println(F("Requesting Temperatures..."));                                            //Print temperatures by address, each sensor address is defined before setup().
  sensors.requestTemperatures();

  delay(1000);

  Serial.print(F("Locating devices..."));                      // locate devices on the bus with getDeviceCount, there are known errors with getDeviceCount returning 0....
  Serial.print(F("Found "));
  int num_ds18b20 = (sensors.getDeviceCount());                //store # of sensors detected, there are known errors with getDeviceCount returning 0....
  delay(100);
  Serial.print(num_ds18b20);
  Serial.println(F(" devices."));

  Serial.print(F("Power = Parasite:  "));
  Serial.println(sensors.readPowerSupply(DS18B20_Address));   //report parasite power condition (should be 0 with our wiring.)
  delay(100);
  Serial.print(("Parasite power is: "));                      // report parasite power condition
  if (sensors.isParasitePowerMode()) {
    Serial.println(F("ON"));
  } else {
    Serial.println(F("OFF"));
  }

  delay(10);

  //Reference code to store the addresses in eeprom. This seems by-far the best solution through a menu() window to identify sensors....
  //< https: //forum.arduino.cc/t/storage-of-ds18b20-address-in-eeprom/934477/19>
  //< https: //www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html>
  Serial.println(F("Begin test of configuring string from array"));
  Serial.println(eeprom_object.num_ds18b20);
  float sum_temp = 0;                                         //for mean
  float mean_temp = 0;
  float single_temp = 0;                                      //for error detection
  float rtcTemp = 0;

  //The top level array (an array of arrays here) must be a pointer.
  uint8_t *ds18b20_array[] {eeprom_object.ds18b20_sensor0, eeprom_object.ds18b20_sensor1, eeprom_object.ds18b20_sensor2, eeprom_object.ds18b20_sensor3, eeprom_object.ds18b20_sensor4, eeprom_object.ds18b20_sensor5, eeprom_object.ds18b20_sensor6, eeprom_object.ds18b20_sensor7, eeprom_object.ds18b20_sensor8, eeprom_object.ds18b20_sensor9, eeprom_object.ds18b20_sensor10, eeprom_object.ds18b20_sensor11, eeprom_object.ds18b20_sensor12, eeprom_object.ds18b20_sensor13, eeprom_object.ds18b20_sensor14, eeprom_object.ds18b20_sensor15};

  rtcTemp = (RTC.temperature() / 4.0);                        //from the ds3232 library

  //Loop through...
  for (int i = 0; i < (eeprom_object.num_ds18b20); i++) {
    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.print(F(" Address: "));
    printAddress(ds18b20_array[i]);
    Serial.print(F(" Temperature C: "));
    single_temp = sensors.getTempC(ds18b20_array[i]);
    delay(10);
    Serial.print(single_temp);
    Serial.println();

    if (single_temp == -127.00 || single_temp == -98.00 || single_temp == 85.00) {              //Check for known fault codes
      Serial.print(F("Error on temperature sensor:  "));
      Serial.print(i);
      Serial.print(F("  With Value: "));
      Serial.println(single_temp);
      Serial.println(F("Replacing this sensors temperature with that of the RTC."));
      single_temp = rtcTemp;
    }

    if (((abs(rtcTemp - single_temp) / ((rtcTemp + single_temp) / 2)) > 20) || ((single_temp - rtcTemp) >= 10) || ((single_temp - rtcTemp) <= -10)) {         //if Percent difference between the sensor temp and the RTC temp is higher than 20%, use the rtcTemp from the RTC. 2022/05/23 Added + or - 10 from rtcTemp as an additional criteria.
      Serial.print(F("Percent Difference threshold (20%) from RTC reference temp exceeded on Sensor "));
      Serial.print(i);
      Serial.print(F(" With Value: "));
      Serial.println(single_temp);
      Serial.println(F("Replacing this sensors temperature with that of the RTC."));
      single_temp = rtcTemp;
    }

    sum_temp += single_temp;
    delay(20);
    mean_temp = (sum_temp / eeprom_object.num_ds18b20);

    temperature_string += 'T';
    temperature_string += (i);
    temperature_string += ',';
    temperature_string += single_temp;
    delay(10);
    temperature_string += ',';
  }
  Serial.print(F("Sum of temperature sensors: "));
  Serial.println(sum_temp);
  Serial.print(F("Number of temperature sensors:  "));
  Serial.println(eeprom_object.num_ds18b20);
  Serial.print(F("Mean temperature:  "));
  Serial.println(mean_temp);
  return mean_temp;

  Serial.println(F("End test of configuring string from array"));
}

//-----Compile Data-------------------------------------------------
void compile() {

  battV = calcbattV();
  Serial.print(F("Battery Pack Voltage:  "));
  Serial.println(battV);

  data = "";
  header = "";
  delay(20);

  header += '[';                                                                //Add in String start indicator '[' and spacers "~~~" for packet # as in RadioString library
  header += '~';
  header += '~';
  header += '~';

  header += eeprom_object.IDnum;
  header += ',';

  if (battV <= lowBatt) {
    header += "Low Battery Voltage: ";
    battLow = true;
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

  data =  header + WM_string + irrigation_prompt_string + temperature_string;                   //Add in responses from sensors

  delay(50);

  data.remove(data.length() - 1, 1);                                                            //remove last comma
  data.concat(']');                                                                             //Add in an indicator for the end of the data string
  Serial.println(data);
  delay(20);
}

//-----Store data on micro sd card----------------------------------
void saveData() {

  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD card not present or card failure."));
  }

  delay(50);

  myfile = SD.open(filename, FILE_WRITE);                                     //Open file for writing
  delay(10);

  if (myfile) {                                                               //If myfile is available...
    if (eeprom_object.firstTime == true) {                                    //Print this header if first time writing to file
      myfile.println();
      myfile.println(F("Data values are stored as follows:"));
      myfile.println(F("Node ID, Node battery voltage (V), DateTime"));
      myfile.println(F("Following DateTime the mux channel position and sensor value (CB/kpa) for all watermark sensors installed are printed (Channel, Value, Channel, Value)"));
      myfile.println(F("IF user specified for reporting of raw resistance values, format will be (Channel, kPa, resistance, Channel, kPa, Resistance), etc."));
      myfile.println(F("Following the raw watermark sensor data, the mean for specifed groups of channels are reported as (G(1-4), Mean)"));
      myfile.println(F("After watermark group means, DS18b20 temperature sensor data is reported as (T(# of sensor), Value (C))"));
      myfile.println();

      eeprom_object.firstTime = false;                                        //Only include this header when first writing to file
      eeprom_address = 0;                                                     //clear eeprom_address
      EEPROM.put(eeprom_address, eeprom_object);                              //Store the new firsttime flag to eeprom
      eeprom_address = 0;
    }

    data.remove(data.length() - 1, 1);                                        //Remove the character that indicated the end of a transmission "]"
    data.remove(0, 4);                                                        //Remove the Transmission information "[~~~". Zero indexed

    myfile.println(data);                                                     //Save data string to file
    delay(200);
    myfile.close();                                                           //Must close file
    delay(200);
    Serial.println(F("Data Written Successfully."));
  } else {
    Serial.println(F("Error opening file."));                                 //Error if myfile is not available
  }

}

//-----Sleep Functions----------------------------------------------

//Power saving functions referenced from ArduinoSoilH2O <https://github.com/ArduinoSoilH2O>
void setRTCInterrupt() {
  sei();                     //turn on Global Interrupt Enable
  PCMSK0 |= (1 << PCINT0);   //set A0/D24/Chippin 37 as PCINT
  PCICR |= (1 << PCIE0);     //enable interrupts on vector 0
}
void clearRTCInterrupt() {
  PCICR &= (1 << PCIE0);
}
ISR(PCINT0_vect) {
  sleep_disable();
  clearRTCInterrupt();
}

void Low_Power_Sleep() {
  Serial.println(F("Going to sleep..."));
  Serial.print(F("Current Measurement Interval: "));
  Serial.print(eeprom_object.ALARM_1_Interval);
  Serial.print(F("  Minutes. Next measurement to occur at:  "));
  DateTime now = rtc.now();                                                       //needed to get unix time in next line
  uint32_t current_unix_epoch_time = now.unixtime();                              //get current unix epoch time
  local_time(current_unix_epoch_time + (eeprom_object.ALARM_1_Interval * 60));
  Serial.print(local_time_irr_update);
  Serial.println();

  Serial.end();                                                                   //disable serial communication
  digitalWrite(15, LOW);                                                          //Turn off the LED pin (15 on moteino mega)
  driver.sleep();                                                                 //Put radio to sleep
  analogComp_off();                                                               //turn off analog comparator
  ADC_off();                                                                      //turn off ADC
  JTAG_off();                                                                     //disable On-Chip Debug system
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                                            //Power down completely on sleep
  sleep_enable();
  setRTCInterrupt();
  sleep_mode();                                                                   //Puts MEGA to sleep

  //This happens after wake up Interrupt-----
  ADC_on();
  Serial.begin(9600);
  SPI.begin();
}

//------------- Power Savers ---------------------
//Power saving functions referenced from ArduinoSoilH2O <https://github.com/ArduinoSoilH2O>
void RTC_osc_off() {                                              //Turn RTC 32kHz oscillator off to save power:
  uint8_t RTC_SR;
  uint8_t res;
  RTC_SR = RTC.readRTC(0x0F);                                     //First read status register
  res = RTC.writeRTC(0x0F, RTC_SR & ~(1 << 3));                   //Now re-write status register bits (bit 3 needs to be set to 0 to disable 32kHz output)
  //res = RTC.writeRTC(0x0F, RTC_SR | (1 << 3));                  //Just as a test, this ENABLES the output (which should pull more current)
}

void analogComp_off() {                                           //Turn off analog comparator
  ACSR &= ~(1 << 7);
}

void analogComp_on() {                                            //Turn on analog comparator
  ACSR |= (1 << 7);
}

void ADC_off() {                                                  //Turn off ADC  --  this saves about 113 uA
  ADCSRA &= ~(1 << 7);
}

void ADC_on() {                                                   //Turn on ADC
  ADCSRA |= (1 << 7);
}

void JTAG_off() {                                                 //We don't ever need to turn it on for this application  //Must be executed twice within 4 clock cycles to disable JTAG!
  cli();
  MCUCR |= (1 << 7);
  MCUCR |= (1 << 7);
  sei();
}



//------Check Time-----------------------------------------------------
void readRTC() {
  DateTime now = rtc.now();
  secs = now.second();
  mins = now.minute();
  hrs = now.hour();
  days = now.day();
  mnths = now.month();
  yrs = now.year();
}

//-----Menu Routine----------------------------------------------------
void menu() {

  if (Serial.available() > 0) {
    Serial.read();                                //clear serial input buffer
  }

  itoa(eeprom_object.IDnum, a, 10);               //convert IDnum to character array

  if (eeprom_object.IDnum < 10) {                 // for naming filename
    filename[0] = '0';                            // put into filename[] array
    filename[1] = '0';
    filename[2] = a[0];
  } else if (eeprom_object.IDnum < 100) {
    filename[0] = '0';                            // put into filename[] array
    filename[1] = a[0];
    filename[2] = a[1];
  } else {
    filename[0] = a[0];                           // put into filename[] array
    filename[1] = a[1];
    filename[2] = a[2];
  }
  //print out board information-----
  Serial.println();
  Serial.println(F("Arduino Drought Stress Manager"));
  Serial.print(F("Project ID:  "));
  Serial.println(eeprom_object.projectID);
  Serial.print(F("Board ID: "));
  Serial.println(eeprom_object.IDnum);
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
  if (mins < 10)
  {
    Serial.print('0');
  }
  Serial.print(mins);
  Serial.print(':');
  if (secs < 10)
  {
    Serial.print('0');
  }
  Serial.println(secs);

  Serial.print(F("Current Battery Voltage:  "));
  Serial.print(battV);
  Serial.println(F("V"));

  Serial.print(F("Current Measurement Interval: "));
  Serial.print(eeprom_object.ALARM_1_Interval);
  Serial.print(F("  Minutes. Next measurement to occur at:  "));
  DateTime now = rtc.now();                                                       //needed to get unix time in next line
  uint32_t current_unix_epoch_time = now.unixtime();                              //get current unix epoch time
  local_time(current_unix_epoch_time + (eeprom_object.ALARM_1_Interval * 60));
  Serial.print(local_time_irr_update);
  Serial.println();

  Serial.print(F("Number of WaterMark sensors currently installed:  "));
  Serial.println(eeprom_object.num_WM);
  Serial.print(F("Include Resistance Values in output:  "));
  Serial.println(eeprom_object.include_resistance);
  Serial.println(F("Current Sensor Grouping for Means:  "));
  Serial.print(F("Group1: "));
  for (int i = 0; i < 16; i++) {
    if (eeprom_object.WM_group2[i] >= 0 ) {
      Serial.print(eeprom_object.WM_group1[i]);
      Serial.print(F("  "));
    }
  }
  Serial.println();
  Serial.print(F("Group2: "));
  for (int i = 0; i < 16; i++) {
    if (eeprom_object.WM_group2[i] >= 0 ) {
      Serial.print(eeprom_object.WM_group2[i]);
      Serial.print(F("  "));
    }
  }
  Serial.println();
  Serial.print(F("Group3: "));
  for (int i = 0; i < 16; i++) {
    if (eeprom_object.WM_group2[i] >= 0 ) {
      Serial.print(eeprom_object.WM_group3[i]);
      Serial.print(F("  "));
    }
  }
  Serial.println();
  Serial.print(F("Group4: "));
  for (int i = 0; i < 16; i++) {
    if (eeprom_object.WM_group2[i] >= 0 ) {
      Serial.print(eeprom_object.WM_group4[i]);
      Serial.print(F("  "));
    }
  }
  Serial.println();

  Serial.print(F("Number of temperature sensors connected:  "));
  Serial.println(eeprom_object.num_ds18b20);

  Serial.print(F("Water Manager Status: "));
  Serial.println(eeprom_object.is_water_manager_on);
  Serial.print(F("Water threshold levels:   "));
  Serial.println();
  Serial.print(F("Group 1 threshold:  "));
  Serial.println(eeprom_object.water_threshold_1);
  Serial.print(F("Group 2 threshold:  "));
  Serial.println(eeprom_object.water_threshold_2);
  Serial.print(F("Group 3 threshold:  "));
  Serial.println(eeprom_object.water_threshold_3);
  Serial.print(F("Group 4 threshold:  "));
  Serial.println(eeprom_object.water_threshold_4);

  Serial.print(F("Minimum time between irrigation events: "));
  Serial.println(eeprom_object.min_time_btwn_irr);
  Serial.print(F("Duration of irrigation event: "));
  Serial.println(eeprom_object.irr_period);

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

  Serial.println(F("Menu Actions:"));                                                 // Define menu Actions--------------------------------------------
  Serial.println(F("   c  <--  Set clock"));                                          // "99" Set RTC clock
  Serial.println(F("   i  <--  Set ID numbers"));                                     // "105" set IDnum, nodeID, gatewayID
  Serial.println(F("   a  <--  Set Alarm (measurement) Interval"));                   // "97" Set the alarm intervals for the RTC
  Serial.println(F("   g  <--  Toggle Radio Transmissions (ON/OFF)"));                // "103" Enable or disable LoRa Radio transmisison
  Serial.println(F("   h  <--  Toggle Troubleshooting (continuous) mode"));           // "104" Enter Troubleshooting continuous loop
  Serial.println(F("   b  <--  Identify connected ds18b20 sensors"));                 // "98" menu routine for iteratively connecting temperature sensors and saving them to eeprom
  Serial.println(F("   t  <--  Test measurements"));                                  // "116" take test measurements from sensors -> not currently used
  Serial.println(F("   s  <--  Switch Water Manager"));                               // "115" Toggle/Switch Water Manager routine
  Serial.println(F("   w  <--  Define water threshold values and times"));            // "119" Define threshold to trigger an event
  Serial.println(F("   n  <--  Specify number of Watermark sensors and Mean groups"));// "110" Set number of WM sensors installed & specify groups based on channel position of multiplexor? Could also write funciton to subtract 1 from numbers specified so that actual terminal numbers can be used.
  Serial.println(F("   o  <--  Specification of resistors for WM circuit"));          // "111" Define resistor information
  Serial.println(F("   p  <--  Set output HIGH on pinouts 1 to 4"));                  // "112" For priming/testing irrigation equipment
  //  Serial.println(F("   r  <--  Download range of data"));                         // "114" set beginning date to download, not functional
  Serial.println(F("   f  <--  Display microSD card information"));                   // "102" Get sdcard information, non functional
  Serial.println(F("   d  <--  Download all data"));                                  // "100" get all data to serial port
  Serial.println(F("   e  <--  Erase all data"));                                     // "101" erase all data
  //  Serial.println(F("   m  <--  Repeat menu"));                                    // "109" repeat menu, not functional
  Serial.println(F("   x  <--  Exit"));                                               // "120" exit

  menutimeout = millis() + 10000;                                                     //wait for user input
  while (millis() < menutimeout) {

    menuinput = 120;                                                                  //will default to ascii 120 "x"
    if (Serial.available() > 0) {
      menuinput = Serial.read();                                                      //get user input
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }

  switch (menuinput) {                                                                //switch cases for menu input, Note the case# corresponds to input in ASCII format
    case 98:                                                                          //"b" for iterating connection of ds18b20 temperature sensors
      Serial.println(F("Iterative identification of ds18b20 temperature sensors."));
      get_integer_input();                                                            //otherwise the first input is always 0?
      Serial.println(F("If you would like to continue, press 1. To return to main menu, press any other character."));
      get_integer_input();
                                                                                      // do not specify int user_resp = indata; // for some reason it does not allow menu access, define the integer then change it.
      int user_resp;
      delay(100);
      user_resp = indata;
      delay(100);
      if (user_resp != 1) {
        menu();
        break;
      } else {
        Serial.println(F("Continuing with iterative identificaiton of ds18b20 temperature sensors..."));
        delay(1000);
        Serial.println(F("How many ds18b20 are being connected?"));
        get_integer_input();
        eeprom_object.num_ds18b20 = indata;
        Serial.print(F("User must complete identification of "));
        Serial.print(eeprom_object.num_ds18b20);
        Serial.println(F(" ds18b20 temperature sensors."));
        Serial.println(F("Prepare to plug in sensors..."));
        delay(2000);
        Identify_1WireDevices();
      }
      menu();
      break;

    case 99:                                                                           // "c" for set clock-----------------------------------------------

      Serial.println(F("Set clock:  "));
      get_integer_input();                                                             //otherwise the first input is always 0?
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

      menu();
      break;

    case 112:                                                                          // "p" for prime pumps--------------------------------
      get_integer_input();                                                             //otherwise the first input is always 0?
      Serial.flush();
      Serial.println(F("Are you sure you want to cycle the pumps?"));
      Serial.println(F("Make sure the lines are oriented where outflow is desired!"));
      Serial.println(F("Type YES to confirm priming of pumps"));

      charinput();
      if (charInput[0]) {
        char answer[4] {0};
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

          digitalWrite(in1, HIGH);                                                    //provide power to relay/switch on respective pin, relays are configured active when HIGH. pumps should be normally open & circuit closed when powered
          delay(10000);
          digitalWrite(in1, LOW);                                                     //power off
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
      menu();
      break;

    case 102:                                                                         // "f" for checking files on sdcard. Non functional 
      Serial.println(F("Getting sdcard information..."));
      sdCheck();
      menu();
      break;

    case 103:                                                                         // "g" for enabling or disabling Radio Transmission.
      Serial.println(F("Would you like to enable radio transmisison?"));
      get_integer_input();                                                            //otherwise first is always 0?
      Serial.println(F("Type 1 to enable radio transmissions or 0 to disable radio transmissions."));
      get_integer_input();
      eeprom_object.toggle_radio = indata;

      if (indata != 1) {
        Serial.println(F("Radio transmissions DISABLED."));
        driver.sleep();                                                               //Puts radio to sleep.
      } else {
        Serial.println(F("Radio transmissions ENABLED."));
        RH_RF95 driver(4, 2);
        RHReliableDatagram manager(driver, radioID);                                  //Set up the radio manager
        if (!driver.init()) {
          Serial.println(F("Radio initialization failed."));
        }
        if (driver.init()) {
          Serial.println(F("Radio initilization successful."));
        }
      }

      break;

    case 104:                                                                         // "h" for enabling or disabling troubleshooting/demo continuous loop.
      Serial.println(F("Would you like to enter troublesooting mode?"));
      Serial.println(F("Troubleshooting mode does not save data to sdcard but DOES send radio transmissions."));
      get_integer_input();                                                            //otherwise first is always 0?
      delay(2000);
      Serial.println(F("If you would like to enable troubleshooting mode enter 1, otherwise enter 0 to enable regular operation."));
      Serial.println(F("Enter any other key to return to the main menu."));
      get_integer_input();

      if (indata == 1 || indata == 0) {
        eeprom_object.demo_mode = indata;
      } else {
        menu();
        break;
      }

      if (eeprom_object.demo_mode) {                                                  //if demo_mode is true (1)...
        Serial.println(F("Troubleshooting mode enabled."));
        delay(1000);
      } else {
        Serial.println(F("Regular operation enabled."));
        delay(1000);
      }
      menu();
      break;

    case 105:                                                                                            // "i" for set ID numbers-----------------------------------------

      Serial.println(F("Set network ID numbers (ProjectID, BoardID, NodeID, GatewayID):"));              // set ProjectID, BoardIDnum, nodeID, and gatewayID numbers
      get_integer_input(); //otherwise the first input is always 0?


      Serial.print(F(" Project ID:     "));                                           // get projectID up to numChars length
      Serial.flush();
      charinput();
      if (charInput[0]) {
        byte i = 0;
        for (int i = 0; i < numChars; i++) {
          if (charInput[i] != 0) {
            eeprom_object.projectID[i] = charInput[i];
          } else {
            break;
          }
        }
      }


      Serial.print(F(" Board ID:     "));                                             // get BoardID
      Serial.flush();
      get_integer_input();                                                            // decode user input
      eeprom_object.IDnum = indata;



      Serial.print(F(" Node ID:  "));                                                 // get nodeID
      Serial.flush();
      get_integer_input();                                                            // decode user input
      eeprom_object.nodeID = indata;


      Serial.print((" Gateway ID:  "));                                               // get GatewayID
      Serial.flush();
      get_integer_input();                                                            // decode user input
      eeprom_object.gatewayID = indata;

      manager.setThisAddress(eeprom_object.nodeID);                                   // Set Radio Address

      menu();                                                                         // go back to menu
      break;

    case 97:                                                                          // "a" for Setting RTC Alarms for measurement interval
      Serial.println(F("Define measurement interval for sensors."));
      get_integer_input();                                                            //otherwise the first input is always 0?
      Serial.println();
      Serial.print(F("Current Measurement Interval (Mins):  "));
      Serial.println(eeprom_object.ALARM_1_Interval);
      Serial.println();
      get_integer_input();
      eeprom_object.ALARM_1_Interval = indata;
      Serial.print(F("Measurement Interval set to:  "));
      Serial.println(eeprom_object.ALARM_1_Interval);
      delay(20);
      menu();
      break;

    case 115:                                                                        // "s" for switch water_manager----------------------------------
      Serial.println(F("Switch Water Manager: "));
      get_integer_input();                                                           // otherwise the first input is always 0?
      Serial.println();
      Serial.println(F("Enable Water Manager?"));
      Serial.print(F("Current Status: "));
      Serial.println( eeprom_object.is_water_manager_on);                            // returns 0 for false and 1 for true
      get_integer_input();
      eeprom_object.is_water_manager_on = indata;
      delay(20);
      menu();
      break;

    case 116:                                                                        // "t" for test measurements-------------------------------------
      Serial.println(F("Test measurements:"));
      get_integer_input();                                                           //otherwise the first input is always 0?
      Serial.println();
      test_measurements();
      delay(10);
      menu();
      break;

    case 119:                                                                        // "w" for setting water threshold levels and min time between irrigation events and reporting of resistance values
      Serial.println(F("Define water threshold level, minimum time between irrigation events, duration of irrigation event, and whether to also print raw resistance values"));
      Serial.println(F("Water threshold levels: "));
      get_integer_input();                                                           //otherwise the first input is always 0?
      Serial.println();
      Serial.print(F("Current Value Group 1: "));                                    
      Serial.println(eeprom_object.water_threshold_1);
      get_integer_input();
      eeprom_object.water_threshold_1 = indata;
      Serial.print(F("Current Value Group 2: "));
      Serial.println(eeprom_object.water_threshold_2);
      get_integer_input();
      eeprom_object.water_threshold_2 = indata;
      Serial.print(F("Current Value Group 3: "));
      Serial.println(eeprom_object.water_threshold_3);
      get_integer_input();
      eeprom_object.water_threshold_3 = indata;
      Serial.print(F("Current Value Group 4: "));
      Serial.println(eeprom_object.water_threshold_4);
      get_integer_input();
      eeprom_object.water_threshold_4 = indata;

      delay(20);

      Serial.println(F("Minimum time between irrigation events (minutes): "));      // define minimum time between irrigation events (units in minutes, 0-255)
      Serial.println();
      Serial.print(F("Current Value: "));
      Serial.println(eeprom_object.min_time_btwn_irr);
      get_integer_input();
      eeprom_object.min_time_btwn_irr = indata;

      Serial.println(F("Duration of irrigation event: "));                          //define duration of irrigation event, seconds. Number gets multiplied by 1000 when put into a delay call (millis operated)
      Serial.println();
      Serial.print(F("Current Value:  "));
      Serial.println(eeprom_object.irr_period);
      get_integer_input();
      eeprom_object.irr_period = indata;

      delay(20);

      Serial.println(F("Do you want to include raw resistance values in the data string? Type 1 for true, Type 0 for false")); // specify if resistance is desired in data string. true = 1 false = 0
      Serial.println();
      Serial.print(F("Current Value: "));
      Serial.print(eeprom_object.include_resistance);
      get_integer_input();
      eeprom_object.include_resistance = indata;

      menu();
      break;

    case 100:                                                                       // "d" for Download all data in sd card----------------------------
      Serial.println(F("Download all data:"));
      delay(100);

      myfile = SD.open(filename);                                                   // open file

      if (myfile) {

        while (myfile.available()) {                                                // read file and print to Serial COM port, Note this will be slow with alot of data due to chip limitations. A desktop with a chip reader is nearly instantaneous.
          Serial.write(myfile.read());
        }
        myfile.close();

      } else {
        Serial.println(F("Error opening file"));
      }
      delay(10000);                                                                 //Give time for user to copy data...
      menu();
      break;

    case 101:                                                                       // "e" for erase data on sd card-------------------------------------------
      Serial.println(F("Erase data on sd card..."));
      Serial.print(F("Currently Writing to: "));
      Serial.println(filename);
      Serial.print(F("Do you want to delete: "));
      Serial.print(filename);
      Serial.println(F("? Type YES to confirm deletion of this file."));

      get_integer_input();                                                          //Otherwise first input is always 0?
      Serial.flush();
      charinput();
      if (charInput[0]) {
        char erase[4] {0};
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
          eeprom_object.firstTime = true;                                           //Return firsttime to true so header is printed when first writing to file
        } else {
          Serial.println(F("If condition not satisfied"));
        }
      }
      menu();
      break;


    case 111:                                                                       //"o" for calibration / fixed resistor settings----------------------------
      get_integer_input();                                                          //Otherwise first input is always 0?
      Serial.flush(); //unneeded?
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

    case 110:                                                                               //"n" for sepecifying number of WM sensors & grouping | 11/18/2021 untested, not sure if will work
      get_integer_input();                                                                  //otherwise the first input is always 0?
      Serial.println(F("Specify number of WaterMark sensors and averaging instructions."));
      Serial.println(F("If you would like to continue, type 1. Press any other key to return to main menu."));
      get_integer_input();
      if (indata != 1) {
        delay(1000);
        Serial.println(F("Returning to main menu."));
        menu();
        break;
      } else {
        Serial.println(F("Clearing Existing Grouping Data."));
        for (int z = 0; z < 16; z++) {
          eeprom_object.WM_group1[z] = -1;
          eeprom_object.WM_group2[z] = -1;
          eeprom_object.WM_group3[z] = -1;
          eeprom_object.WM_group4[z] = -1;
        }
      }
      delay(1000);
      Serial.println(F("Proceed with WaterMark sensor specification."));
      Serial.println(F("Number of WaterMark sensors. Must be installed in sequential order 1-16"));
      Serial.print(F("Current value:  "));
      Serial.println(eeprom_object.num_WM);
      get_integer_input();
      eeprom_object.num_WM = indata;

      Serial.println(F("Determine averaging instructions for the WaterMark sensors."));
      Serial.println(F("Four groups of watermark sensors can be defined based on their channel position of the Multiplexor."));
      Serial.println(F("Specify 0 for no averaging routine."));
      Serial.println(F("Note that the water management routine toggles relays based on group averages."));

      Serial.print(F("Number of channels to average, Group1: "));                             //This works great! repeat for each group 1-4
      get_integer_input();
      int num_sensors = indata;
      Serial.print(F("Number sensors: "));
      Serial.println(num_sensors);
      int num_sensors_group1 = num_sensors;
      eeprom_object.n_channels_wm_group1 = num_sensors;
      
      //Attempt a less memory intensive method------
      for (int i = 0; i < num_sensors; i++) {
        if (num_sensors == 0) {
          break;                                                                              //If 0 sensors are selected, break out of for loop
        }
        int sensor_num = i + 1;
        Serial.print(F("Specify sensor number "));
        Serial.print(sensor_num);
        Serial.println(F("  in group1: "));
        get_integer_input();
        eeprom_object.WM_group1[i] = indata;
      }
      
      Serial.print(F("Number of channels to average, Group2: "));
      get_integer_input();
      num_sensors = indata;
      Serial.print(F("Number sensors: "));
      Serial.println(num_sensors);
      int num_sensors_group2 = num_sensors;
      eeprom_object.n_channels_wm_group2 = num_sensors;
      for (int i = 0; i < num_sensors; i++) {
        if (num_sensors == 0) {
          break;
        }
        int sensor_num = i + 1;
        Serial.print(F("Specify sensor number "));
        Serial.print(sensor_num);
        Serial.println(F("  in group2: "));
        get_integer_input();
        eeprom_object.WM_group2[i] = indata;
      }
      
      Serial.print(F("Number of channels to average, Group3: "));
      get_integer_input();
      num_sensors = indata;
      Serial.print(F("Number sensors: "));
      Serial.println(num_sensors);
      int num_sensors_group3 = num_sensors;
      eeprom_object.n_channels_wm_group3 = num_sensors;
      for (int i = 0; i < num_sensors; i++) {
        if (num_sensors == 0) {
          break;
        }
        int sensor_num = i + 1;
        Serial.print(F("Specify sensor number "));
        Serial.print(sensor_num);
        Serial.println(F("  in group3: "));
        get_integer_input();
        eeprom_object.WM_group3[i] = indata;
      }
      
      Serial.print(F("Number of channels to average, Group4: "));
      get_integer_input();
      num_sensors = indata;
      Serial.print(F("Number sensors: "));
      Serial.println(num_sensors);
      int num_sensors_group4 = num_sensors;
      eeprom_object.n_channels_wm_group4 = num_sensors;
      for (int i = 0; i < num_sensors; i++) {
        if (num_sensors == 0) {
          break;
        }
        int sensor_num = i + 1;
        Serial.print(F("Specify sensor number "));
        Serial.print(sensor_num);
        Serial.println(F("  in group4: "));
        get_integer_input();
        eeprom_object.WM_group4[i] = indata;
      }

      Serial.println(F("Current Grouping Settings."));
      Serial.print(F("Group1: "));
      for (int i = 0; i < num_sensors_group1; i++) {
        Serial.print(eeprom_object.WM_group1[i]);
        Serial.print(F("  "));
      }
      Serial.println();
      Serial.print(F("Group2: "));
      for (int i = 0; i < num_sensors_group2; i++) {
        Serial.print(eeprom_object.WM_group2[i]);
        Serial.print(F("  "));
      }
      Serial.println();
      Serial.print(F("Group3: "));
      for (int i = 0; i < num_sensors_group3; i++) {
        Serial.print(eeprom_object.WM_group3[i]);
        Serial.print(F("  "));
      }
      Serial.println();
      Serial.print(F("Group4: "));
      for (int i = 0; i < num_sensors_group4; i++) {
        Serial.print(eeprom_object.WM_group4[i]);
        Serial.print(F("  "));
      }
      menu();
      break;

    //    case 109:                                               //"m" for repeat menu----------------------------------------------
    //      menu();
    //      break;

    case 120:                                                     //"x" for Exit ---------------------------------------------------
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;


    default:                                                      //Define default case, exit menu if no valid user input
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;


  }
  eeprom_address = 0;                                             //clear eeprom_address
  EEPROM.put(eeprom_address, eeprom_object);                      //store new settings (eeprom_object with structure eeprom_struct) to chip EEPROM
  eeprom_address = 0;                                             //clear eeprom_address
  delay(100);
}

//-----Get User input as integer----------------------------------------------- 
void get_integer_input() {
  menutimeout = millis() + 10000;                                 // time to wait for user to input something
  int sign = 1;                                                   // for handling negatives
  indata = 0;                                                     // initialize
  while (millis() < menutimeout)                                  // wait for user to input something
  {
    if (Serial.available() > 0)                                   // something came in to serial buffer
    {
      delay(100);                                                 // give time for everything to come in
      numincoming = Serial.available();                           // number of incoming bytes (1 byte = 1 HEX (ASCII) character or 8bits of binary)
      for (i = 1; i <= numincoming; i++)                          // read in everything
      {
        incoming[i] = Serial.read();                              // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)               // ignore carriage return & line feed
        {
        }
        else if (incoming[i] == '-') {                            //if a minus is read in
          sign = -1;                                              //update sign to neagative 1
        }
        else if (incoming[i] >= '0' && incoming[i] <= '9')        // otherwise
        {
          input = incoming[i] - '0';                              // convert ASCII value to ??numerical equivalent?? decimal.
          indata = indata * 10 + input * sign;                    // assemble to get total value if sequence of numbers
        }


      }
      break;                                                      // exit before menutimeout
    }
  }
  Serial.println(indata);                                         //return entered value for visual user feedback
  delay(10);
}

//-----Get User Input for Character variables etc.-----------------------------------

void charinput() {
  memset(charInput, 0, sizeof(charInput));
  delay(50);
  long timeout;
  timeout = millis() + 10000;                                     //length of time to wait for user input
  byte numincoming;                                               //for # of bytes incoming

  while (millis() < timeout) {
    if (Serial.available() > 0) {
      delay(100);
      numincoming = Serial.available();
      for (byte i = 0; i <= numincoming; i++) {
        incomingChar[i] = Serial.read();
        if (incomingChar[i] == 13 || incomingChar == 10) {
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

//---------- Clear buffer---------------
void clearBuffer() {
  WM_string = "";
  temperature_string = "";
  irrigation_prompt_string = "";
  data = "";
  delay(100);
}

//-----Return to menu-----------------------
void check_menu() {
  String serial_in;

  serial_in = Serial.readString();
  serial_in.trim();
  if (serial_in.substring(0) == "menu") {
    delay(10);
    Serial.println(F("Return to setup menu"));
    delay(10);
    setup();
  }
}


//-----WM Group means handling------------------                                 //This portion could probably be substantially optimized...

int WM_group_means(int WM_group_num[], int datasize, int water_threshold_group) {

  force_irr = false;                                                             //reset force_irr
  int set_water_threshold;

  if (water_threshold_group == 1) {
    set_water_threshold == eeprom_object.water_threshold_1;
  } else if (water_threshold_group == 2) {
    set_water_threshold == eeprom_object.water_threshold_2;
  } else if (water_threshold_group == 3) {
    set_water_threshold == eeprom_object.water_threshold_3;
  } else if (water_threshold_group == 4) {
    set_water_threshold == eeprom_object.water_threshold_4;
  } else {
    Serial.println(F("Water threshold undefined, define it in the menu."));
  }


  char WM_array[WM_string.length() + 1];                                         //arrays are zero-indexed (start at position 0)
  WM_string.toCharArray(WM_array, sizeof(WM_array));                             //convert the WM_string to a character array

  char *pointer_array[sizeof(WM_array)];                                         //create an empty character (pointer_array) the size of the character array
  char *pointer = NULL;                                                          //declare an empty pointer variable
  byte index = 0;                                                                //Declare an index to increment starting at 0 since arrays are 0 indexed

  pointer = strtok(WM_array, ",");                                               //Looking withing WM_array, a comma is a delimiter for separation, declare what pointer points to
  Serial.print(F("pointer:  "));
  Serial.println(pointer);
  while (pointer != NULL) {
    pointer_array[index] = pointer;                                              //within pointer_array declare the pointer (0-inf) and the value pointed to (pointer)
    index++;
    pointer = strtok(NULL, ",");                                                 //end the while loop
  }

  int sum = 0;                                                                   //declare holders for sum, count
  int count = 0;
  int raw_group_mean = 0;
  int buff_sum = 0;
  int buff_count = 0;
  int count_outliers = 0;                                                        //Start at 0

  for (int i = 0; i < datasize; i++) {                                           //for the declared channel group,
    int q = WM_group_num[i];                                                     //q mirrors each element of channel group up to the globally specified datasize
    if (q != ',' && q != ' ' && q != '-' && q != "-1" && q <= 16) {              //if q is not a comma or a space, -1, or some random number above 16...
      //  q = (q * 2) + 1;                                                       //q is the location of the data of the desired channel, because channel of mux and arrays begin at 0, the value of channel 0 will be on element 1 of the array. formula is x*2+1                                                                                 //This is of course different if resistance values are included in the string.
      if (eeprom_object.include_resistance == true) {
        q = (q * 3) + 2;                                                         //if we are including resistances in the wm_string, the formula is x*3+2
      } else {
        q = (q * 2) + 1;
      }
      int wm_val;
      wm_val = atoi(pointer_array[q]);
      Serial.print(F("wm_val: "));
      Serial.println(wm_val);

      if (wm_val != 255 && wm_val != 240 && wm_val != 0) {                        //If not throwing a fault code calculate an initial mean
        sum += abs(wm_val);
        count++;
      }
      if (WM_group_num[i] != -1 && WM_group_num <= 16 && wm_val != 255 && wm_val != 240 && wm_val != 0) {
        Serial.print(F("Node: "));                                                //If a fault code was one of the data requested in the group
        Serial.print(eeprom_object.nodeID);                                       //Serial.print the nodeID, channel (algorithim is (x-1) / 2), and value of the fault
        Serial.print(F("  Fault code on channel:  "));
        Serial.print(WM_group_num[i]);
        Serial.print(F("  with value: "));
        Serial.print(wm_val);
        Serial.println(F(".  Value not included in Group mean."));
      }
    }
  }

  raw_group_mean = (sum / count) * -1;                                            //calculate the raw_mean of the of the desired channels, negative 1 here as kpa is in tension
  Serial.print(F("Raw group mean: "));
  Serial.println(raw_group_mean);


  //Run through calculation again, if percent difference of each sensor is < 20% from the raw_group_mean (all sensors as long as not fault code) include in the calculation of the buffered_mean that gets output & triggers irr events.
  // 04/25/2022 Note that pdiff calc is working but that small differences in kpa result in large percent differences due to scale...
  // Need to consider a secondary flag. for example a "grace window" -> if the pdiff is > 20, check grace window. -> grace window = 10 (could be changed) -> if this sensors kpa is +10 or -10 from the raw_group_mean OR the set water threshold, include in calc of buffered mean, else exclude from buffered mean.
 
  for (int i = 0; i < datasize; i++) {                                           //for the declared channel group,
    int q = WM_group_num[i];                                                     //q mirrors each element of channel group up to the globally specified datasize
    if (q != ',' && q != ' ' && q != '-' && q != "-1" && q <= 16) {              //if q is not a comma or a space, -1, or some random number above 16...
      if (eeprom_object.include_resistance == true) {                            //q is the location of the data of the desired channel, because channel of mux and arrays begin at 0, the value of channel 0 will be on element 1 of the array. formula is x*2+1
        q = (q * 3) + 2;                                                         //if we are including resistances in the wm_string, the formula is x*3+2
      } else {
        q = (q * 2) + 1;
      }
      int wm_val;
      wm_val = atoi(pointer_array[q]);
      
      ////// Another Question needs addressed. What if the mean is such that each individual sensor is >= 20% different from the raw_mean? ->> It needs to trigger the irrigation event anyway.
      if (wm_val != 255 && wm_val != 256 && wm_val != 240 && wm_val != 0 && WM_group_num[i] != -1) {
        if (abs((abs(raw_group_mean - wm_val) / ((raw_group_mean + wm_val) / 2.0)) * 100) < 20.0) {    //if the sensor reading is less than 20% different from the raw_group_mean, include in calc of buffered_group_mean
          Serial.print(F("Channel: "));
          Serial.print(WM_group_num[i]);
          Serial.print(F("  value:  "));
          Serial.print(wm_val);
          Serial.print(F(" Pdiff from raw mean = "));
          Serial.println(abs(((abs(raw_group_mean - wm_val)) / ((raw_group_mean + wm_val) / 2.0)) * 100));
          buff_sum += abs(wm_val);
          buff_count++;

        } else if ((wm_val - raw_group_mean >= wm_grace_window || wm_val - raw_group_mean <= -1 * wm_grace_window) && (wm_val - set_water_threshold >= wm_grace_window || wm_val - raw_group_mean <= -1 * wm_grace_window)) {     //check grace window against raw group mean and the user defined water threshold, wm_grace_window defined globally.
          Serial.print(F("Channel: "));
          Serial.print(WM_group_num[i]);
          Serial.print(F("  value:  "));
          Serial.print(wm_val);
          Serial.print(F(" Pdiff from raw mean = "));
          Serial.print(abs(((abs(raw_group_mean - wm_val)) / ((raw_group_mean + wm_val) / 2.0)) * 100));
          Serial.print(F("  detected as an outlier (>20 pdiff & > +- 10kpa from raw_group_mean)"));
          Serial.println(F("  This channel is not counted in buffered group mean."));
          count_outliers ++;                                                                            //increment outlier count.
        } else {
          Serial.print(F("Channel: "));
          Serial.print(WM_group_num[i]);
          Serial.print(F("  value:  "));
          Serial.print(wm_val);
          Serial.print(F("  detected as a POSSIBLE outlier (>20 pdiff from raw_group_mean"));
          Serial.print(F(" Pdiff from raw mean = "));
          Serial.println(abs(((abs(raw_group_mean - wm_val)) / ((raw_group_mean + wm_val) / 2.0)) * 100));
          buff_sum += abs(wm_val);                                                                      //include in buffered_mean calc anyway if + - 10 from raw mean or set water threshold
          buff_count++;
        }
      }
    }
  }
  Serial.print(F("Total Outliers: "));
  Serial.println(count_outliers);

  // if(WM_group) <-- in the future, use this here to grab from eeprom, not explicitly defined below-----

  if (count_outliers >= 3) {                                                            //  num_sensors in each group, This needs changed to reflect what is stored in eeprom for each group. The problem is that the rest of this function alone does not know which group it is dealing with so it would be hard to call the variable with the correct number in eeprom...
    Serial.println(F("Number of outliers equal to number of sensors (edge case). Check and see whether to force irrigation event or encountering error."));
    Serial.print(F("Raw group mean is equal to: "));
    Serial.println(raw_group_mean);
    if ((raw_group_mean <= 0 && raw_group_mean >= -239) && (raw_group_mean < set_water_threshold)) {
      Serial.print(F("The raw group mean is reasonable and is lower than the set water threshold for the group."));
      Serial.println(F("  Proceed with forced irrigation event, note this result is possible with poor connection of ALL WM sensors in the group."));
      force_irr = true;
    } else {
      Serial.println(F("The raw group mean is not within the measurement range of WM sensors OR is higher than the set water threshold for the group."));
      Serial.println(F("Force irrigation event prevented."));
    }
  }

  Serial.print(F("buff_sum:  "));
  Serial.println(buff_sum);
  Serial.print(F("buff_count:  "));
  Serial.println(buff_count);
  int buffered_group_mean;
  buffered_group_mean = (buff_sum / buff_count) * -1;                                    //calculate the mean of the of the desired channels, negative 1 here as kpa is in tension

  Serial.print(F("Buffered Group mean: "));
  Serial.println(buffered_group_mean);

  if (force_irr) {
    Serial.println("Force irrigation event detected.");                                  //Give indication you are forcing the irrigation event
    buffered_group_mean = -999;
  }

  return buffered_group_mean;                                                            //return the requested group_mean
}

//-----Water manager irrigation prompt--------------------------------------------------------

//New prompt for the 4 threshold groups of sensors.
uint32_t WM_irrigation_prompt(int WM_group_num, int WM_group_mean, int WM_group_water_threshold, uint32_t last_irr_time_for_group) { // The way this is setup as a function, you just run it for each group, no loop req.
  uint32_t update_last_irr_time;                                                                                                     // an updated last irr time is needed.

  // if sensors indicate the need for a watering event (for each group threshold)-----
  if (WM_group_num == 1) {
    if (WM_group_mean < eeprom_object.water_threshold_1) {                               //if sensors indicate need for watering event...
      Serial.print(F("Need for watering event indicated for sensor group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  with a mean of: "));
      Serial.println(WM_group_mean);
      DateTime now = rtc.now();                                                         //needed to get unix time in next line
      uint32_t current_unix_epoch_time = now.unixtime();                                //get current unix epoch time
      if (current_unix_epoch_time - last_irr_time_for_group >= (eeprom_object.min_time_btwn_irr * 60)) {                            //IF the time since last irrigation event is greater than or equal to the minnimum time between irrigations (seconds*60 = min)...
        
        // 2022/03/22 Note that this will need changed in future if separate timing differences are specified for each group-----
        Serial.println(F("The minimum time since last irrigation event has been exceeded. Proceed with irrigation")); 
        digitalWrite(in1, HIGH);                                                        //provide power to pump on relay on respective pin
        delay(eeprom_object.irr_period * 1000);                                         //defined length/duration of irrigation event (seconds so * 1000)
        digitalWrite(in1, LOW);                                                         //open the respective relay pin, removing power to the pump
        last_irr_time_unix_epoch_group1 = now.unixtime();                               //reset the time of last irrigation event for that group on the global variable
        update_last_irr_time = last_irr_time_unix_epoch_group1;

        local_time(last_irr_time_unix_epoch_group1);
        delay(10);

        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;                                       //Amend relevant data to string for storage/transmission
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';

        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                       //The global variable keeping track of last irr event
          } else {
            break;
          }
        }
        irrigation_prompt_string += ',';
                                                                                        //add condition to not overwater??
                                                                                        //irr_count ++;                                     
                                                                                        //like incrementing irr_count for throwing flag if X events take place in Y time?
                                                                                        //Then do something? or send flag to gateway?
      }  else {
        Serial.print(F("Minimum Time between irrigation events not reached for Group: ")); //declare that the minimum time between irrigations has not elapsed for specified group
        Serial.print(WM_group_num);
        Serial.print(F("  with a mean of: "));
        Serial.println(WM_group_mean);
                                                                                        //Report the group #, group mean, and a unix timestamp of the last irrigation event to the irrigation_prompt_string that gets saved etc.
        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';
                                                                                        //different here than in case above
                                                                                        //if minimum time between irrigations has not been exceeded, return the time of last irrigation event for the group
        if (WM_group_num == 1) {
          local_time(last_irr_time_unix_epoch_group1);
          delay(10);
        } else if (WM_group_num == 2) {
          local_time(last_irr_time_unix_epoch_group2);
          delay(10);
        } else if (WM_group_num == 3) {
          local_time(last_irr_time_unix_epoch_group3);
          delay(10);
        } else if (WM_group_num == 4) {
          local_time(last_irr_time_unix_epoch_group4);
          delay(10);
        }
        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                       //The global variable keeping track of last irr event
          } else {
            break;
          }
        }
        irrigation_prompt_string += ',';
      }
    } else {
      Serial.print(F("Group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  Mean: "));
      Serial.print(WM_group_mean);
      Serial.print(F(", Threshold water content of "));
      Serial.print(eeprom_object.water_threshold_1);
      Serial.println(F("  has not been exceeded."));
                                                                                      //New print routine to add to the irrigation_prompt_string even when water threshold has not been reached
                                                                                      //and min time has not elapsed
      irrigation_prompt_string += 'G';                                                //This is looped through for each group
      irrigation_prompt_string += WM_group_num;                                       //Amend relevant data to string for storage/transmission
      irrigation_prompt_string += ',';
      irrigation_prompt_string += WM_group_mean;
      irrigation_prompt_string += ',';

      if (WM_group_num == 1) {
        local_time(last_irr_time_unix_epoch_group1);
        delay(10);
      } else if (WM_group_num == 2) {
        local_time(last_irr_time_unix_epoch_group2);
        delay(10);
      } else if (WM_group_num == 3) {
        local_time(last_irr_time_unix_epoch_group3);
        delay(10);
      } else if (WM_group_num == 4) {
        local_time(last_irr_time_unix_epoch_group4);
        delay(10);
      }

      for (int i = 0; i <= numChars; i++) {
        if (local_time_irr_update[i] != '\0') {
          irrigation_prompt_string += local_time_irr_update[i];                       //The global variable keeping track of last irr event
        } else {
          break;
        }
      }

      irrigation_prompt_string += ',';
    }

    /*
      For troubleshooting...
      Serial.print(F("The last irr time for group 1  = "));
      local_time(last_irr_time_unix_epoch_group1);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group1);
      Serial.print(F("The last irr time for group 2  = "));
      local_time(last_irr_time_unix_epoch_group2);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group2);
      Serial.print(F("The last irr time for group 3  = "));
      local_time(last_irr_time_unix_epoch_group3);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group3);
      Serial.println();
      Serial.println(F("END Water Manager Loop."));
      Serial.println();
    */

  } else if (WM_group_num == 2) {
    if (WM_group_mean < eeprom_object.water_threshold_2) {                          //if sensors indicate need for watering event...
      Serial.print(F("Need for watering event indicated for sensor group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  with a mean of: "));
      Serial.println(WM_group_mean);
      DateTime now = rtc.now();                                                     //needed to get unix time in next line
      uint32_t current_unix_epoch_time = now.unixtime();                            //get current unix epoch time
      if (current_unix_epoch_time - last_irr_time_for_group >= (eeprom_object.min_time_btwn_irr * 60)) {              //IF the time since last irrigation event is greater than or equal to the minnimum time between irrigations (seconds*60 = minutes)...
        Serial.println(F("The minimum time since last irrigation event has been exceeded. Proceed with irrigation")); //Note that this will need changed in future if separate timing differences are specified for each group...
        digitalWrite(in2, HIGH);
        delay(eeprom_object.irr_period * 1000);
        digitalWrite(in2, LOW);
        last_irr_time_unix_epoch_group2 = now.unixtime();
        update_last_irr_time = last_irr_time_unix_epoch_group2;
        local_time(last_irr_time_unix_epoch_group2);
        delay(10);

        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;                                   //Amend relevant data to string for storage/transmission
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';
        
        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                   //The global variable keeping track of last irr event
          } else {
            break;
          }
        }
        irrigation_prompt_string += ',';
      }  else {
        Serial.print(F("Minimum Time between irrigation events not reached for Group: "));  //declare that the minimum time between irrigations has not elapsed for specified group
        Serial.print(WM_group_num);
        Serial.print(F("  with a mean of: "));
        Serial.println(WM_group_mean);
                                                                                            //Report the group #, group mean, and a unix timestamp of the last irrigation event to the irrigation_prompt_string that gets saved etc.
        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';
                                                                                            //different here than in case above
                                                                                            //if minimum time between irrigations has not been exceeded, return the time of last irrigation event for the group
        if (WM_group_num == 1) {
          local_time(last_irr_time_unix_epoch_group1);
          delay(10);
        } else if (WM_group_num == 2) {
          local_time(last_irr_time_unix_epoch_group2);
          delay(10);
        } else if (WM_group_num == 3) {
          local_time(last_irr_time_unix_epoch_group3);
          delay(10);
        } else if (WM_group_num == 4) {
          local_time(last_irr_time_unix_epoch_group4);
          delay(10);
        }
        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                           //The global variable keeping track of last irr event
          } else {
            break;
          }
        }

        irrigation_prompt_string += ',';
      }
    } else {
      Serial.print(F("Group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  Mean: "));
      Serial.print(WM_group_mean);
      Serial.print(F(", Threshold water content of "));
      Serial.print(eeprom_object.water_threshold_2);
      Serial.println(F("  has not been exceeded."));
                                                                //New print routine to add to the irrigation_prompt_string even when water threshold has not been reached
                                                                //and min time has not elapsed
      irrigation_prompt_string += 'G';                          //This is looped through for each group
      irrigation_prompt_string += WM_group_num;                 //Amend relevant data to string for storage/transmission
      irrigation_prompt_string += ',';
      irrigation_prompt_string += WM_group_mean;
      irrigation_prompt_string += ',';

      if (WM_group_num == 1) {
        local_time(last_irr_time_unix_epoch_group1);
        delay(10);
      } else if (WM_group_num == 2) {
        local_time(last_irr_time_unix_epoch_group2);
        delay(10);
      } else if (WM_group_num == 3) {
        local_time(last_irr_time_unix_epoch_group3);
        delay(10);
      } else if (WM_group_num == 4) {
        local_time(last_irr_time_unix_epoch_group4);
        delay(10);
      }

      for (int i = 0; i <= numChars; i++) {
        if (local_time_irr_update[i] != '\0') {
          irrigation_prompt_string += local_time_irr_update[i];  //The global variable keeping track of last irr event
        } else {
          break;
        }
      }

      irrigation_prompt_string += ',';
    }

    /*
      For troubleshooting... or else printed during each run through function
      Serial.print(F("The last irr time for group 1  = "));
      local_time(last_irr_time_unix_epoch_group1);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group1);
      Serial.print(F("The last irr time for group 2  = "));
      local_time(last_irr_time_unix_epoch_group2);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group2);
      Serial.print(F("The last irr time for group 3  = "));
      local_time(last_irr_time_unix_epoch_group3);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group3);
      Serial.println();
      Serial.println(F("END Water Manager Loop."));
      Serial.println();
    */

  } else if (WM_group_num == 3) {
    if (WM_group_mean < eeprom_object.water_threshold_3) {                          //if sensors indicate need for watering event...
      Serial.print(F("Need for watering event indicated for sensor group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  with a mean of: "));
      Serial.println(WM_group_mean);
      DateTime now = rtc.now();                                                     //needed to get unix time in next line
      uint32_t current_unix_epoch_time = now.unixtime();                            //get current unix epoch time
      if (current_unix_epoch_time - last_irr_time_for_group >= (eeprom_object.min_time_btwn_irr * 60)) {              //IF the time since last irrigation event is greater than or equal to the minnimum time between irrigations (seconds*60 = min)...
        Serial.println(F("The minimum time since last irrigation event has been exceeded. Proceed with irrigation")); //Note that this will need changed in future if separate timing differences are specified for each group...
        digitalWrite(in3, HIGH);
        delay(eeprom_object.irr_period * 1000);
        digitalWrite(in3, LOW);
        last_irr_time_unix_epoch_group3 = now.unixtime();
        update_last_irr_time = last_irr_time_unix_epoch_group3;
        local_time(last_irr_time_unix_epoch_group3);
        delay(10);

        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;                                   //Amend relevant data to string for storage/transmission
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';

        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                   //The global variable keeping track of last irr event
          } else {
            break;
          }
        }

        irrigation_prompt_string += ',';
      }  else {
        Serial.print(F("Minimum Time between irrigation events not reached for Group: "));  //declare that the minimum time between irrigations has not elapsed for specified group
        Serial.print(WM_group_num);
        Serial.print(F("  with a mean of: "));
        Serial.println(WM_group_mean);
                                                                                            //Report the group #, group mean, and a unix timestamp of the last irrigation event to the irrigation_prompt_string that gets saved etc.
        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';
                                                                                            //different here than in case above
                                                                                            //if minimum time between irrigations has not been exceeded, return the time of last irrigation event for the group
        if (WM_group_num == 1) {
          local_time(last_irr_time_unix_epoch_group1);
          delay(10);
        } else if (WM_group_num == 2) {
          local_time(last_irr_time_unix_epoch_group2);
          delay(10);
        } else if (WM_group_num == 3) {
          local_time(last_irr_time_unix_epoch_group3);
          delay(10);
        } else if (WM_group_num == 4) {
          local_time(last_irr_time_unix_epoch_group4);
          delay(10);
        }
        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                           //The global variable keeping track of last irr event
          } else {
            break;
          }
        }
        irrigation_prompt_string += ',';
      }
    } else {
      Serial.print(F("Group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  Mean: "));
      Serial.print(WM_group_mean);
      Serial.print(F(", Threshold water content of "));
      Serial.print(eeprom_object.water_threshold_3);
      Serial.println(F("  has not been exceeded."));
                                                                //New print routine to add to the irrigation_prompt_string even when water threshold has not been reached
                                                                //and min time has not elapsed
      irrigation_prompt_string += 'G';                          //This is looped through for each group
      irrigation_prompt_string += WM_group_num;                 //Amend relevant data to string for storage/transmission
      irrigation_prompt_string += ',';
      irrigation_prompt_string += WM_group_mean;
      irrigation_prompt_string += ',';

      if (WM_group_num == 1) {
        local_time(last_irr_time_unix_epoch_group1);
        delay(10);
      } else if (WM_group_num == 2) {
        local_time(last_irr_time_unix_epoch_group2);
        delay(10);
      } else if (WM_group_num == 3) {
        local_time(last_irr_time_unix_epoch_group3);
        delay(10);
      } else if (WM_group_num == 4) {
        local_time(last_irr_time_unix_epoch_group4);
        delay(10);
      }

      for (int i = 0; i <= numChars; i++) {
        if (local_time_irr_update[i] != '\0') {
          irrigation_prompt_string += local_time_irr_update[i]; //The global variable keeping track of last irr event
        } else {
          break;
        }
      }

      irrigation_prompt_string += ',';
    }

    /*
      For troubleshooting... or else printed during each run through function
      Serial.print(F("The last irr time for group 1  = "));
      local_time(last_irr_time_unix_epoch_group1);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group1);
      Serial.print(F("The last irr time for group 2  = "));
      local_time(last_irr_time_unix_epoch_group2);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group2);
      Serial.print(F("The last irr time for group 3  = "));
      local_time(last_irr_time_unix_epoch_group3);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group3);
      Serial.println();
      Serial.println(F("END Water Manager Loop."));
      Serial.println();
    */

  } else if (WM_group_num == 4) {
    if (WM_group_mean < eeprom_object.water_threshold_4) {                        //if sensors indicate need for watering event...
      Serial.print(F("Need for watering event indicated for sensor group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  with a mean of: "));
      Serial.println(WM_group_mean);
      DateTime now = rtc.now();                                                   //needed to get unix time in next line
      uint32_t current_unix_epoch_time = now.unixtime();                          //get current unix epoch time
      if (current_unix_epoch_time - last_irr_time_for_group >= (eeprom_object.min_time_btwn_irr * 60)) {              //IF the time since last irrigation event is greater than or equal to the minnimum time between irrigations (seconds*60=min)...
        Serial.println(F("The minimum time since last irrigation event has been exceeded. Proceed with irrigation")); //Note that this will need changed in future if separate timing differences are specified for each group...
        digitalWrite(in4, HIGH);
        delay(eeprom_object.irr_period * 1000);
        digitalWrite(in4, LOW);
        last_irr_time_unix_epoch_group4 = now.unixtime();
        update_last_irr_time = last_irr_time_unix_epoch_group4;
        local_time(last_irr_time_unix_epoch_group4);
        delay(10);

        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;                                 //Amend relevant data to string for storage/transmission
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';

        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                 //The global variable keeping track of last irr event
          } else {
            break;
          }
        }

        irrigation_prompt_string += ',';
      }  else {
        Serial.print(F("Minimum Time between irrigation events not reached for Group: ")); //declare that the minimum time between irrigations has not elapsed for specified group
        Serial.print(WM_group_num);
        Serial.print(F("  with a mean of: "));
        Serial.println(WM_group_mean);
                                                                                           //Report the group #, group mean, and a unix timestamp of the last irrigation event to the irrigation_prompt_string that gets saved etc.
        irrigation_prompt_string += 'G';
        irrigation_prompt_string += WM_group_num;
        irrigation_prompt_string += ',';
        irrigation_prompt_string += WM_group_mean;
        irrigation_prompt_string += ',';
                                                                                           //different here than in case above
                                                                                           //if minimum time between irrigations has not been exceeded, return the time of last irrigation event for the group
        if (WM_group_num == 1) {
          local_time(last_irr_time_unix_epoch_group1);
          delay(10);
        } else if (WM_group_num == 2) {
          local_time(last_irr_time_unix_epoch_group2);
          delay(10);
        } else if (WM_group_num == 3) {
          local_time(last_irr_time_unix_epoch_group3);
          delay(10);
        } else if (WM_group_num == 4) {
          local_time(last_irr_time_unix_epoch_group4);
          delay(10);
        }
        for (int i = 0; i <= numChars; i++) {
          if (local_time_irr_update[i] != '\0') {
            irrigation_prompt_string += local_time_irr_update[i];                         //The global variable keeping track of last irr event
          } else {
            break;
          }
        }

        irrigation_prompt_string += ',';
      }
    } else {
      Serial.print(F("Group: "));
      Serial.print(WM_group_num);
      Serial.print(F("  Mean: "));
      Serial.print(WM_group_mean);
      Serial.print(F(", Threshold water content of "));
      Serial.print(eeprom_object.water_threshold_4);
      Serial.println(F("  has not been exceeded."));
                                                                //New print routine to add to the irrigation_prompt_string even when water threshold has not been reached
                                                                //and min time has not elapsed
      irrigation_prompt_string += 'G';                          //This is looped through for each group
      irrigation_prompt_string += WM_group_num;                 //Amend relevant data to string for storage/transmission
      irrigation_prompt_string += ',';
      irrigation_prompt_string += WM_group_mean;
      irrigation_prompt_string += ',';

      if (WM_group_num == 1) {
        local_time(last_irr_time_unix_epoch_group1);
        delay(10);
      } else if (WM_group_num == 2) {
        local_time(last_irr_time_unix_epoch_group2);
        delay(10);
      } else if (WM_group_num == 3) {
        local_time(last_irr_time_unix_epoch_group3);
        delay(10);
      } else if (WM_group_num == 4) {
        local_time(last_irr_time_unix_epoch_group4);
        delay(10);
      }

      for (int i = 0; i <= numChars; i++) {
        if (local_time_irr_update[i] != '\0') {
          irrigation_prompt_string += local_time_irr_update[i];//The global variable keeping track of last irr event
        } else {
          break;
        }
      }

      irrigation_prompt_string += ',';
    }
    /*
      For troubleshooting... or else printed during each run through function
      Serial.print(F("The last irr time for group 1  = "));
      local_time(last_irr_time_unix_epoch_group1);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group1);
      Serial.print(F("The last irr time for group 2  = "));
      local_time(last_irr_time_unix_epoch_group2);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group2);
      Serial.print(F("The last irr time for group 3  = "));
      local_time(last_irr_time_unix_epoch_group3);
      Serial.print(F("  OR  UNIX  "));
      Serial.println(last_irr_time_unix_epoch_group3);
      Serial.println();
      Serial.println(F("END Water Manager Loop."));
      Serial.println();
    */

  } else {
    Serial.println(F("Undefined Group number..."));
  }
}

//----print ROM of ds18b20----------------------------------------------- 
void printROM() {
  for (int i = 0; i <= 15; i++) {
    Serial.print(F("Sensor  "));
    Serial.print(i);
    Serial.print(F(" : "));
    sensors.getAddress(DS18B20_Address, i);
    delay(10);
    printAddress(DS18B20_Address);
    printTemperature(DS18B20_Address);
  }
}
////
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.print(F(" "));
}
void printTemperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  delay(100);
  Serial.print(tempC);
  Serial.println(F("C"));
}


//------Calc battery voltage (0- ~ 10V) with voltage divider circuit------------- Closed/opened by mosfet with digital input to prevent battery drain.
float calcbattV() {
  pinMode(pin_mBatt, OUTPUT);             //Set digital pin as an output to activate the circuit
  pinMode(pin_battV, INPUT);              //Set the other pin as input to read the voltage

  digitalWrite(pin_mBatt, HIGH);          //Turn the gate of the MOSFET HIGH, closing the circuit
  delayMicroseconds(500);                 //delay time for stabilization

  uint16_t vInt;                          //The raw integer returned by analogRead
  vInt = analogRead(pin_battV);           //read vInt on analog pin connected where the resistors in the divider circuit meet.
  digitalWrite(pin_mBatt, LOW);           //Turn the gate of the MOSFET LOW, deactivating the circuit

  float vout = 0.00;
  float vin = 0.00;
  float R1 = 10000;                       //1st Resistor in divider circuit taking load from battery
  float R2 = 4700;                        //2nd Resistor in divider circuit leading to board GND

  vout = (vInt * ADC_REF_VOLTAGE) / ADC_RESOLUTION;             //voltage modified by divider circuit, voltage at dividing point
  vin = vout / (R2 / (R1 + R2));                                //Actual voltage coming into the divider circuit, i.e. battery level
  return vin;
}

//-----Return sdCard information----------------------------------------- Nonfunctional

void sdCheck() {
  Sd2Card card;                                 // set up variables using the SD utility library functions
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

  volumesize = volume.blocksPerCluster();       // clusters are collections of blocks
  volumesize *= volume.clusterCount();          // we'll have a lot of clusters
  volumesize /= 2;                              // SD card blocks are always 512 bytes (2 blocks are 1KB

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
  temp.toCharArray(local_time_irr_update, numChars);                      //Update the global irr_update variable
}

//------get nist time-------------------------------------- Non functional, intended to pull datetime from a standard online source
//void get_nist_time();

//-----time_sync------------------------------------------- Non functional, intended to standardize times of Nodes connected to a Gateway
                                                                          //Needs to...
                                                                          //Import time from connected gateway
                                                                          //Import measurement interval from connected gateway (transmisison time)
                                                                          //Gateway needs to wake ~1 minute before nodes send transmission and wait until 1 min after the nodes wake to stop listening?

void time_sync() {
  bool message_received = false;
  uint8_t buf[20];                                                        //Array to receive time from gateway
  uint8_t len = sizeof(buf);
  uint8_t from;
  digitalWrite(15, HIGH);                                                 //Turn board LED on while getting radio data

  if (!driver.init())
    Serial.println(F("Radio failed to initialize."));                     //If the radio manager didnt initialize...
  delay(20);
  if (driver.init()) {
    Serial.println(F("Radio initilization successful."));
  }



  if (manager.recvfromAck(buf, &len, &from_id)) {
    DateTime now = rtc.now();                                             //needed to get unix time in next line
    uint32_t message_received_unix_time = now.unixtime();                 //get current unix epoch time from the node when message was received
    int nYear;
    int nMonth;
    int nDay;
    int nHour;
    int nMin;
    int nSec;
    char *nbuf = (char*)buf;                                              //arrange data in character array
    for (int i = 0; i = len; i++) {
      Serial.print(buf[i]);                                               //What came through
    }
    Serial.println();
    uint32_t message_sent_unix_time;
    uint32_t transmission_time = (message_received_unix_time - message_sent_unix_time); //How long did it take to receive the message, i.e. the transmission time
    uint32_t update_time = (transmission_time + message_received_unix_time);            //Add transmision time to the time received for sync.
                                                                                        //getting from unix to std time format
    byte month_i = month(update_time);
    byte day_i = day(update_time);
    int year_i = year(update_time);
    byte hour_i = hour(update_time);
    byte minute_i = minute(update_time);
    byte second_i = second(update_time);

    Serial.print(F("Recieved Request From : 0x"));
    Serial.print(from_id, HEX);
    Serial.print(F(": "));
    Serial.println((char*)int_buff);
    message_received = true;

    if (message_received = true) {
      if (sscanf(nbuf, "%d/%d/%d|%d:%d:%d", &nYear, &nMonth, &nDay, &nHour, &nMin, &nSec)) {// evaluate buffer and set clock to new time
        tm.Year = nYear - 1970;
        tm.Month = nMonth;
        tm.Day = nDay;
        tm.Hour = nHour;
        tm.Minute = nMin;
        tm.Second = nSec;
        RTC.write(tm);      // update clock
        Serial.print(F("Received time from Gateway: "));
        Serial.println(nbuf);
      }
    }
  }
  digitalWrite(15, LOW);
}

//-----Set ALARM1 ---------------------------------
void Set_ALARM_1_Interval() {
  alarm_1_Mins = mins + eeprom_object.ALARM_1_Interval;
/*
 *  //For troubleshooting
  //Serial.print(F("eeprom_object.ALARM_1_Interval: "));
  //Serial.println(eeprom_object.ALARM_1_Interval);

  //Serial.print(F("alarm_1_Mins: "));
  //Serial.println(alarm_1_Mins);
 */
  if (alarm_1_Mins >= 60) {
    alarm_1_Mins = (alarm_1_Mins % 60);                                 //if >=60, modulo 60 to get remainding minutes
  }
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, alarm_1_Mins, 0, 0);

  Serial.print(F("Alarm at: "));
  Serial.print(alarm_1_Mins);
  Serial.println(F("  minutes past the hour."));
}
//-----Identification of ds18b20 addresses-----------
void Identify_1WireDevices() {
  byte addr[8];
  byte addr0[8];
  byte addr1[8];
  byte addr2[8];
  byte addr3[8];
  byte addr4[8];
  byte addr5[8];
  byte addr6[8];
  byte addr7[8];
  byte addr8[8];
  byte addr9[8];
  byte addr10[8];
  byte addr11[8];
  byte addr12[8];
  byte addr13[8];
  byte addr14[8];
  byte addr15[8];

  int intSensorsPassed = 0;

  Serial.println(F("Looking for 1-wire devices..."));
  delay(1000);
  Serial.println(F("Connect sensor 1..."));
  do {
    while (oneWire.search(addr)) {
      if ( OneWire::crc8( addr, 7) != addr[7]) {
        Serial.print(F("CRC is not valid!\n"));                            
        Serial.println();
        Serial.print(F("CRC:  "));
        Serial.print(OneWire::crc8( addr, 7));
        Serial.print(F("addr: "));
        Serial.println(addr[7]);
        delay (100);
        break;
      }
      if (addr[0] != 0x28) {
        Serial.println (" Not a DS18b20");                                      //The least significant byte of the DS18b20 should be 0x28
        break;
      }

      switch (intSensorsPassed) {
        case 0:
          memcpy(addr0, addr, 8);  //Store the first address in addr0
          intSensorsPassed++;
          Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr0); Serial.println();
          Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
          Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          break;

        case 1:
                                                                                //check if the address was earlier identified
          if (memcmp(addr, addr0, 8) != 0) {
            memcpy (addr1, addr, 8);                                            //Store the addess in addr1
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr1); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 2:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0)) {
            memcpy (addr2, addr, 8);                                            //Store the addess in addr2
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr2); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 3:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0)) {
            memcpy (addr3, addr, 8);                                            //Store the addess in addr3
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr1); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 4:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0)) {
            memcpy (addr4, addr, 8);                                            //Store the addess in addr4
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr4); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 5:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0)) {
            memcpy (addr5, addr, 8);                                            //Store the addess in addr5
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr5); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 6:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0)) {
            memcpy (addr6, addr, 8);                                            //Store the addess in addr6
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr6); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 7:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0)) {
            memcpy (addr7, addr, 8);                                            //Store the addess in addr7
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr7); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 8:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0)) {
            memcpy (addr8, addr, 8);                                            //Store the addess in addr8
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr8); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 9:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0)) {
            memcpy (addr9, addr, 8);                                            //Store the addess in addr9
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr9); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 10:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0)) {
            memcpy (addr10, addr, 8);                                           //Store the addess in addr10
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr10); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 11:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0)) {
            memcpy (addr11, addr, 8);                                           //Store the addess in addr11
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr11); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 12:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0)) {
            memcpy (addr12, addr, 8);                                           //Store the addess in addr12
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr12); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 13:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0) && (memcmp(addr, addr12, 8) != 0)) {
            memcpy (addr13, addr, 8);                                           //Store the addess in addr13
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr13); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 14:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0) && (memcmp(addr, addr12, 8) != 0) && (memcmp(addr, addr13, 8) != 0)) {
            memcpy (addr14, addr, 8);                                           //Store the addess in addr14
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr14); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

        case 15:
                                                                                //check if the address was earlier identified
          if ((memcmp(addr, addr0, 8) != 0)  && (memcmp(addr, addr1, 8) != 0) && (memcmp(addr, addr2, 8) != 0) && (memcmp(addr, addr3, 8) != 0) && (memcmp(addr, addr4, 8) != 0) && (memcmp(addr, addr5, 8) != 0) && (memcmp(addr, addr6, 8) != 0) && (memcmp(addr, addr7, 8) != 0) && (memcmp(addr, addr8, 8) != 0) && (memcmp(addr, addr9, 8) != 0) && (memcmp(addr, addr10, 8) != 0) && (memcmp(addr, addr11, 8) != 0) && (memcmp(addr, addr12, 8) != 0) && (memcmp(addr, addr13, 8) != 0) && (memcmp(addr, addr14, 8) != 0)) {
            memcpy (addr15, addr, 8);                                           //Store the addess in addr15
            intSensorsPassed++;
            Serial.print ("Assigned to " + String(intSensorsPassed - 1) + ": "); printAddress(addr15); Serial.println();
            Serial.println ("Sensors passed = " + String(intSensorsPassed) + "\n");
            Serial.println("Add sensor  " + String(intSensorsPassed + 1));
          }                                                                     //End if the new address obtained is the same as the last iteration of intSensorsPassed
          break;

      }                                                                         // End switch case
    }                                                                           // End while sensors Passed
    delay (500);
  } while (intSensorsPassed < eeprom_object.num_ds18b20);                       //This would hang up the program until this number is passed...
                                                                                //Store the detected addresses in EEPROM
  for (int z = 0; z < 8; z++) {
    eeprom_object.ds18b20_sensor0[z] = addr0[z];
    eeprom_object.ds18b20_sensor1[z] = addr1[z];
    eeprom_object.ds18b20_sensor2[z] = addr2[z];
    eeprom_object.ds18b20_sensor3[z] = addr3[z];
    eeprom_object.ds18b20_sensor4[z] = addr4[z];
    eeprom_object.ds18b20_sensor5[z] = addr5[z];
    eeprom_object.ds18b20_sensor6[z] = addr6[z];
    eeprom_object.ds18b20_sensor7[z] = addr7[z];
    eeprom_object.ds18b20_sensor8[z] = addr8[z];
    eeprom_object.ds18b20_sensor9[z] = addr9[z];
    eeprom_object.ds18b20_sensor10[z] = addr10[z];
    eeprom_object.ds18b20_sensor11[z] = addr11[z];
    eeprom_object.ds18b20_sensor12[z] = addr12[z];
    eeprom_object.ds18b20_sensor13[z] = addr13[z];
    eeprom_object.ds18b20_sensor14[z] = addr14[z];
    eeprom_object.ds18b20_sensor15[z] = addr15[z];
  }
  Serial.println(F("Iterative addressing of ds18b20 sensors complete."));
}

//-----Test Measurements function-------------------------------------------------------------
void test_measurements() {
  readRTC();                      //Read the RTC
  delay(50);
  Temp = getTemp();               //Read DS18B20 temperature sensors
  delay(50);
  read_watermark();               //Read the WaterMark Sensors
  delay(50);
  compile();                      //Format data from subroutines into one string for transmission
}

//____________________________________________________________________________________________________________________________________
//____________________________________________________________________________________________________________________________________
//____________________________________________________________________________________________________________________________________
