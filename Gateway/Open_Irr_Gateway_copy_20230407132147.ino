//-----License------------------------------------------------------------------------
/*
 * MIT License

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
/*
   Gateway/Reciever for Open_Irr soil water tension management system

   Purpose: Compile node data for storage on a sd card module. Gateway may initiate an irrigation
     event when a defined threshold in soil moisture tension is reached. Water management system will be toggleable and
     thresholds for irrigation, irrigation time, and time between irrigation events programable

   Note: Future expansion through integration of LTE cellular network is desired. the SIMCom SIM7500V is a verison carrier with B4 & B13 band LTE service.

   Much thanks to the ArduinoSoilH2O project <https://github.com/ArduinoSoilH2O>
   which was partial inspiration for the present system.



  Version History:
  
  2022.09.19 -> First Version of record. 

*/

//-----Some future considerations------------------------------------------------------
/*
   1  Troubleshoot prototype switchboard connection with gateway/battery pack
   2  menu call for displaying radio information
   3  Checking time with some standard?
   4  Sending time to Addressed nodes for update
   5  Error log file
*/
//-----Libraries---------------------------------------------------

#include<SD.h>                    //sd card functionality
#include<SPI.h>                   //SPI functions
#include<Wire.h>                  //I2C functions for rtc
#include<EEPROM.h>                //Built in EEPROM routines
#include"RTClib.h"                //for RTC functions
#include<DS3232RTC.h>             //Precision (generic) RTC library for DS3232/DS3231 rtc module, requires v 1.3 - do not update
#include <RH_RF95.h>              //Radio library for RF95 transciever 
#include <RHReliableDatagram.h>   //Radio library to simplify data sending/recieving
#include <RadioString.h>          //Custom Library to ease sending and receiving of large Strings
#include<LowPower.h>              //Low power functionality
#include <RHGenericDriver.h>      //Generic library
//#include <SoftwareSerial.h>     //for BT function, Nonfunctional 

//-----Assign Pins---------------------------------------------------
#define pin_mBatt 30                      //Digital pin to activate battery voltage measurement circuit
#define pin_battV 31                      //Pin for reading Voltage out from divider circuit to calculate Voltage in
#define ADC_REF_VOLTAGE 3.3               //The reference output of the moteino mega board
#define ADC_RESOLUTION  1024              //The analog digital converter in moteino mega is 10 bit resolution
#define ADC_MAXVALUE  1023                //1024 possibilities including 0 (0-1023) on 10 bit ADC 

#define LED 15                            //Pin for LED on Moteino Mega board
#define SD_CS 0                           //Chipselect pin for sdcard module
#define BTsleep 12                        //Bluetooth on/off switch | LOW = on, HIGH = off
#define BTstate 13                        //Bluetooth State (tells if on or off)
#define RTC_Interrupt 24                  //(A0) for RTC_SQW for interrupts

#define in1 19                            // The four pins on the Moteino-Mega that can be used as an low-level output 
#define in2 20
#define in3 21
#define in4 22

//-----Declare Variables---------------------------------------------

char    filename[] = "000_Data.txt";                               //sd card file name, 000 to be replaced by IDnum
char    filename2[] = "debug.txt";

char    data[RH_RF95_MAX_MESSAGE_LEN];                             //array to concatenate data arrays //dont think this is used? 2022-06-15
uint8_t int_buff[RH_RF95_MAX_MESSAGE_LEN];                         //integer type array buffer
uint8_t length_buff;                                               //integer denoting the size of the buffer
uint8_t from_node_id;                                              //integer specifying the node id that a transmission is coming from
uint16_t TRANSMIT_TIMEOUT = 2000;                                  //Set the minimum retransmit timeout. If sendtoWait is waiting for an ack longer than this time (in milliseconds), it will retransmit the message. Defaults to 200ms.
uint8_t TRANSMIT_POWER = 20;                                       //Set transmission power, 0-20,  defaults to 14
float RADIO_FREQUENCY = 915.0;                                     //Set frequency of radio module
uint16_t ROUTINE_TIMEOUT = 2000;                                   //Set delay at the end of the routine, too low and some packets will be missed, too high and you may also miss packets
uint8_t RETRY_NUM = 3;                                             //Set number of retry attempts for a single packet
uint8_t RECEIVE_ID;                                                //Holder to place identity of received transmission
uint8_t PACKET_NUM;                                                //To hold number of packets expected
uint8_t PACKET_COUNT;                                              //To count/increment on number of packets expected

uint8_t last_node_id = 0;                                          //integer soecifying the node the last transmission came from
String   all_data;                                                 //for incoming data string
unsigned long incomplete_transmission_count = 0;                   //to keep track of the number of incomplete transmissions recieved, but not parsed, to the data file. Nonfunctional

bool track_transmissions_G1 = false;                               //To track if a complete set of expected transmissions have been received. see void gateway_water_management_action()
bool track_transmissions_G2 = false;
bool track_transmissions_G3 = false;
bool track_transmissions_G4 = false;

int gateway_group1_mean;                                           //Integer to hold gateway group means. groups 1 to 4 correspond to output pins 1 to 4, respectively.
int gateway_group2_mean;
int gateway_group3_mean;
int gateway_group4_mean;

uint32_t last_irr_time_unix_epoch_group1;                          //save the time as unix epoch time from RTC for the groups.
uint32_t last_irr_time_unix_epoch_group2;
uint32_t last_irr_time_unix_epoch_group3;
uint32_t last_irr_time_unix_epoch_group4;

byte secs;                                                         //time and date values
byte mins;
byte hrs;
byte dow;
byte days;
byte mnths;
int yrs;
byte alarm_1_Mins;
tmElements_t tm;

unsigned long startMillis_1;                                       //Holders for timing of events without delay(). Need one per water management setting.
unsigned long startMillis_2;
unsigned long startMillis_3;
unsigned long startMillis_4;
unsigned long currentMillis;

bool irr_indicator_1 = false;                                      //for flagging when an irrigation event is active. Start false.
bool irr_indicator_2 = false;
bool irr_indicator_3 = false;
bool irr_indicator_4 = false;


char serial_inc;                                                   //store incoming value from serial

char a[4];                                                         //char array for itoa function
byte i;                                                            //for-loop counter

float battV;                                                       //battery voltage
float lowBatt = 4.0;                                               //low-battery limit
bool battLow = false;                                              //tracks if battery voltage falls below specified threshold

boolean isMenuOn = false;
int menuinput;                                                     //user input to menu prompt
long menutimeout;                                                  //length of time to wait for user input
long submenutimeout;
int indata;                                                        //user input data
int input;
int numincoming;
int incoming[7];
boolean firstTime = true;                                          //flag for first time

const byte numChars = 32;                                          //for reading in character user input
char charInput[200];                                               //for charinput function
char incomingChar[200];

//EEPROM structure method for storing and retrieving values from eeprom: relevant for variables defined in the menu
// Note that each EEPROM position can save only one byte of information, i.e. 8-bit numbers 0-255 and leading values (001) ARE NOT INTERPRETABLE
// but more than one position can be accessed using eeprom.put and eeprom.get with a defined structure
// this also enables saving different data types together
//ATMEL EEPROM LIFESPAN ~ 100,000 read/writes
//ATMEL FLASH LIFESPAN ~ 10,000 read/writes

int eeprom_address = 0;                                //Start storing to eeprom at position 0

struct eeprom_struct {

  uint8_t IDnum;                                       //numeric board identifier 0-255
  uint8_t gatewayID;                                   //gatewayID for radio networking, we will always use 1 for the gatewayID and number nodes thereafter 2-X
  char projectID[numChars] = {0};                      //Project identifier

  boolean firstTime = true;                            //flag for first time for writing to sdcard module

  int ALARM_1_Interval = 1;                            //Set the interval for alarm 1 (wake and listen routine), minutes

  boolean bluetooth_on = false;

  boolean is_water_manager_on;                         //1=true 0=false
  
  int water_threshold_1;                               //for defining water threshold for each group, 0-255, integer type
  int water_threshold_2;                 
  int water_threshold_3;                 
  int water_threshold_4;                  

  long min_time_btwn_irr;                              //for defining minimum time between irrigation events, minutes
  
  long irr_period_1;                                   //for defining the duration of an irrigation event, minutes
  long irr_period_2;                     
  long irr_period_3;                     
  long irr_period_4;                     

                                                       //Permit and Exclusion windows to allow/disallow watering events via unix time? Nonfunctional
  uint32_t exclusion_window_start_1;
  uint32_t exclusion_window_start_2;
  uint32_t exclusion_window_start_3;
  uint32_t exclusion_window_start_4;

  uint32_t exclusion_window_end_1;
  uint32_t exclusion_window_end_2;
  uint32_t exclusion_window_end_3;
  uint32_t exclusion_window_end_4;

  uint32_t permit_window_start_1;
  uint32_t permit_window_start_2;
  uint32_t permit_window_start_3;
  uint32_t permit_window_start_4;

  uint32_t permit_window_end_1;
  uint32_t permit_window_end_2;
  uint32_t permit_window_end_3;
  uint32_t permit_window_end_4;

  int nodes_group_1[16] {0};                           //Array to hold NodeIDs to consider for water management routine
  int nodes_group_2[16] {0};
  int nodes_group_3[16] {0};
  int nodes_group_4[16] {0};

  int irr_group_1[64] {0};                             //Array to hold Node groups considered in water management routine
  int irr_group_2[64] {0};
  int irr_group_3[64] {0};
  int irr_group_4[64] {0};

  int received_nodes_group_1[16] {0};                  //Array to hold NodeID's that we have received a transmission from and are part of a water management group
  int received_nodes_group_2[16] {0};
  int received_nodes_group_3[16] {0};
  int received_nodes_group_4[16] {0};

  int received_irr_group_1[64] {0};                    //Array to hold Node Groups values considered in the water mangement routine
  int received_irr_group_2[64] {0};
  int received_irr_group_3[64] {0};
  int received_irr_group_4[64] {0};
};
eeprom_struct eeprom_object = {};                      //Declare an object with the eeprom_struct structure, access objects as eeprom_object."element of struct without quotes"


//-----Initialize----------------------------------------------------

RTC_DS3231 rtc;                                                 //Specify real time clock

RH_RF95 driver(4, 2);                                           //Specify radio driver, explicit call to driver pins req?? see <https://lowpowerlab.com/guide/moteino/lora-support/>
RadioString manager(driver, eeprom_object.gatewayID);           //Setup Radio manager

File myfile;                                                    //make sdcard file object
File myfile2;                                                   //make second sdcard file object??
//FlashTools flash;                                             //initialize flash memory
//SoftwareSerial bluetooth(10, 11);                             //corresponds to Rx|Tx of moteino where bt module is connected, Nonfunctional

//_______________________________________________________________________________________________________________________________
void setup() {
  Serial.begin(9600);
  delay(100);
  Wire.begin();                                                 //enable I2C bus for rtc
  delay(300);

  pinMode(LED, OUTPUT);                                         
  pinMode(SD_CS, OUTPUT);                                       //set sd card ChipSelect pin as output
  //pinMode(SD_CD, INPUT);                                      //sd CD pin, has lead to fried sdcards? DO NOT UTILIZE

  pinMode(in1, OUTPUT);                                         //declare relay pins as outputs
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);                                       //Four channel relay we want normally lOW and activated when HIGH. Bridge the COM & HIGH pins on the relay with provided jumper. Provide Vcc & GND from 5v wall plug. connect microcontroller pins to relay switches. Connect common ground between microcontroller and 5v wall plug.
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  pinMode(BTsleep, OUTPUT);
  pinMode(BTstate, INPUT);

  pinMode(pin_mBatt, OUTPUT);
  digitalWrite(pin_mBatt, LOW);                                 //Leave the battery voltage circuit open to avoid battery drain
  pinMode(pin_battV, INPUT);                                    //to read voltage

  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC, routine hang up"));
    while (1);
  }

  if (! SD.begin(SD_CS)) {
    Serial.println(F("SD card not present or card failure."));
  }; 

  Serial.println(F("Initialization Completed"));

  battV = calcbattV();                                          //Get board battery level on startup

  //----- check chip EEPROM for stored data and place into "eeprom_object" with the structure of "eeprom_struct" declared earlier
  
  EEPROM.get(eeprom_address, eeprom_object);                    //eeprom_address may be redundant if only writing one eeprom object (i.e. it would always begin at position 0)

  delay(10);
  driver.setFrequency(915.0);
  delay(10);
  // driver.ModemConfigChoice(Bw125Cr45Sf128);
  // delay(10);
  driver.setTxPower(20, false);
  delay(10);
  manager.setThisAddress(eeprom_object.gatewayID);              //Set the current GATEWAY address for radio communication
  delay(10);
  manager.setTimeout(10000);                                    //set timeout period where if an ACK not recieved it will retransmit message Default is 200ms
  delay(1000);
  menu();                                                       //launch menu routine
}
//__________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

void loop() {
  //---Real loop----------------------
  currentMillis = millis();                                     //get the current "time" (actually the number of milliseconds since the program started)
  if (eeprom_object.bluetooth_on) {                             //***Bluetooth nonfunctional
    bt_on_loop();                                               //Loop for bluetooth ON.
    readRTC();                                                  //Read the DS3231 RTC
    delay(50);
    Serial.println(F("Looking for Nodes addressed to this gatewayID..."));
    Serial.print(F("Current gatewayID is: "));
    Serial.println(eeprom_object.gatewayID);
    battV = calcbattV();                                        //check battery voltage with subroutine
    Serial.print(F("Gateway Battery Pack Voltage:  "));
    Serial.println(battV);

    if (!manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.gatewayID)) {
      Serial.println(F("Radio Failed"));
    }
    if (manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.gatewayID)) {
      Serial.println(F("Radio initilization successful."));
    }
    manager.Receive_String(all_data, TRANSMIT_TIMEOUT, ROUTINE_TIMEOUT, RECEIVE_ID);
                                                                //When the Receive_String function has parsed all packets of an intended message,the Received_String contains the message.
    if (all_data != "") {
      Serial.print("Received_String:  ");
      Serial.println(all_data);
      storeData();                                              //Call the storeData function if there is data to store
    }           
    clearBuffer();                                              //Reset the Received_String for next transmission.
    Serial.println(F("Loop complete."));
    check_menu();
    
  } else {
    //---- loop with constant read, No Bluetooth function / Bluetooth off

    readRTC();                                                  //Read the DS3231 RTC
    delay(50);
    Serial.println(F("Looking for Nodes addressed to this gatewayID..."));
    Serial.print(F("Current gatewayID is: "));
    Serial.println(eeprom_object.gatewayID);

    battV = calcbattV();                                        //check battery voltage with subroutine
    Serial.print(F("Gateway Battery Pack Voltage:  "));
    Serial.println(battV);

    if (!manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.gatewayID)) {
      Serial.println(F("Radio Failed"));
    }
    if (manager.init(RADIO_FREQUENCY, TRANSMIT_TIMEOUT, TRANSMIT_POWER, RETRY_NUM, eeprom_object.gatewayID)) {
      Serial.println(F("Radio initilization successful."));
    }
    manager.Receive_String(all_data, TRANSMIT_TIMEOUT, ROUTINE_TIMEOUT, RECEIVE_ID);
                                                                //When the Receive_String function has parsed all packets of an intended message,the Received_String contains the message.
    if (all_data != "") {
      Serial.print("Received_String:  ");
      Serial.println(all_data);
      storeData();                                              //Call the storeData function if there is data to store
    }
    gateway_water_management_action();                          //water management action routine, see function.
    clearBuffer();                                              //Reset the Received_String for next transmission.
    // Serial.print(F("Incomplete transmission count:  "));     //report the count of incomplete transmissions.. Note this would need added to EEPROM to pull it from memory instead of resetting everytime you plug into the gateway...
    // Serial.println(incomplete_transmission_count);
    Serial.println(F("Loop complete."));
    check_menu();
  }
}

//___________________________________________________________________________________________________________________________________
//-----Other Defined Functions------
//___________________________________________________________________________________________________________________________________

//-----storeData Routine--------------------------------------------------

void storeData() {

  if (! SD.begin(SD_CS)) {                                                      //check if sdcard is inserted, if not dont do anything
    Serial.println(F("SD card not present or card failure."));
  };
  battV = calcbattV();                                                          //check battery voltage with subroutine
  myfile = SD.open(filename, FILE_WRITE);                                       //open data file
  delay(10);
  if (myfile) {                                                                 //if open
    if (eeprom_object.firstTime == true) {                                      //and it is the first time the file was opened, then print the expected data format that will be written to sd card
      myfile.println();
      myfile.println(F("Open_Irr: soil water tension management system"));
      myfile.print(F("Project ID: "));
      myfile.println(eeprom_object.projectID);
      myfile.println(F("Data values are stored as follows:"));
      myfile.println(F("GatewayID, Gateway battery voltage (V), Node ID, Node battery voltage (V), DateTime, Node data string"));
      myfile.println(F("Node data strings have the follwing format:"));
      myfile.println(F("Following DateTime the mux channel position and sensor value (CB/kpa) for all watermark sensors installed are printed (Channel, Value, Channel, Value)"));
      myfile.println(F("Following the raw watermark sensor data, the water management group means are reported as (G(1-4), Mean, DateTime of last irrigation)"));
      myfile.println(F("After watermark group means, DS18b20 temperature sensor data is reported as (T(# of sensor), Value (C))"));
      myfile.println();
      eeprom_object.firstTime = false;                                          //Only include this header when first writing to file
      eeprom_address = 0;                                                       //clear eeprom_address
      EEPROM.put(eeprom_address, eeprom_object);                                //Store the new firsttime flag to eeprom
      eeprom_address = 0;
    }
    myfile.print(F("GatewayID: "));                                            //Print information from gateway
    myfile.print(eeprom_object.gatewayID);
    myfile.print(',');
    myfile.print(F("Gateway battV: "));
    if (battV <= lowBatt) {
      myfile.print(F("Low Battery Voltage!: "));                                //flag to indicate low battery voltage
    }
    myfile.print(battV);
    myfile.print(',');

    all_data.remove(all_data.length() - 1, 1);                                  //Remove the character that indicated the end of a transmission "]"
    all_data.remove(0, 4);                                                      //Remove the Transmission information "[~~~". Zero indexed

    myfile.println(all_data);                                                   //print the all_data string
    delay(100);                                                                 //allow time to write data
    myfile.close();
    delay(200);                                                                 //allow time for file to close
    Serial.print(F("The data string written:  "));
    Serial.println(all_data);
    Serial.println(F("Data Written Successfully."));
  } else {
    Serial.println(F("Error opening file."));                                   //cannot find "myfile"
  }
}

//------Calc battery voltage (0- ~ 10V) with voltage divider circuit-------------
float calcbattV() {
  pinMode(pin_mBatt, OUTPUT);                                                   //Set digital pin as an output to activate the circuit
  pinMode(pin_battV, INPUT);                                                    //Set the other pin as input to read the voltage

  digitalWrite(pin_mBatt, HIGH);                                                //Turn the gate of the MOSFET HIGH, closing the circuit
  delayMicroseconds(500);                                                       //delay time for stabilization

  uint16_t vInt;                                                                //The raw integer returned by analogRead
  vInt = analogRead(pin_battV);                                                 //read vInt on analog pin connected where the resistors in the divider circuit meet.
  digitalWrite(pin_mBatt, LOW);                                                 //Turn the gate of the MOSFET LOW, deactivating the circuit

  float vout = 0.00;
  float vin = 0.00;
  float R1 = 10000;                                                             //1st Resistor in divider circuit taking load from battery
  float R2 = 4700;                                                              //2nd Resistor in divider circuit leading to board GND

  vout = (vInt * ADC_REF_VOLTAGE) / ADC_RESOLUTION;                             //voltage modified by divider circuit, voltage at dividing point
  vin = vout / (R2 / (R1 + R2));                                                //Actual voltage coming into the divider circuit, i.e. battery level
  return vin;
}

//---------- Clear buffer---------------
void clearBuffer() {
  all_data = "";
  delay(100);
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

//-----Check Temp of DS3231 RTC-------------------------------------------
float rtc_temp() {
  float rtc_temperature;
  rtc_temperature = ((float)RTC.temperature() / 4.0);
  return rtc_temperature;
}

//-----Menu Routine-------------------------------------------------------

void menu() {

  if (Serial.available() > 0) {
    Serial.read();                                                              //clear serial input buffer
  }

  itoa(eeprom_object.IDnum, a, 10);                                             //convert IDnum to character array


  if (eeprom_object.IDnum < 10) {                                               //for naming filename
    filename[0] = '0';                                                          //put into filename[] array
    filename[1] = '0';
    filename[2] = a[0];
  } else if (eeprom_object.IDnum < 100) {
    filename[0] = '0';                                                          //put into filename[] array
    filename[1] = a[0];
    filename[2] = a[1];
  } else {
    filename[0] = a[0];                                                         //put into filename[] array
    filename[1] = a[1];
    filename[2] = a[2];
  }

  Serial.println();                                                             //print out board information
  Serial.println(F("Open_Irr: soil water tension management system"));
  Serial.print(F("Project ID:  "));                                             
  Serial.println(eeprom_object.projectID);
  delay(10);
  Serial.print(F("Board ID: "));
  Serial.println(eeprom_object.IDnum);
  delay(10);
  Serial.print(F("Gatway ID:  "));
  Serial.println(eeprom_object.gatewayID);
  delay(10);
  Serial.print(F("Gateway Temperature:  "));
  float temp_rtc;
  temp_rtc = rtc_temp();
  Serial.println(temp_rtc);

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

  Serial.print(F("Gateway Temperature:  "));
  Serial.print(rtc_temp());
  Serial.println(F("  C."));

  Serial.print(F("Bluetooth Mode is "));
  if (eeprom_object.bluetooth_on) {
    Serial.println(F("ON."));
  } else {
    Serial.println(F("OFF."));
  }

  display_water_management_settings();                                                    //Displaying these settings is its own function as it is large...

  Serial.println(F("Menu Actions:"));                                                     //Define menu Actions--------------------------------------------
  Serial.println(F("   b  <--  Toggle Bluetooth Serial"));                                // "98" Toggle node bluetooth mode
  Serial.println(F("   c  <--  Set clock"));                                              // "99" Set RTC clock
  Serial.println(F("   w  <--  Set Gateway water management routine"));                   // "119" Specify information relevant to low_level output for irrigation events
  Serial.println(F("   s  <--  Switch Water Manager"));                                   // "115" Switch Gateway Water Manager routine
  Serial.println(F("   i  <--  Set ID numbers"));                                         // "105" set ProjectID IDnum, nodeID, gatewayID
  Serial.println(F("   f  <--  Display microSD card information"));                       // "102" Get sdcard information
  Serial.println(F("   d  <--  Download all data"));                                      // "100" get all data
  Serial.println(F("   e  <--  Erase all data"));                                         // "101" erase all data
  Serial.println(F("   l  <--  Display LoRa radio settings"));                            // "l08" display LoRa radio information
  Serial.println(F("   x  <--  Exit"));                                                   // "120" exit, turn off Bluetooth
  Serial.flush();

  menutimeout = millis() + 10000;                                                         //wait for user input
  while (millis() < menutimeout) {

    menuinput = 120;                                                                      //will default to ascii 120 "x"
    if (Serial.available() > 0) {
      menuinput = Serial.read();                                                          //get user input
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }


  switch (menuinput) {                                                                    //switch cases for menu input, Note the case# corresponds to input in ASCII format

    case 119:                                                                             //"w" for defining water manager settings------------------------
      Serial.println(F("Defining Water Management Settings."));
      Serial.println(F("NOTE: All Nodes considered for a water management setting must be on the same transmission frequency."));
      get_integer_input();                                                                //otherwise the first input is always 0?
      Serial.println();
      if (!eeprom_object.is_water_manager_on) {
        Serial.println(F("Gateway water management is currently not on."));
        Serial.println(F("Enable Gateway water management using menu option 's'."));
        Serial.println(F("Enter 1 to proceed with specifying Gateway water management settings."));
        get_integer_input();
        if (indata != 1) {
          menu();
          break;
        }
      }
      Serial.println(F("Enter a pinout (water management setting)to configure. (1,2,3,4)"));
      Serial.println(F("To return to the main menu enter '0'. "));
      get_integer_input();
      int pinout_num;
      pinout_num = indata;
      if (pinout_num == 0) {
        Serial.println(F("Returning to main menu..."));
        menu();
        break;
      } else {
        gateway_water_management_config(pinout_num);                                       //has its own void function.
      }
      menu();
      break;

    case 115:                                                                              //"s" for switch water_manager----------------------------------
      Serial.println(F("Switch Gateway Water Manager: "));
      get_integer_input();                                                                 //otherwise the first input is always 0?
      Serial.println();
      Serial.println(F("Enable Gateway Water Manager?"));
      Serial.print(F("Current Status: "));
      Serial.println(eeprom_object.is_water_manager_on);                                   //returns 0 for false and 1 for true
      get_integer_input();
      eeprom_object.is_water_manager_on = indata;
      delay(20);
      menu();
      break;

    case 98:                                                                               //"b" for Toggle Bluetooth mode---------------------------------- Bluetooth Nonfunctional at this time

      Serial.println(F("Toggle Bluetooth Mode."));
      get_integer_input();                                                                 //otherwise the first input is always 0?
      Serial.print(F("Input 1 to turn Bluetooth Mode ON, Input 0 to turn Bluetooth Mode OFF."));
      get_integer_input();
      int bt_mode;
      bt_mode = indata;
      if (bt_mode == 1) {
        eeprom_object.bluetooth_on = true;
        Serial.println(F("Bluetooth Mode turned ON."));
        delay(1000);
      } else if (bt_mode == 0) {
        eeprom_object.bluetooth_on = false;
        Serial.println(F("Bluetooth Mode turned OFF."));
        delay(1000);
      } else {
        Serial.println(F("Invalid Input."));
      }

      menu();
      break;

    case 108:                                                                               //"l" for Display LoRa radio information--------------------------
      Serial.println(F("Display all LoRa Settings."));
      driver.printRegisters();                                                              //needs registers parsed to be user friendly
      menu();
      break;
    case 99:                                                                                //"c" for set clock-----------------------------------------------

      Serial.println(F("Set clock:  "));
      get_integer_input();                                                                  //otherwise the first input is always 0?
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


    case 102:                                                                               //"f" for checking files on sdcard...
      Serial.println(F("Getting sdcard information..."));
      sdCheck();
      menu();
      break;



    case 105:                                                                               //"i" for set ID numbers-----------------------------------------

      Serial.println(F("Set network ID numbers (ProjectID, BoardID, and GatewayID):"));     //set ProjectID, BoardIDnum, and GatewayID numbers
      get_integer_input();                                                                  //otherwise the first input is always 0?
      Serial.flush();
      Serial.print(F(" Project ID:     "));                                                 //get projectID
      Serial.flush();
      charinput();
      if (charInput[0]) {
        byte i = 0;
        int count;                                                                          //will allow tracking where the name ends and can be appended
        for (int i = 0; i < numChars; i++) {
          if (charInput[i] != 0) {
            eeprom_object.projectID[i] = charInput[i];
            count++;
          } else {
            break;
          }
        }                                                            
        for (int strt = count + 1; strt <= numChars; strt++) {                              //append the rest of the char array with filler to force out old name remnants
          eeprom_object.projectID[strt] = '0';
        }
      }

      Serial.print(F(" Board ID:     "));                                                   //get BoardID
      Serial.flush();
      get_integer_input();                                                                  //decode user input
      eeprom_object.IDnum = indata;

      Serial.print((" Gateway ID:  "));                                                     //get GatewayID
      Serial.flush();
      get_integer_input();                                                                  //decode user input
      eeprom_object.gatewayID = indata;

      manager.setThisAddress(eeprom_object.gatewayID);                                      //set the gatewayID for radio manager

      menu();                                                                               //go back to menu
      break;

    case 100:                                                                               //"d" for Download all data in sd card (prints to serial port)---------------------------------------------------------------
      Serial.println(F("Download all data:"));
      delay(100);

      myfile = SD.open(filename);                                                           //open file
      if (myfile) {
        while (myfile.available()) {                                                        //read file and print to screen Serial COM port
          Serial.write(myfile.read());
        }
        myfile.close();
      } else {
        Serial.println(F("Error opening file"));
      }
      delay(10000);                                                                         //Give time for user to copy data...
      menu();
      break;

    case 101:                                                                               //"e" for erase data on sd card-------------------------------------------
      Serial.println(F("Erase data on sd card..."));
      Serial.print(F("Currently Writing to: "));
      Serial.println(filename);
      Serial.print(F("Do you want to delete: "));
      Serial.print(filename);
      Serial.println(F("? Type YES to confirm deletion of this file."));
      get_integer_input();                                                                  //otherwise the first input is always 0?
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
          eeprom_object.firstTime = true;                                                   //Return firsttime to true so header is printed when first writing to file
        } else {
          Serial.println(F("If condition not satisfied"));
        }
      }
      menu();
      break;

    case 120:                                                                               //"x" for Exit ---------------------------------------------------
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;
      
    default:                                                                                //Define default case, here exit if no valid user input, leave menu
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;

  }
  eeprom_address = 0;                                                                       //clear eeprom_address
  EEPROM.put(eeprom_address, eeprom_object);                                                //store new settings (eeprom_object with structure eeprom_struct) to chip EEPROM
  eeprom_address = 0;                                                                       //clear eeprom_address
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

//-----Get User input-----------------------------------------------
void get_integer_input() {
  menutimeout = millis() + 10000;                                               //time to wait for user to input something
  indata = 0;                                                                   //initialize
  while (millis() < menutimeout)                                                //wait for user to input something
  {
    if (Serial.available() > 0)                                                 //something came in to serial buffer
    {
      delay(100);                                                               //give time for everything to come in
      numincoming = Serial.available();                                         //number of incoming bytes
      for (i = 1; i <= numincoming; i++)                                        //read in everything
      {
        incoming[i] = Serial.read();                                            //read from buffer
        if (incoming[i] == 13 || incoming[i] == 10)                             //ignore carriage return & line feed
        {
        }
        else                                                                    //otherwise
        {
          input = incoming[i] - 48;                                             //convert ASCII value to decimal
          indata = indata * 10 + input;                                         //assemble to get total value
        }
      }
      break;                                                                    //exit before menutimeout
    }
  }
  Serial.println(indata); delay(10);
}

//-----Get User Input for Character variables etc.-----------------------------------
void charinput() {
  memset(charInput, 0, sizeof(charInput));
  delay(50);
  long timeout;
  timeout = millis() + 10000;                                                   //length of time to wait for user input
  byte numincoming;                                                             //for # of bytes incoming
  while (millis() < timeout) {
    if (Serial.available() > 0) {
      delay(100);
      numincoming = Serial.available();
      for (byte i = 0; i <= numincoming; i++) {
        incomingChar[i] = Serial.read();
        if (incomingChar[i] == 13 || incomingChar == 10) {                      //ignore carriage return and linefeed
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

//------get nist time-------------------------------------- Non functional, intended to pull datetime from a standard online source
//void get_nist_time();

//-----time_sync------------------------------------------- Non functional, intended to standardize times of Nodes connected to a Gateway
//void time_sync();

//-----Set ALARM1 ---------------------------------                             // Alarm 1 is to wake and recieve data from nodes. 1/7/2022 Note that this is NOT functional for the gateway at this time. Continuous power must be provided to gateway as it is always listening
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
    alarm_1_Mins = (alarm_1_Mins % 60);                                         //if >=60, modulo 60 to get remainding minutes
  }
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, alarm_1_Mins, 0, 0);
  Serial.print(F("Alarm at: "));
  Serial.print(alarm_1_Mins);
  Serial.println(F("  minutes past the hour."));

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

//----- Bluetooth Mode ON Routine ------------------------------------- Nonfunctional
void bt_on_loop() {
  /*
     if(bluetooth.available()){
    Serial.println(F("BT available."));
    if(eeprom_object.bluetooth_on){
      Serial.write(bluetooth.read());       //From bluetooth to terminal
      if(Serial.available()){
        bluetooth.write(Serial.read());     //From terminal to bluetooth
      }
    } else {
      Serial.println(F("Bluetooth in EEPROM is OFF."));
    }
    }
  */
}

//-----Gateway water management settings-------------------------------

void gateway_water_management_config(int setting_num) {
  Serial.print(F("Configuring Gateway water management settings for pinout "));
  Serial.print(setting_num);
  Serial.println(F("."));
  delay(1000);

  Serial.println(F("Sub-Menu Actions:"));                                                      // Define Sub-menu Actions-----
  Serial.println(F("   T  <--  Set water management threshold levels"));                       // 84, set matric potential levels that will trigger an irrigation event
  Serial.println(F("   E  <--  Set irrigation Exclusion windows"));                            // 69, set timeframes where irrigation will not be triggered
  Serial.println(F("   P  <--  Set irrigation Permission windows"));                           // 80, set timeframe where irrigation can be triggered
  Serial.println(F("   G  <--  Set Node grouping details for water management groups"));       // 71, set which Nodes and groups within those nodes are considered for the pinout.
  Serial.println(F("   X  <--  Exit and return to main menu"));                                // 88, Exit and return to main menu
  Serial.println(F("   D  <--  Set the duration of irrigation events (in minutes)"));          // 68, set irr_period (in minutes)
  Serial.println(F("   M  <--  Set the minimum time between irrigation events (in minutes)")); // 77, set minimum time between irrigation events to permit an event to occur.

  submenutimeout = millis() + 10000;                                                           //wait for user input
  while (millis() < submenutimeout) {
    menuinput = 120;                                                                           //defaults to ascii 120 "x"
    if (Serial.available() > 0) {
      menuinput = Serial.read();                                                               //get user input
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }


  Serial.print(F("Please specify what to configure for water management setting "));
  Serial.println(setting_num);
  switch (menuinput) {

    case 77:                                                                                    //M
      get_integer_input();                                                                      //otherwise the first input is always 0?
      Serial.println(F("Setting minimum time between irrigation events."));
      Serial.println(F("This setting is shared among each water management setting."));
      Serial.print(F("Current value:  "));
      Serial.print(eeprom_object.min_time_btwn_irr);
      Serial.println(F(" minutes."));
      Serial.println(F("Specify a value in minutes."));
      get_integer_input();
      eeprom_object.min_time_btwn_irr = indata;
      menu();
      break;

    case 68:                                                                                    //D
      get_integer_input();                                                                      //otherwise the first input is always 0?
      Serial.print(F("Setting duration of irrigation events on water management setting "));
      Serial.println(setting_num);
      Serial.print(F("Current duration in minutes:  "));
      if (setting_num == 1) {
        Serial.println(eeprom_object.irr_period_1);
      }
      if (setting_num == 2) {
        Serial.println(eeprom_object.irr_period_2);
      }
      if (setting_num == 3) {
        Serial.println(eeprom_object.irr_period_3);
      }
      if (setting_num == 4) {
        Serial.println(eeprom_object.irr_period_4);
      }
      Serial.println(F("Please enter the desired duration in minutes"));
      get_integer_input();
      if (setting_num == 1) {
        eeprom_object.irr_period_1 = indata;
      }
      if (setting_num == 2) {
        eeprom_object.irr_period_2 = indata;
      }
      if (setting_num == 3) {
        eeprom_object.irr_period_3 = indata;
      }
      if (setting_num == 4) {
        eeprom_object.irr_period_4 = indata;
      }
      Serial.println(F("Returning to main menu..."));
      menu();
      break;

    case 84:                                                                                        //T
      get_integer_input();                                                                          //otherwise the first input is always 0?
      Serial.print(F("Setting matric potential threshold for trigger of irrigation events on water management setting "));
      Serial.println(setting_num);

      Serial.print(F("Current threshold:  "));
      if (setting_num == 1) {
        Serial.println(eeprom_object.water_threshold_1);
      }
      if (setting_num == 2) {
        Serial.println(eeprom_object.water_threshold_2);
      }
      if (setting_num == 3) {
        Serial.println(eeprom_object.water_threshold_3);
      }
      if (setting_num == 4) {
        Serial.println(eeprom_object.water_threshold_4);
      }

      Serial.println(F("Please enter the desired threshold"));
      get_integer_input();
      if (setting_num == 1) {
        eeprom_object.water_threshold_1 = indata;
      }
      if (setting_num == 2) {
        eeprom_object.water_threshold_2 = indata;
      }
      if (setting_num == 3) {
        eeprom_object.water_threshold_3 = indata;
      }
      if (setting_num == 4) {
        eeprom_object.water_threshold_4 = indata;
      }

      Serial.println(F("Returning to main menu..."));
      menu();
      break;

    case 69:                                                                                         //E
      Serial.print(F("Setting Exclusion windows for irrigation events on setting "));
      Serial.println(setting_num);
      Serial.println(("Function not currently supported. Check for the latest firmware on https://github.com/andrewbierer/Open_Irr "));
      menu();
      break;
    case 80:                                                                                         //P
      Serial.print(F("Setting Permission windows for irrigation events on setting "));
      Serial.println(setting_num);
      Serial.println(("Function not currently supported. Check for the latest firmware on https://github.com/andrewbierer/Open_Irr "));
      menu();
      break;
    case 71:                                                                                         //G
      get_integer_input();                                                                           //otherwise the first input is always 0?
      Serial.print(F("Setting the Nodes and Groups considered for Gateway setting "));
      Serial.println(setting_num);
      Serial.println(F("Note that a value of '0' indicates nothing has been specified."));
                                                                                                     //Display current settings
      Serial.print(F("Current settings for water management group "));
      Serial.print(setting_num);
      Serial.println(F(": "));
      if (setting_num == 1) {
        Serial.print(F("Nodes and groups considered: "));
        for (int j = 0; j < 16; j++) {
          Serial.print(F("Node: "));
          Serial.print(eeprom_object.nodes_group_1[j]);
          Serial.print(F(" Groups: "));
          for (int k = 0; k < 4; k++) {
            Serial.print(eeprom_object.irr_group_1[(k + 1) * j]);                                    //formula should be correct.
            Serial.print(',');
          }
          Serial.println();
        }
      }

      if (setting_num == 2) {
        Serial.print(F("Nodes and groups considered: "));
        for (int j = 0; j < 16; j++) {
          Serial.print(F("Node: "));
          Serial.print(eeprom_object.nodes_group_2[j]);
          Serial.print(F(" Groups: "));
          for (int k = 0; k < 4; k++) {
            Serial.print(eeprom_object.irr_group_2[(k + 1) * j]);                                    //formula should be correct.
            Serial.print(',');
          }
          Serial.println();
        }
      }

      if (setting_num == 3) {
        Serial.print(F("Nodes and groups considered: "));
        for (int j = 0; j < 16; j++) {
          Serial.print(F("Node: "));
          Serial.print(eeprom_object.nodes_group_3[j]);
          Serial.print(F(" Groups: "));
          for (int k = 0; k < 4; k++) {
            Serial.print(eeprom_object.irr_group_3[(k + 1) * j]);                                    //formula should be correct.
            Serial.print(',');
          }
          Serial.println();
        }
      }

      if (setting_num == 4) {
        Serial.print(F("Nodes and groups considered: "));
        for (int j = 0; j < 16; j++) {
          Serial.print(F("Node: "));
          Serial.print(eeprom_object.nodes_group_4[j]);
          Serial.print(F(" Groups: "));
          for (int k = 0; k < 4; k++) {
            Serial.print(eeprom_object.irr_group_4[(k + 1) * j]);                                    //formula should be correct.
            Serial.print(',');
          }
          Serial.println();
        }
      }
      delay(1000);
      //-----This is going to be the lonnnnnnng one...
      Serial.print(F("Clearing existing settings for pinout "));                                     //Clear the settings for the pinout being configured.
      Serial.print(setting_num);
      Serial.println(F("."));
      if (setting_num == 1) {
        for (int z = 0; z < 16; z++) {
          eeprom_object.nodes_group_1[z] = 0;
        }
        for (int y = 0; y < 64; y++) {
          eeprom_object.irr_group_1[y] = 0;
        }
      } else if (setting_num == 2) {
        for (int z = 0; z < 16; z++) {
          eeprom_object.nodes_group_2[z] = 0;
        }
        for (int y = 0; y < 64; y++) {
          eeprom_object.irr_group_2[y] = 0;
        }
      } else if (setting_num == 3) {
        for (int z = 0; z < 16; z++) {
          eeprom_object.nodes_group_3[z] = 0;
        }
        for (int y = 0; y < 64; y++) {
          eeprom_object.irr_group_3[y] = 0;
        }
      } else if (setting_num == 4) {
        for (int z = 0; z < 16; z++) {
          eeprom_object.nodes_group_4[z] = 0;
        }
        for (int y = 0; y < 64; y++) {
          eeprom_object.irr_group_4[y] = 0;
        }
      } else {
        Serial.println(F("Invalid pinout number... Returning to main menu."));
        menu();
        break;
      }
      int num_nodes;
      Serial.print(F("How many different Nodes are to be considered for pinout "));                   //Specify the number of Nodes considered in the pinout
      Serial.print(setting_num);
      Serial.println(F("?"));

      get_integer_input();
      num_nodes = indata;

      Serial.print(num_nodes);
      Serial.print(F(" Nodes specifed for pinout "));
      Serial.print(setting_num);
      Serial.println(F("."));
      delay(500);

                                                                                                     //A for loop work for identifying Nodes/Groups to act on
      for (int q = 0; q < num_nodes; q++) {                                                          //For each Node considered in the pinout...
        int fun_input;
        int num_WM_groups;

        Serial.print(F("Specify Node number "));
        Serial.print(q + 1);
        Serial.print(F(" of "));
        Serial.print(num_nodes);
        Serial.print(F(" for pinout "));
        Serial.println(setting_num);
        get_integer_input();
        fun_input = indata;

        if (setting_num == 1) {
          eeprom_object.nodes_group_1[q] = fun_input;
          delay(1000);
          Serial.print(F("How many WM groups to average in Node "));
          Serial.print(fun_input);
          Serial.print(F(" for triggering of pinout "));
          Serial.print(setting_num);
          Serial.println(F("?"));
          get_integer_input();
          num_WM_groups = indata;

          for (int w = 0; w < num_WM_groups; w++) {                                                  //Define which Groups (1,2,3,4) to consider from that Node
            Serial.print(F("Define WM group "));
            Serial.print(w + 1);
            Serial.print(F(" of "));
            Serial.print(num_WM_groups);
            Serial.print(F(" from Node "));
            Serial.print(q + 1);
            Serial.println(F("."));
                                                                                                     //Store that information in eeprom_object.
            get_integer_input();
            eeprom_object.irr_group_1[((q + 1) * (4)) - (4 - w)] = indata;                           //formula is [((num_nodes+1)*4)-(4-w)] to find the location to store the WM group number for all possible Nodes.
          }
        } else if (setting_num == 2) {
          eeprom_object.nodes_group_2[q] = fun_input;
          delay(1000);
          Serial.print(F("How many WM groups to average in Node "));
          Serial.print(fun_input);
          Serial.print(F(" for triggering of pinout "));
          Serial.print(setting_num);
          Serial.println(F("?"));
          get_integer_input();
          num_WM_groups = indata;

          for (int w = 0; w < num_WM_groups; w++) {                                                  //Define which Groups (1,2,3,4) to consider from that Node
            Serial.print(F("Define WM group "));
            Serial.print(w + 1);
            Serial.print(F(" of "));
            Serial.print(num_WM_groups);
            Serial.print(F(" from Node "));
            Serial.print(q + 1);
            Serial.println(F("."));
                                                                                                     //Store that information in eeprom_object.
            get_integer_input();
            eeprom_object.irr_group_2[((q + 1) * (4)) - (4 - w)] = indata;                           //formula is [((num_nodes+1)*4)-(4-w)] to find the location to store the WM group number for all possible Nodes.
          }
        } else if (setting_num == 3) {
          eeprom_object.nodes_group_3[q] = fun_input;
          delay(1000);
          Serial.print(F("How many WM groups to average in Node "));
          Serial.print(fun_input);
          Serial.print(F(" for triggering of pinout "));
          Serial.print(setting_num);
          Serial.println(F("?"));
          get_integer_input();
          num_WM_groups = indata;

          for (int w = 0; w < num_WM_groups; w++) {                                                  //Define which Groups (1,2,3,4) to consider from that Node
            Serial.print(F("Define WM group "));
            Serial.print(w + 1);
            Serial.print(F(" of "));
            Serial.print(num_WM_groups);
            Serial.print(F(" from Node "));
            Serial.print(q + 1);
            Serial.println(F("."));
                                                                                                     //Store that information in eeprom_object.
            get_integer_input();
            eeprom_object.irr_group_3[((q + 1) * (4)) - (4 - w)] = indata;                           //formula is [((num_nodes+1)*4)-(4-w)] to find the location to store the WM group number for all possible Nodes.
          }
        } else {
          eeprom_object.nodes_group_4[q] = fun_input;
          delay(1000);
          Serial.print(F("How many WM groups to average in Node "));
          Serial.print(fun_input);
          Serial.print(F(" for triggering of pinout "));
          Serial.print(setting_num);
          Serial.println(F("?"));
          get_integer_input();
          num_WM_groups = indata;

          for (int w = 0; w < num_WM_groups; w++) {                                                  //Define which Groups (1,2,3,4) to consider from that Node
            Serial.print(F("Define WM group "));
            Serial.print(w + 1);
            Serial.print(F(" of "));
            Serial.print(num_WM_groups);
            Serial.print(F(" from Node "));
            Serial.print(q + 1);
            Serial.println(F("."));
                                                                                                     //Store that information in eeprom_object.
            get_integer_input();
            eeprom_object.irr_group_4[((q + 1) * (4)) - (4 - w)] = indata;                           //formula is [((num_nodes+1)*4)-(4-w)] to find the location to store the WM group number for all possible Nodes.
          }
        }
        fun_input = 0;
        num_WM_groups = 0;
      }
      //-----Done configuring.
      menu();
      break;

    case 88:                                                                                          //X
      Serial.println(F("Returning to main menu"));
      menu();
      break;

    default:                                                                                          //If no action is taken
      Serial.println(F("Returning to main menu"));
      menu();
      break;
  }
}

//-----Gateway water management action------------------------------
void gateway_water_management_action() {
  
  int pinout_considered;
  int start_loc;
  int end_loc;
  int m;
  int n;
  int count;
  int count2;

  bool compile_G1 = false;
  bool compile_G2 = false;
  bool compile_G3 = false;
  bool compile_G4 = false;

  int N_Nodes_G1;
  int N_Nodes_G2;
  int N_Nodes_G3;
  int N_Nodes_G4;

  int N_Groups_G1;
  int N_Groups_G2;
  int N_Groups_G3;
  int N_Groups_G4;

  for (int q = 0; q < 16; q++) {                                                    //How many Nodes are expected in each management group?
    if (eeprom_object.nodes_group_1[q] != 0) {
      N_Nodes_G1++;
    }
    if (eeprom_object.nodes_group_2[q] != 0) {
      N_Nodes_G2++;
    }
    if (eeprom_object.nodes_group_3[q] != 0) {
      N_Nodes_G3++;
    }
    if (eeprom_object.nodes_group_4[q] != 0) {
      N_Nodes_G4++;
    }
  }
                                                                                    //How many Groups are expected in each management group?
  for (int w = 0; w < 64; w++) {
    if (eeprom_object.irr_group_1[w] != 0) {
      N_Groups_G1++;
    }
    if (eeprom_object.irr_group_2[w] != 0) {
      N_Groups_G2++;
    }
    if (eeprom_object.irr_group_3[w] != 0) {
      N_Groups_G3++;
    }
    if (eeprom_object.irr_group_4[w] != 0) {
      N_Groups_G4++;
    }
  }
                                                                                    //The expected Nodes in each group are in eeprom_object.nodes_group_1 etc...
  if (all_data != "") {                                                             //A complete message was received and parsed. contains the start, stop and # packet indicators "[~~~" and "]"

    int relevant_Node;
    String fun_string;
    fun_string = all_data.substring(0, 3);                                          //substring the portion with the NodeID
    relevant_Node = fun_string.toInt();                                             //Cast to an integer

                                                                                    //Is it from a node we care about? Use a loop to check elements of array
    for (int element = 0; element < 16; element++) {
                                                                                    //Begin routine for group 1 setting.
      if (relevant_Node == eeprom_object.nodes_group_1[element]) {                  //If it is, add it to the eeprom_object.received_nodes_group
        for (int r = 15; r >= 0; r--) {                                             //shift the array right and write the integer to the 1st element
          if (r == 0) {
            eeprom_object.received_nodes_group_1[r] = relevant_Node;
          } else {
            eeprom_object.received_nodes_group_1[r] = eeprom_object.received_nodes_group_1[(r - 1)];
          }
        }
                                                                                    //And if we care about this Node, grab/identify which groups we need to pull from in the received string
        start_loc = element * 4;                                                    //the starting location in the eeprom_object.irr_group_1 | also the location to store the group integer in eeprom_object.received_irr_group_1
        end_loc = start_loc + 3;                                                    //the ending location in the eeprom_object.irr_group_1
                                                                                   
        for (int index_group_num = 1; index_group_num <= 4; index_group_num++) {    //Which element (as an index) from the start_loc to end_loc != 0? These should be the group numbers we need to pull from the received string.
          if (eeprom_object.irr_group_1[start_loc + index_group_num - 1] == index_group_num) {  //If these group numbers are not 0 (indicate group 1,2,3,4 need to be included in the average)
                                                                                                //This is the element number (as an index) to place the identified Group mean from that Node.
            int loc_to_store_group_mean;
            loc_to_store_group_mean = &eeprom_object.irr_group_1[start_loc + index_group_num - 1]; //the address (&) to store the mean
                                                                                                   //Grab the corresponding mean from the received data string. Can do this by searching the string for "G1,G2,G3,G4"
                                                                                                   //The characters between the two following commas contain the desired mean example: G2,-25,5/4/2022
            String s1 = "G";                                                                       //Makeing String s1 = G1 G2 G3 G4 in the for loop and if condition...
            s1 += index_group_num;
            int extract_start;
            int extract_end;
            int extracted_value;
            extract_start = all_data.indexOf(s1) + 3;                                              //Find the index of s1 in all_data, returns the index of 'G' so the data we want begins 3 places later in the index.
            extract_end = all_data.indexOf(',', extract_start);                                    //Find the index of the next comma.
            s1 = all_data.substring(extract_start, extract_end);                                   //extract the data and convert to integer. Use the String s1 and toInt()
            extracted_value = s1.toInt();                                                                      
            eeprom_object.received_irr_group_1[start_loc] = extracted_value;                       //Store this integer in the appropriate location (identified as start_loc) in eeprom_object.received_irr_group_1
            s1 = "";                                                                               //reset String
          }
        }
      } 
      //------End routine for group 1 setting.
      
      //------Begin routine for group 2 setting.
      if (relevant_Node == eeprom_object.nodes_group_2[element]) {                                 //If it is, add it to the eeprom_object.received_nodes_group
                                                                                                   //shift the array right and write the integer to the 1st element
        for (int r = 15; r >= 0; r--) {
          if (r == 0) {
            eeprom_object.received_nodes_group_2[r] = relevant_Node;
          } else {
            eeprom_object.received_nodes_group_2[r] = eeprom_object.received_nodes_group_2[(r - 1)];
          }
        }
                                                                                                   //And if we care about this Node, grab/identify which groups we need to pull from in the received string
        start_loc = element * 4;                                                                   //the strating location in the eeprom_object.irr_group_2 | also the location to store the group integer in eeprom_object.received_irr_group_2
        end_loc = start_loc + 3;                                                                   //the ending location in the eeprom_object.irr_group_2
        for (int index_group_num = 1; index_group_num <= 4; index_group_num++) {                   //Which element (as an index) from the start_loc to end_loc != 0? These should be the group numbers we need to pull from the received string.
          if (eeprom_object.irr_group_2[start_loc + index_group_num - 1] == index_group_num) {     //If these group numbers are not 0 (indicate group 1,2,3,4 need to be included in the average)
                                                                                                   //This is the element number (as an index) to place the identified Group mean from that Node.
            int loc_to_store_group_mean;
            loc_to_store_group_mean = &eeprom_object.irr_group_2[start_loc + index_group_num - 1]; //the address (&) to store the mean
                                                                                                   //Grab the corresponding mean from the received data string. Can do this by searching the string for "G1,G2,G3,G4"
                                                                                                   //The characters between the two following commas contain the desired mean example: G2,-25,5/4/2022

            String s1 = "G";                                                                       //Makeing String s1 = G1 G2 G3 G4 in the for loop and if condition...
            s1 += index_group_num;
            int extract_start;
            int extract_end;
            int extracted_value;
            extract_start = all_data.indexOf(s1) + 3;                                              //Find the index of s1 in all_data, returns the index of 'G' so the data we want begins 3 places later in the index.
            extract_end = all_data.indexOf(',', extract_start);                                    //Find the index of the next comma.
            s1 = all_data.substring(extract_start, extract_end);                                   //extract the data and convert to integer. Use the String s1 and toInt()
            extracted_value = s1.toInt();
            eeprom_object.received_irr_group_2[start_loc] = extracted_value;                       //Store this integer in the appropriate location (identified as start_loc) in eeprom_object.received_irr_group_1
            s1 = "";                                                                               //reset String
          }
        }
      } 
      //-----End routine for group 2 setting.

      //-----Begin routine for group 3 setting.
      if (relevant_Node == eeprom_object.nodes_group_3[element]) {                                 //If it is, add it to the eeprom_object.received_nodes_group
                                                                                                   //shift the array right and write the integer to the 1st element
        for (int r = 15; r >= 0; r--) {
          if (r == 0) {
            eeprom_object.received_nodes_group_3[r] = relevant_Node;
          } else {
            eeprom_object.received_nodes_group_3[r] = eeprom_object.received_nodes_group_3[(r - 1)];
          }
        }
                                                                                                   //And if we care about this Node, grab/identify which groups we need to pull from in the received string
        start_loc = element * 4;                                                                   //the strating location in the eeprom_object.irr_group_3 | also the location to store the group integer in eeprom_object.received_irr_group_3
        end_loc = start_loc + 3;                                                                   //the ending location in the eeprom_object.irr_group_3
                                                                                        
        for (int index_group_num = 1; index_group_num <= 4; index_group_num++) {                   //Which element (as an index) from the start_loc to end_loc != 0? These should be the group numbers we need to pull from the received string.
          if (eeprom_object.irr_group_3[start_loc + index_group_num - 1] == index_group_num) {     //If these group numbers are not 0 (indicate group 1,2,3,4 need to be included in the average)
                                                                                                   //This is the element number (as an index) to place the identified Group mean from that Node.
            int loc_to_store_group_mean;
            loc_to_store_group_mean = &eeprom_object.irr_group_3[start_loc + index_group_num - 1]; // the address (&) to store the mean
                                                                                                   //Grab the corresponding mean from the received data string. Can do this by searching the string for "G1,G2,G3,G4"
                                                                                                   //The characters between the two following commas contain the desired mean example: G2,-25,5/4/2022
            String s1 = "G";                                                                       //Makeing String s1 = G1 G2 G3 G4 in the for loop and if condition...
            s1 += index_group_num;
            int extract_start;
            int extract_end;
            int extracted_value;
            extract_start = all_data.indexOf(s1) + 3;                                              //Find the index of s1 in all_data, returns the index of 'G' so the data we want begins 3 places later in the index.
            extract_end = all_data.indexOf(',', extract_start);                                    //Find the index of the next comma.
            s1 = all_data.substring(extract_start, extract_end);                                   //extract the data and convert to integer. Use the String s1 and toInt()
            extracted_value = s1.toInt();                                                                                        
            eeprom_object.received_irr_group_3[start_loc] = extracted_value;                       //Store this integer in the appropriate location (identified as start_loc) in eeprom_object.received_irr_group_3
            s1 = "";                                                                               //reset String
          }
        }
      } 
      //-----End routine for group 3 setting.

      //-----Begin routine for group 4 setting.
      if (relevant_Node == eeprom_object.nodes_group_4[element]) {                                 //If it is, add it to the eeprom_object.received_nodes_group
                                                                                                   //shift the array right and write the integer to the 1st element
        for (int r = 15; r >= 0; r--) {
          if (r == 0) {
            eeprom_object.received_nodes_group_4[r] = relevant_Node;
          } else {
            eeprom_object.received_nodes_group_4[r] = eeprom_object.received_nodes_group_4[(r - 1)];
          }
        }
                                                                                                   //write that integer to the first element of the array that is not 0.
        for (int r = 0; r < 16; r++) {
          if (eeprom_object.received_nodes_group_4[r] != 0 && eeprom_object.received_nodes_group_4[r] != -1) {
            eeprom_object.received_nodes_group_4[r] = relevant_Node;
          }
        }
                                                                                                   //And if we care about this Node, grab/identify which groups we need to pull from in the received string
        start_loc = element * 4;                                                                   //the starting location in the eeprom_object.irr_group_4 | also the location to store the group integer in eeprom_object.received_irr_group_4
        end_loc = start_loc + 3;                                                                   //the ending location in the eeprom_object.irr_group_4
                                                              
        for (int index_group_num = 1; index_group_num <= 4; index_group_num++) {                   //Which element (as an index) from the start_loc to end_loc != 0? These should be the group numbers we need to pull from the received string.
          if (eeprom_object.irr_group_4[start_loc + index_group_num - 1] == index_group_num) {     //If these group numbers are not 0 (indicate group 1,2,3,4 need to be included in the average)
                                                                                                   //This is the element number (as an index) to place the identified Group mean from that Node.
            int loc_to_store_group_mean;
            loc_to_store_group_mean = &eeprom_object.irr_group_4[start_loc + index_group_num - 1]; //the address (&) to store the mean
                                                                                                   //Grab the corresponding mean from the received data string. Can do this by searching the string for "G1,G2,G3,G4"
                                                                                                   //The characters between the two following commas contain the desired mean example: G2,-25,5/4/2022
            String s1 = "G";                                                                       //Makeing String s1 = G1 G2 G3 G4 in the for loop and if condition...
            s1 += index_group_num;
            int extract_start;
            int extract_end;
            int extracted_value;
            extract_start = all_data.indexOf(s1) + 3;                                              //Find the index of s1 in all_data, returns the index of 'G' so the data we want begins 3 places later in the index.
            extract_end = all_data.indexOf(',', extract_start);                                    //Find the index of the next comma.
            s1 = all_data.substring(extract_start, extract_end);                                   //extract the data and convert to integer. Use the String s1 and toInt()
            extracted_value = s1.toInt();
            eeprom_object.received_irr_group_4[start_loc] = extracted_value;                       //Store this integer in the appropriate location (identified as start_loc) in eeprom_object.received_irr_group_1
            s1 = "";                                                                               //reset String
          }
        }
      } 
      //-----End routine for group 4 setting.
    }  
    //------End loop through 16 elements

                                                                                                   //Determine / Track if a message has been obtained from all nodes we care about.
                                                                                                   //This needs done for each management group.
                                                                                                   //use isSubset bool function -> Returns 1 if array2 is a subset of array1

    //-----Group 1
    pinout_considered = 1;
    m = sizeof(eeprom_object.nodes_group_1) / sizeof(eeprom_object.nodes_group_1[0]);
    n = sizeof(eeprom_object.received_nodes_group_1) / sizeof(eeprom_object.received_nodes_group_1[0]);

                                                                                                   //begin if statements
    if (isSubset(eeprom_object.nodes_group_1, eeprom_object.received_nodes_group_1, m, n)) {       //If the received node array contains all unique elements of the expected node array
      Serial.print(F("Transmissions from each expected Node have been received for pinout "));
      Serial.println(pinout_considered);
      compile_G1 = true;                                                                           //compile_G1
    }
    count = 0;
    for (int g = 0; g < 16; g++) {                                                                 //OR if the received node array contains 3 instances of any particular element of the expected node array
      for (int h = 0; h < 16; h++) {
        if (eeprom_object.received_nodes_group_1[h] == eeprom_object.nodes_group_1[g]) {
          if (eeprom_object.nodes_group_1[g] != 0 && eeprom_object.nodes_group_1[g] != -1) {
            count++;
          }
        }
        if (count >= 3) {
          compile_G1 = true;                                                                       //compile_G1
          Serial.print(F("Three instances of Node "));
          Serial.print(eeprom_object.received_nodes_group_1[h]);
          Serial.print(F(" have been received for water management group "));
          Serial.print(pinout_considered);
          Serial.println(F(" prompting the water manager."));
          Serial.print(F("Please troubleshoot transmission reliability for nodes connected to water management group "));
          Serial.println(pinout_considered);
          break;                                                                                   //break out of the current for loop
        }
      }
      count = 0;                                                                                   //reset count
    }

    count2 = 0;                                                                                    //OR if the received node array does not contain 0
    for (int  v = 0; v < 16; v++) {
      if (eeprom_object.received_nodes_group_1[v] != 0 && eeprom_object.received_nodes_group_1[v] != -1) {
        count2++;
      }
    }
    if (count2 >= 16) {
      compile_G1 = true;                                                                           //compile_G1
      Serial.print(F("The received node array is fully populated, but not all expected nodes have been received for water management group "));
      Serial.print(pinout_considered);
      Serial.println(F("."));
      Serial.print(F("The water manager will be prompted. Please troubleshoot transmission reliability of nodes connected to water management group "));
      Serial.println(pinout_considered);
    }
    if (!compile_G1) {                                                                              //IF none of the above conditions were satisfied (i.e. compile_G1 != true | !compile_G1)
      Serial.print(F("Compiling not yet required for water management group "));
      Serial.println(pinout_considered);
    }
    //-----END Group 1



    //-----Group 2
    pinout_considered = 2;
    m = sizeof(eeprom_object.nodes_group_2) / sizeof(eeprom_object.nodes_group_2[0]);
    n = sizeof(eeprom_object.received_nodes_group_2) / sizeof(eeprom_object.received_nodes_group_2[0]);
                                                                                                   //begin if statements
    if (isSubset(eeprom_object.nodes_group_2, eeprom_object.received_nodes_group_2, m, n)) {       //If the received node array contains all unique elements of the expected node array
      Serial.print(F("Transmissions from each expected Node have been received for pinout "));
      Serial.println(pinout_considered);
      compile_G2 = true;                                                                           //compile_G2
    }
    count = 0;
    for (int g = 0; g < 16; g++) {                                                                 //OR if the received node array contains 3 instances of any particular element of the expected node array
      for (int h = 0; h < 16; h++) {
        if (eeprom_object.received_nodes_group_2[h] == eeprom_object.nodes_group_2[g]) {
          if (eeprom_object.nodes_group_2[g] != 0 && eeprom_object.nodes_group_2[g] != -1) {
            count++;
          }
        }
        if (count >= 3) {
          compile_G2 = true;                                                                       //compile_G2
          Serial.print(F("Three instances of Node "));
          Serial.print(eeprom_object.received_nodes_group_2[h]);
          Serial.print(F(" have been received for water management group "));
          Serial.print(pinout_considered);
          Serial.println(F(" prompting the water manager."));
          Serial.print(F("Please troubleshoot transmission reliability for nodes connected to water management group "));
          Serial.println(pinout_considered);
          break;                                                                                   //break out of the current for loop
        }
      }
      count = 0;                                                                                   //reset count
    }
    count2 = 0;                                                                                    //OR if the received node array does not contain 0
    for (int  v = 0; v < 16; v++) {
      if (eeprom_object.received_nodes_group_2[v] != 0 && eeprom_object.received_nodes_group_2[v] != -1) {
        count2++;
      }
    }
    if (count2 >= 16) {
      compile_G2 = true;                                                                           //compile_G2
      Serial.print(F("The received node array is fully populated, but not all expected nodes have been received for water management group "));
      Serial.print(pinout_considered);
      Serial.println(F("."));
      Serial.print(F("The water manager will be prompted. Please troubleshoot transmission reliability of nodes connected to water management group "));
      Serial.println(pinout_considered);
    }
    if (!compile_G2) {                                                                             //IF none of the above conditions were satisfied (i.e. compile_G2 != true | !compile_G2)
      Serial.print(F("Compiling not yet required for water management group "));
      Serial.println(pinout_considered);
    }
    //-----End Group 2
    
    //-----Group 3
    pinout_considered = 3;
    m = sizeof(eeprom_object.nodes_group_3) / sizeof(eeprom_object.nodes_group_3[0]);
    n = sizeof(eeprom_object.received_nodes_group_3) / sizeof(eeprom_object.received_nodes_group_3[0]);
                                                                                                 //begin if statements
    if (isSubset(eeprom_object.nodes_group_3, eeprom_object.received_nodes_group_3, m, n)) {     //If the received node array contains all unique elements of the expected node array
      Serial.print(F("Transmissions from each expected Node have been received for pinout "));
      Serial.println(pinout_considered);
      compile_G3 = true;                                                                         //compile_G3
    }
    count = 0;
    for (int g = 0; g < 16; g++) {                                                               //OR if the received node array contains 3 instances of any particular element of the expected node array
      for (int h = 0; h < 16; h++) {
        if (eeprom_object.received_nodes_group_3[h] == eeprom_object.nodes_group_3[g]) {
          if (eeprom_object.nodes_group_3[g] != 0 && eeprom_object.nodes_group_3[g] != -1) {
            count++;
          }
        }
        if (count >= 3) {
          compile_G3 = true;                                                                     //compile_G3
          Serial.print(F("Three instances of Node "));
          Serial.print(eeprom_object.received_nodes_group_3[h]);
          Serial.print(F(" have been received for water management group "));
          Serial.print(pinout_considered);
          Serial.println(F(" prompting the water manager."));
          Serial.print(F("Please troubleshoot transmission reliability for nodes connected to water management group "));
          Serial.println(pinout_considered);
          break;                                                                                 //break out of the current for loop
        }
      }
      count = 0; //reset count
    }
    count2 = 0;                                                                                  //OR if the received node array does not contain 0
    for (int  v = 0; v < 16; v++) {
      if (eeprom_object.received_nodes_group_3[v] != 0 && eeprom_object.received_nodes_group_3[v] != -1) {
        count2++;
      }
    }
    if (count2 >= 16) {
      compile_G3 = true;                                                                         //compile_G3
      Serial.print(F("The received node array is fully populated, but not all expected nodes have been received for water management group "));
      Serial.print(pinout_considered);
      Serial.println(F("."));
      Serial.print(F("The water manager will be prompted. Please troubleshoot transmission reliability of nodes connected to water management group "));
      Serial.println(pinout_considered);
    }
    if (!compile_G3) {                                                                           //IF none of the above conditions were satisfied (i.e. compile_G3 != true | !compile_G3)
      Serial.print(F("Compiling not yet required for water management group "));
      Serial.println(pinout_considered);
    }
    //-----end Group 3

    //-----Group 4
    pinout_considered = 4;
    m = sizeof(eeprom_object.nodes_group_4) / sizeof(eeprom_object.nodes_group_4[0]);
    n = sizeof(eeprom_object.received_nodes_group_4) / sizeof(eeprom_object.received_nodes_group_4[0]);
                                                                                                 //begin if statements
    if (isSubset(eeprom_object.nodes_group_4, eeprom_object.received_nodes_group_4, m, n)) {     //If the received node array contains all unique elements of the expected node array
      Serial.print(F("Transmissions from each expected Node have been received for pinout "));
      Serial.println(pinout_considered);
      compile_G4 = true;                                                                         //compile_G4
    }
    count = 0;
    for (int g = 0; g < 16; g++) {                                                               //OR if the received node array contains 3 instances of any particular element of the expected node array
      for (int h = 0; h < 16; h++) {
        if (eeprom_object.received_nodes_group_4[h] == eeprom_object.nodes_group_4[g]) {
          if (eeprom_object.nodes_group_4[g] != 0 && eeprom_object.nodes_group_4[g] != -1) {
            count++;
          }
        }
        if (count >= 3) {
          compile_G4 = true;                                                                     //compile_G4
          Serial.print(F("Three instances of Node "));
          Serial.print(eeprom_object.received_nodes_group_4[h]);
          Serial.print(F(" have been received for water management group "));
          Serial.print(pinout_considered);
          Serial.println(F(" prompting the water manager."));
          Serial.print(F("Please troubleshoot transmission reliability for nodes connected to water management group "));
          Serial.println(pinout_considered);
          break;                                                                                 //break out of the current for loop
        }
      }
      count = 0;                                                                                 //reset count
    }
    count2 = 0;                                                                                  //OR if the received node array does not contain 0
    for (int  v = 0; v < 16; v++) {
      if (eeprom_object.received_nodes_group_4[v] != 0 && eeprom_object.received_nodes_group_4[v] != -1) {
        count2++;
      }
    }
    if (count2 >= 16) {
      compile_G4 = true;                                                                         //compile_G4
      Serial.print(F("The received node array is fully populated, but not all expected nodes have been received for water management group "));
      Serial.print(pinout_considered);
      Serial.println(F("."));
      Serial.print(F("The water manager will be prompted. Please troubleshoot transmission reliability of nodes connected to water management group "));
      Serial.println(pinout_considered);
    }
    if (!compile_G4) {                                                                           //IF none of the above conditions were satisfied (i.e. compile_G4 != true | !compile_G4)
      Serial.print(F("Compiling not yet required for water management group "));
      Serial.println(pinout_considered);
    }
//-----End Group 4


    //Compile Routine for Groups 1,2,3,4 | need to average the data in the storage array
    //AND Compare to water management settings in eeprom and trigger an event if so...

    //-----Group 1
    if (compile_G1) {                                                                               //If the boolean indicates compiling is ready...
      int received_mean_G1;
      int comp_count;
      int sum;
      for (int d = 0; d < 64; d++) {                                                               //for each element of the array
        if (eeprom_object.received_irr_group_1[d] != 0) {                                          //if not equal to 0
          sum + eeprom_object.received_irr_group_1[d];                                             //Add to sum
          comp_count ++;                                                                           //increment count
        }
        received_mean_G1 = sum / comp_count;                                                       //obtain mean
      }
      //-----Compare here
      Serial.print(F("Compiled mean for water management group 1 is: "));
      Serial.println(received_mean_G1);
      Serial.print(F("The threshold mean for water management group 1 is: "));
      Serial.println(eeprom_object.water_threshold_1);
      if (received_mean_G1 <= eeprom_object.water_threshold_1) {                                   
        Serial.println(F("The compiled mean indicates the need for an irrigation event."));
        if (eeprom_object.is_water_manager_on) {
          Serial.println(F("Water Manager is currently ON, proceed to check Permission and Exclusion windows.")); //Check current time against the Permission and Exclusion settings. Not functional 8/15/2022
          Serial.println(F("Permission and Exclusion windows not currently supported, please visit https://github.com/andrewbierer/Open_Irr for the latest firmware."));
          if (!irr_indicator_1) {                                                                  //if bool = false
            irr_indicator_1 = true;                                                                //flag for irrigation event, turn the indicator true
            startMillis_1 = currentMillis;                                                         //save the start time of the current pinout state.
          } else{
            Serial.println(F("Irr_indicator did not trigger the event."));
          }
        } else {
          Serial.println(F("Water Manager is currently OFF, no action will be taken"));
        }
      } else {
        Serial.println(F("The compiled mean does not indicate the need for an irrigation event, no action will be taken"));
      }
    }

    //-----Group 2
    if (compile_G2) {                                                                               //If the boolean indicates compiling is ready...
      int received_mean_G2;
      int comp_count;
      int sum;
      for (int d = 0; d < 64; d++) {                                                                //for each element of the array
        if (eeprom_object.received_irr_group_2[d] != 0) {                                           //if not equal to 0
          sum + eeprom_object.received_irr_group_2[d];                                              //Add to sum
          comp_count ++;                                                                            //increment count
        }
        received_mean_G2 = sum / comp_count;                                                        //obtain mean
      }
      //-----Compare here
      Serial.print(F("Compiled mean for water management group 2 is: "));
      Serial.println(received_mean_G2);
      Serial.print(F("The threshold mean for water management group 2 is: "));
      Serial.println(eeprom_object.water_threshold_2);
      if (received_mean_G2 < eeprom_object.water_threshold_2) {
        Serial.println(F("The compiled mean indicates the need for an irrigation event."));
        if (eeprom_object.is_water_manager_on) {
          Serial.println(F("Water Manager is currently ON, proceed to check Permission and Exclusion windows.")); //Check current time against the Permission and Exclusion settings. Not functional 8/15/2022

          Serial.println(F("Permission and Exclusion windows not currently supported, please visit https://github.com/andrewbierer/Open_Irr for the latest firmware."));
          if (!irr_indicator_2) {                                                                   //if bool = false
            irr_indicator_2 = true;                                                                 //flag for irrigation event, turn the indicator true
            startMillis_2 = currentMillis;                                                          //save the start time of the current pinout state.
          }
        } else {
          Serial.println(F("Water Manager is currently OFF, no action will be taken"));
        }
      } else {
        Serial.println(F("The compiled mean does not indicate the need for an irrigation event, no action will be taken"));
      }
    }

    //-----Group 3
    if (compile_G3) {                                                                               //If the boolean indicates compiling is ready...
      int received_mean_G3;
      int comp_count;
      int sum;
      for (int d = 0; d < 64; d++) {                                                                //for each element of the array
        if (eeprom_object.received_irr_group_1[d] != 0) {                                           //if not equal to 0
          sum + eeprom_object.received_irr_group_1[d];                                              //Add to sum
          comp_count ++;                                                                            //increment count
        }
        received_mean_G3 = sum / comp_count;                                                        //obtain mean
      }
      //-----Compare here
      Serial.print(F("Compiled mean for water management group 3 is: "));
      Serial.println(received_mean_G3);
      Serial.print(F("The threshold mean for water management group 3 is: "));
      Serial.println(eeprom_object.water_threshold_3);
      if (received_mean_G3 < eeprom_object.water_threshold_3) {
        Serial.println(F("The compiled mean indicates the need for an irrigation event."));
        if (eeprom_object.is_water_manager_on) {
          Serial.println(F("Water Manager is currently ON, proceed to check Permission and Exclusion windows.")); //Check current time against the Permission and Exclusion settings. Not functional 8/15/2022

          Serial.println(F("Permission and Exclusion windows not currently supported, please visit https://github.com/andrewbierer/Open_Irr for the latest firmware."));
          if (!irr_indicator_3) {                                                                   //if bool = false
            irr_indicator_3 = true;                                                                 //flag for irrigation event, turn the indicator true
            startMillis_3 = currentMillis;                                                          //save the start time of the current pinout state.
          }
        } else {
          Serial.println(F("Water Manager is currently OFF, no action will be taken"));
        }
      } else {
        Serial.println(F("The compiled mean does not indicate the need for an irrigation event, no action will be taken"));
      }
    }

    //-----Group 4
    if (compile_G4) {                                                                               //If the boolean indicates compiling is ready...
      int received_mean_G4;
      int comp_count;
      int sum;
      for (int d = 0; d < 64; d++) {                                                                //for each element of the array
        if (eeprom_object.received_irr_group_4[d] != 0) {                                           //if not equal to 0
          sum + eeprom_object.received_irr_group_4[d];                                              //Add to sum
          comp_count ++;                                                                            //increment count
        }
        received_mean_G4 = sum / comp_count;                                                        //obtain mean
      }
      //-----Compare here
      Serial.print(F("Compiled mean for water management group 4 is: "));
      Serial.println(received_mean_G4);
      Serial.print(F("The threshold mean for water management group 4 is: "));
      Serial.println(eeprom_object.water_threshold_4);
      if (received_mean_G4 < eeprom_object.water_threshold_4) {
        Serial.println(F("The compiled mean indicates the need for an irrigation event."));
        if (eeprom_object.is_water_manager_on) {
          Serial.println(F("Water Manager is currently ON, proceed to check Permission and Exclusion windows.")); //Check current time against the Permission and Exclusion settings. Not functional 8/15/2022
          Serial.println(F("Permission and Exclusion windows not currently supported, please visit https://github.com/andrewbierer/Open_Irr for the latest firmware."));
          if (!irr_indicator_4) {                                                                   //if bool = false
            irr_indicator_4 = true;                                                                 //flag for irrigation event, turn the indicator true
            startMillis_4 = currentMillis;                                                          //save the start time of the current pinout state.
          }
        } else {
          Serial.println(F("Water Manager is currently OFF, no action will be taken"));
        }
      } else {
        Serial.println(F("The compiled mean does not indicate the need for an irrigation event, no action will be taken"));
      }
    }
  }
  //------Check for active irrigation events.
  if (irr_indicator_1) {
    if ((currentMillis - startMillis_1) <= (eeprom_object.irr_period_1 * 60000)) {                  //If the irrigation period has not elapsed... (minutes * 60000) = milliseconds
      Serial.println(F("Irrigation event active for management group 1..."));
      Serial.print(F("Approximately "));
      Serial.print((currentMillis - startMillis_1) / 60000);                                        //minutes remaining
      Serial.println(F("  minutes remaining in the event."));
      digitalWrite(in1, HIGH);
    } else {
      Serial.println(F("Irrigation event for management group 1 complete..."));
      digitalWrite(in1, LOW);                                                                       //set in1 LOW
      irr_indicator_1 = false;                                                                      //reset irr_indicator
    }
  }

  if (irr_indicator_2) {
    if ((currentMillis - startMillis_2) <= (eeprom_object.irr_period_2 * 60000)) {                  //If the irrigation period has not elapsed... (minutes * 60000) = milliseconds
      Serial.println(F("Irrigation event active for management group 2..."));
      Serial.print(F("Approximately "));
      Serial.print((currentMillis - startMillis_2) / 60000);                                        //minutes remaining
      Serial.println(F("  minutes remaining in the event."));
      digitalWrite(in2, HIGH);
    } else {
      Serial.println(F("Irrigation event for management group 2 complete..."));
      digitalWrite(in2, LOW);                                                                       //set in1 LOW
      irr_indicator_2 = false;                                                                      //reset irr_indicator
    }
  }

  if (irr_indicator_3) {
    if ((currentMillis - startMillis_3) <= (eeprom_object.irr_period_3 * 60000)) {                  //If the irrigation period has not elapsed... (minutes * 60000) = milliseconds
      Serial.println(F("Irrigation event active for management group 3..."));
      Serial.print(F("Approximately "));
      Serial.print((currentMillis - startMillis_3) / 60000);                                        //minutes remaining
      Serial.println(F("  minutes remaining in the event."));
      digitalWrite(in3, HIGH);
    } else {
      Serial.println(F("Irrigation event for management group 3 complete..."));
      digitalWrite(in3, LOW);                                                                       //set in1 LOW
      irr_indicator_3 = false;                                                                      //reset irr_indicator
    }
  }

  if (irr_indicator_4) {
    if ((currentMillis - startMillis_4) <= (eeprom_object.irr_period_4 * 60000)) {                  //If the irrigation period has not elapsed... (minutes * 60000) = milliseconds
      Serial.println(F("Irrigation event active for management group 4..."));
      Serial.print(F("Approximately "));
      Serial.print((currentMillis - startMillis_4) / 60000);                                        //minutes remaining
      Serial.println(F("  minutes remaining in the event."));
      digitalWrite(in4, HIGH);
    } else {
      Serial.println(F("Irrigation event for management group 4 complete..."));
      digitalWrite(in4, LOW);                                                                       //set in1 LOW
      irr_indicator_4 = false;                                                                      //reset irr_indicator
    }
  }
}

//-----is Subset function----------------------------------------------

bool isSubset(int array1[], int array2[], int m, int n) {                                           //Returns 1 if array 2 is a subset of array 1
  int i = 0;
  int j = 0;
  for (i = 0; i < n; i ++) {
    for (j = 0; j < m; j++) {
      if (array2[i] == array1[j]) {
        break;
      }
    }
    if (j == m) {                                                                                   //If the loop above is not broken at all then array2[i] is not present in array1[]
      return 0;
    }
  }
  return 1;                                                                                         //If we get here then all elements of array2[] are present in array1[]
}

//-----Display Water Management Settings function-----------------------
void display_water_management_settings() {
  Serial.println();
  Serial.print(F("Gateway Water Manager is "));
  if (eeprom_object.is_water_manager_on) {
    Serial.println(F("ON."));
  } else {
    Serial.println(F("OFF."));
  }
  Serial.println();
  Serial.println(F("Current water management system settings: "));
  
  Serial.println(F("Water management Setting 1: "));
  int elmnt_chk = 0;
  for (int j = 0; j < 16; j++) {
    if (eeprom_object.nodes_group_1[j] == 0) {
      elmnt_chk++;
    }
    if (eeprom_object.nodes_group_1[j] == -1) {
      elmnt_chk++;
    }
  }
  if (elmnt_chk != 16) {
    for (int j = 0; j < 16; j++) {
      Serial.print(F("Node: "));
      Serial.print(eeprom_object.nodes_group_1[j]);
                                                                                    //[((num_nodes+1)*4)-(4-w)] from below
      Serial.print(F(" Groups: "));
      for (int k = 0; k < 4; k++) {
        Serial.print(eeprom_object.irr_group_1[((j + 1) * 4) - (4 - k)]);          //formula should be correct.
        Serial.print(',');
      }
      Serial.println();
    }
  } else {
    Serial.println(F(" Setting undefined. "));                                     //if element check fails, report the setting is undefined.
  }

  Serial.println(F("Water management Setting 2: "));
  elmnt_chk = 0;
  for (int j = 0; j < 16; j++) {
    if (eeprom_object.nodes_group_2[j] == 0) {
      elmnt_chk++;
    }
    if (eeprom_object.nodes_group_2[j] == -1) {
      elmnt_chk++;
    }
  }
  if (elmnt_chk != 16) {
    for (int j = 0; j < 16; j++) {
      Serial.print(F("Node: "));
      Serial.print(eeprom_object.nodes_group_2[j]);
      Serial.print(F(" Groups: "));
      for (int k = 0; k < 4; k++) {
        Serial.print(eeprom_object.irr_group_2[((j + 1) * 4) - (4 - k)]);          //formula should be correct.
        Serial.print(',');
      }
      Serial.println();
    }
  } else {
    Serial.println(F(" Setting undefined. "));                                     //if element check fails, report the setting is undefined.
  }

  Serial.println(F("Water management Setting 3: "));
  elmnt_chk = 0;
  for (int j = 0; j < 16; j++) {
    if (eeprom_object.nodes_group_3[j] == 0) {
      elmnt_chk++;
    }
    if (eeprom_object.nodes_group_3[j] == -1) {
      elmnt_chk++;
    }
  }
  if (elmnt_chk != 16) {
    for (int j = 0; j < 16; j++) {
      Serial.print(F("Node: "));
      Serial.print(eeprom_object.nodes_group_3[j]);
      Serial.print(F(" Groups: "));
      for (int k = 0; k < 4; k++) {
        Serial.print(eeprom_object.irr_group_3[((j + 1) * 4) - (4 - k)]);          //formula should be correct.
        Serial.print(',');
      }
      Serial.println();
    }
  } else {
    Serial.println(F(" Setting undefined. "));                                     //if element check fails, report the setting is undefined.
  }

  Serial.println(F("Water management Setting 4: "));
  elmnt_chk = 0;
  for (int j = 0; j < 16; j++) {
    if (eeprom_object.nodes_group_4[j] == 0) {
      elmnt_chk++;
    }
    if (eeprom_object.nodes_group_4[j] == -1) {
      elmnt_chk++;
    }
  }
  if (elmnt_chk != 16) {
    for (int j = 0; j < 16; j++) {
      Serial.print(F("Node: "));
      Serial.print(eeprom_object.nodes_group_4[j]);
      Serial.print(F(" Groups: "));
      for (int k = 0; k < 4; k++) {
        Serial.print(eeprom_object.irr_group_4[((j + 1) * 4) - (4 - k)]);          //formula should be correct.
        Serial.print(',');
      }
      Serial.println();
    }
  } else {
    Serial.println(F(" Setting undefined. "));                                     //if element check fails, report the setting is undefined.
  }
  Serial.println();

  Serial.println(F("Water threshold levels: "));
  Serial.print(F("Setting 1 threshold:  "));
  if (eeprom_object.water_threshold_1 == -1) {
    Serial.println(F("Threshold undefined."));
  } else {
    Serial.println(eeprom_object.water_threshold_1);
  }
  Serial.print(F("Setting 2 threshold:  "));
  if (eeprom_object.water_threshold_2 == -1) {
    Serial.println(F("Threshold undefined."));
  } else {
    Serial.println(eeprom_object.water_threshold_2);
  }
  Serial.print(F("Setting 3 threshold:  "));
  if (eeprom_object.water_threshold_3 == -1) {
    Serial.println(F("Threshold undefined."));
  } else {
    Serial.println(eeprom_object.water_threshold_3);
  }
  Serial.print(F("Setting 4 threshold:  "));
  if (eeprom_object.water_threshold_4 == -1) {
    Serial.println(F("Threshold undefined."));
  } else {
    Serial.println(eeprom_object.water_threshold_4);
  }

  Serial.print(F("Minimum time between irrigation events: "));
  if (eeprom_object.min_time_btwn_irr == -1) {
    Serial.println(F("Time between irrigation events undefined."));
  } else {
    Serial.println(eeprom_object.min_time_btwn_irr);
  }

  Serial.println(F("Duration of irrigation events: "));
  Serial.print(F("Setting 1 duration:  "));
  if (eeprom_object.irr_period_1 == -1) {
    Serial.println(F("Duration undefined."));
  } else {
    Serial.println(eeprom_object.irr_period_1);
  }
  Serial.print(F("Setting 2 duration:  "));
  if (eeprom_object.irr_period_2 == -1) {
    Serial.println(F("Duration undefined."));
  } else {
    Serial.println(eeprom_object.irr_period_2);
  }
  Serial.print(F("Setting 3 duration:  "));
  if (eeprom_object.irr_period_3 == -1) {
    Serial.println(F("Duration undefined."));
  } else {
    Serial.println(eeprom_object.irr_period_3);
  }
  Serial.print(F("Setting 4 duration:  "));
  if (eeprom_object.irr_period_4 == -1) {
    Serial.println(F("Duration undefined."));
  } else {
    Serial.println(eeprom_object.irr_period_4);
  }
}



//____________________________________________________________________________________________________________________________________
//____________________________________________________________________________________________________________________________________
//____________________________________________________________________________________________________________________________________