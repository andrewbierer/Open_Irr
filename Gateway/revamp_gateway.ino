/*
   Gateway/Reciever for watermark soil water tension management system

   Purpose: Collect node data for storage on a sd card module / bluetooth to external PC for
   logging.

   Note: Future expansion through integration of LTE cellular network is desired. the SIMCom SIM7500V is a verison carrier with B4 & B13 band LTE service.
   Design based on the ArduinoSoilH2O project <https://github.com/ArduinoSoilH2O>

  Version History:
  2022/04/12 -> The Node and Gateway setup is now configured to send/recieve/parse/save up to 2 packets of information with a character ']' indicating the end of the transmission from the node.

*/

//-----Some future considerations------------------------------------------------------
/*
   1  3/28/22, The battery power supply is shorted (When switch is "ON", "OFF" is OK) causing voltge regulator to overheat, troubleshoot power supply issue. USB-power supply OK
   2  menu call for displaying radio information
   3  Checking time with some standard.
   4  Sending time to Addressed nodes for update
   5  Our RSSI is ~ -132 in the lab.... why so low?
*/
//-----Libraries---------------------------------------------------

#include<SD.h>                    //sd card functionality
#include<SPI.h>                   //SPI functions
#include<Wire.h>                  //I2C functions for rtc
#include<EEPROM.h>                //Built in EEPROM routines
#include"RTClib.h"
#include<DS3232RTC.h>             //Precision (generic) RTC library for DS3232/DS3231 rtc module, requires v 1.3
#include <RH_RF95.h>              // Radio library for RF95 transciever 
#include <RHReliableDatagram.h>   // Radio library to simplify data sending/recieving
#include<LowPower.h>              //Low power functionality
#include <RHGenericDriver.h>
#include <SoftwareSerial.h>       //for BT function

//-----Assign Pins---------------------------------------------------

#define pin_mBatt 30              //Digital pin to activate battery voltage measurement circuit
#define pin_battV 31              //Pin for reading Voltage out from divider circuit to calculate Voltage in
#define ADC_REF_VOLTAGE 3.3               //The reference output of the moteino mega board
#define ADC_RESOLUTION  1024              //The analog digital converter in moteino mega is 10 bit resolution
#define ADC_MAXVALUE  1023                // 1024 possibilities including 0 (0-1023)

#define LED 15                    //Pin for LED on Moteino Mega board
#define SD_CS 0                   //Chipselect pin for sdcard module
//#define SD_CD 1                   //Chipdetect pin for sdcard module
#define BTsleep 12                // Bluetooth on/off switch | LOW = on, HIGH = off
#define BTstate 13                // Bluetooth State (tells if on or off)
#define RTC_Interrupt 24          // 

//-----Declare Variables---------------------------------------------

char    filename[] = "000_Data.txt";                    // SDcard file name as char array
char    filename2[] = "debug.txt";

char    data[RH_RF95_MAX_MESSAGE_LEN];                 // array to concatenate data arrays //dont think this is used? 2022-06-15
uint8_t int_buff[RH_RF95_MAX_MESSAGE_LEN];             // integer type array buffer
uint8_t length_buff;                                   // integer denoting the size of the buffer
uint8_t from_node_id;                                  // integer specifying the node id that a transmission is coming from

uint8_t last_node_id = 0;                              // integer soecifying the node the last transmission came from
String   all_data;                                     // testing, appears to work
unsigned long incomplete_transmission_count = 0;       // to keep track of the number of incomplete transmissions recieved, but not parsed to the data file.

byte secs;                                             //time and date values
byte mins;
byte hrs;
byte dow;
byte days;
byte mnths;
int yrs;
byte alarm_1_Mins;
tmElements_t tm;

char serial_inc;                                       //store incoming value from serial

char a[4];                                             //char array for itoa function
byte i;                                                //for-loop counter

float battV;                             // battery voltage
float lowBatt = 4.0;                     // low-battery limit
bool battLow = false;                    //tracks if battery voltage falls below specified threshold, save to eeprom?

boolean isMenuOn = false;
int menuinput;                                         //user input to menu prompt
long menutimeout;                                      //length of time to wait for user input
int indata;                                            //user input data
int input;
int numincoming;
int incoming[7];
boolean firstTime = true;                              //flag for first time

const byte numChars = 32;                              //for reading in character user input
char charInput[200];                                   //for charinput function
char incomingChar[200];

//EEPROM structure method for storing and retrieving values from eeprom: relevant for variables defined in the menu
// Note that each EEPROM position can save only one byte of information, i.e. 8-bit numbers 0-255 and leading values (001) ARE NOT INTERPRETABLE
// but more than one position can be accessed using eeprom.put and eeprom.get with a defined structure
// this also enables saving different data types together

int eeprom_address = 0;                                //Start storing to eeprom at position 0

struct eeprom_struct {

  uint8_t IDnum;                                       //numeric board identifier 0-255
  uint8_t gatewayID;                                   //gatewayID for radio networking, we will always use 1 for the gatewayID and number nodes thereafter 2-X
  char projectID[numChars] = {0};                      //Project identifier

  boolean firstTime = true;                            //flag for first time for writing to sdcard module

  int ALARM_1_Interval = 1;                            //Set the interval for alarm 1 (wake and listen routine), minutes

  boolean bluetooth_on = false;

};
eeprom_struct eeprom_object = {};                      //Declare an object with the eeprom_struct structure


//-----Initialize----------------------------------------------------

RTC_DS3231 rtc;                                                 //Specify real time clock

RH_RF95 driver(4, 2);                                           //Specify radio driver, explicit call to driver pins req?? see <https://lowpowerlab.com/guide/moteino/lora-support/>
RHReliableDatagram manager(driver, eeprom_object.gatewayID);    //Setup Radio manager

File myfile;                                                    //make sdcard file object
File myfile2;                                                   //make second sdcard file object??

SoftwareSerial bluetooth(10, 11);                               //corresponds to Rx|Tx of moteino where bt module is connected


//_______________________________________________________________________________________________________________________________
void setup() {
  Serial.begin(9600);
  delay(100);
  Wire.begin();                                    //enable I2C bus for rtc
  delay(300);

  pinMode(LED, OUTPUT);                            // transistor pin to sleep Bluetooth??

  pinMode(SD_CS, OUTPUT);                          // set sd card ChipSelect pin as output
  //pimMode(SD_CD, INPUT);                           // sd CD pin, has lead to fried sdcards...

  pinMode(BTsleep, OUTPUT);
  pinMode(BTstate, INPUT);

  pinMode(pin_mBatt, OUTPUT);
  digitalWrite(pin_mBatt, LOW);                 //Leave the battery voltage circuit open to avoid battery drain
  pinMode(pin_battV, INPUT);                     // to read voltage

  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC, routine hang up"));
    while (1);
  }

  if (! SD.begin(SD_CS)) {
    Serial.println(F("SD card not present or card failure."));
  }; //If utilized to check sd card conditions?

  Serial.println(F("Initialization Completed"));

  battV = calcbattV();                            // Get board battery level on startup

  // check chip EEPROM for stored data and place into "eeprom_object" with the structure of "eeprom_struct" declared earlier
  // will need to call eeprom_object.VARIABLE to access the variables within the objects structure
  EEPROM.get(eeprom_address, eeprom_object); //eeprom_address may be pointless if only writing one eeprom object (i.e. it would always begin at position 0)

  delay(10);
  driver.setFrequency(915.0);
  delay(10);
  // driver.ModemConfigChoice(Bw125Cr45Sf128);
  // delay(10);
  driver.setTxPower(20, false);
  delay(10);
  manager.setThisAddress(eeprom_object.gatewayID);                           //Set the current GATEWAY address for radio communication
  delay(10);
  manager.setTimeout(10000);                                               //set timeout period where if an ACK not recieved it will retransmit message Default is 200ms
  delay(1000);

  menu();                                           //launch menu routine

}

void loop() {

  if (eeprom_object.bluetooth_on) {
    bt_on_loop();                                  //Loop for bluetooth ON.
    readRTC();            //Read the DS3231 RTC
    delay(50);
    Serial.println(F("Looking for Nodes addressed to this gatewayID..."));
    Serial.print(F("Current gatewayID is: "));
    Serial.println(eeprom_object.gatewayID);
    battV = calcbattV();                                            //check battery voltage with subroutine
    Serial.print(F("Gateway Battery Pack Voltage:  "));
    Serial.println(battV);
    getRadioData();
    while (all_data[all_data.length() - 1] == ']') {
      Serial.println(F("Transmission end character recieved."));
      storeData();                                              //Call the storeData function if there is data to store
      clearBuffer();
    }
    Serial.println(F("Loop complete."));
    check_menu();
  } else {
    //----demo loop with constant read, NOTE that a function to send the bluetooth data may still be needed 11/16/2021

    readRTC();            //Read the DS3231 RTC
    delay(50);

    Serial.println(F("Looking for Nodes addressed to this gatewayID..."));
    Serial.print(F("Current gatewayID is: "));
    Serial.println(eeprom_object.gatewayID);

    battV = calcbattV();                                            //check battery voltage with subroutine
    Serial.print(F("Gateway Battery Pack Voltage:  "));
    Serial.println(battV);

    getRadioData();

    //Serial.print(F("all_data: "));
    //Serial.println(all_data);

    while (all_data[all_data.length() - 1] == ']') {
      Serial.println(F("Transmission end character recieved."));
      storeData();                                              //Call the storeData function if there is data to store
      clearBuffer();
    }

    // Serial.print(F("Incomplete transmission count:  "));      //report the count of incomplete transmissions.. Note this would need added to EEPROM to pull it from memory instead of resetting everytime you plug into the gateway...
    // Serial.println(incomplete_transmission_count);
    Serial.println(F("Loop complete."));


    check_menu();
    //delay(1000);
    //----real loop with sleep time integrated
    //if(mins % 15 == 0){                       //Every 15 minutes (identified using modulo), do something
    //Serial.println(F("Looking for Nodes addressed to this gatewayID..."));
    //getRadioData();
    //Serial.println(F("Loop complete, return to sleep."));
    //sleepytime(7);
    //} else {sleepytime(7)};
  }
}

//____________________________________________________________________________________________________________________________________________________________________________________________________________

//-----getRadioData function---------------------------------------------

void getRadioData() {

  digitalWrite(LED, HIGH);                                  //Turn board LED on while getting radio data

  if (!manager.init())
    Serial.println(F("Radio failed to initialize."));        //If the radio manager didnt initialize...
  delay(20);

  if (manager.init()) {
    Serial.println(F("Radio initilization successful."));
    driver.setFrequency(915.0);                               //Define the radio settings explicitly each time 
    delay(10);
    driver.setTxPower(20, false);
    
    int freq_error = 0;
    freq_error = driver.frequencyError();                                   //estimates the offset of last measurement in Hz
    Serial.print(F("Frequency Error:  "));                                  //The estimated centre frequency offset in Hz of the last received message. If the modem bandwidth selector in register RH_RF95_REG_1D_MODEM_CONFIG1 is invalid, returns 0.
    Serial.println(freq_error);
  }

  while (manager.available()) {
    Serial.println(F("Waiting for transmissions from addressed nodes..."));
    length_buff = sizeof(int_buff);                                 //this is max size of 1 packet, 251
    uint8_t from_node_id;
    if (manager.recvfromAck(int_buff, &length_buff, &from_node_id)) {         //If you want to timeout while waiting for a packet use manager.recvfromAckTimeout(int_buff, &length_buff, 2000 &from_node_id) indicating ms for timeout
      //Serial.print(F("Recieved Request From : 0x"));              //If recieving in HEX is desired
      //Serial.print(from_node_id, HEX);                            //
      Serial.print(F("Recieved request from Node: "));
      Serial.print(from_node_id);
      Serial.print(F(": "));
      Serial.println((char*)int_buff);

      Serial.print(F("RSSI: "));                                    //Recieved Signal Strength Inidicator (RSSI) measures amount of power present in radio signal. A greater negative value indicates a weaker signal.
      Serial.println(driver.lastRssi(), DEC);

      Serial.print(F("last_node_id: "));
      Serial.println(last_node_id);
      Serial.print(F("from_node_id: "));
      Serial.println(from_node_id);

      if (last_node_id == from_node_id && last_node_id != 0) {                        //If this packet came from the same node as the last packet..
        Serial.println(F("packet came from same node as last packet."));
        if (strchr((char*)int_buff, ']') != NULL) {                                //This packet MUST end/contain the transmission end character ']' to be appended to all_data string
          Serial.println(F("It is okay, this packet has the transmission end character."));
          all_data += (char*)int_buff;
        } else {                                                                      //If this packet does not end with ']', clear the all_data string and increment the incomplete_transmisison_count
          Serial.println(F("This packet does not end with the transmission end character, clearing buffer before appending all_data."));
          clearBuffer();
          delay(10);
          all_data += (char*)int_buff;
          //incomplete_transmission_count ++;                                          //doesnt count right,
        }
      } else if (last_node_id == from_node_id) {                                       //If from the same node and last_node_id is in fact 0....
        clearBuffer();
        delay(10);
        //incomplete_transmission_count ++;
      } else {

         //If this packet came from a different node as the last packet, what if the gateway only recieved the second packet and not the first? -> the string would be shorter than expected
         // 05/03/2022 -> This can probably be fixed by transmitting the number of packets to be expected. a simple Modulo calculation on the total string length can help identify number of packets to be sent. 
           
        all_data += (char*)int_buff;                                                   //If this packet came from a different node as the last packet, append the all_data string 
      }
    }


    // for (i = 0; i < length_buff; i++) {                           //Append the data character array for the length of the filled buffer, data is a character array of length 251
    //   data[i] = int_buff[i];
    // }

    //  if (length_buff > 0) {                                        //this code works..
    //    storeData();                                                //Call the storeData function if there is data to store NOTE DISABLED FOR NOW
    //  }
    last_node_id = from_node_id;                                     //set the id of the node from which the most recent transmission was recieved
  }
  // Serial.println(data);                                         //For troubleshooting
  digitalWrite(LED, LOW);                                          //Set board LED pin low when not trying to recieve transmissions
  delay(50);
}

//-----storeData Routine--------------------------------------------------

void storeData() {

  if (! SD.begin(SD_CS)) {                                        //check if sdcard is inserted, if not dont do anything
    Serial.println(F("SD card not present or card failure."));
  };

  battV = calcbattV();                                            //check battery voltage with subroutine

  myfile = SD.open(filename, FILE_WRITE);                         //open data file
  delay(10);

  if (myfile) {                                                   //if open
    if (eeprom_object.firstTime == true) {                                      //and it is the first time the file was opened, then print the expected data format that will be written to sd card
      myfile.println();
      //myfile.print(F("Temp header for data format."));
      myfile.println(F("Arduino soil water tension management system"));
      myfile.print(F("Project ID: "));
      myfile.println(eeprom_object.projectID);
      myfile.println(F("Data values are stored as follows:"));
      myfile.println(F("GatewayID, Gateway battery voltage (V), Node ID, Node battery voltage (V), DateTime, Node data string"));
      myfile.println(F("Node data strings have the follwing format:"));
      myfile.println(F("Following DateTime the mux channel position and sensor value (CB/kpa) for all watermark sensors installed are printed (Channel, Value, Channel, Value)"));
      myfile.println(F("Following the raw watermark sensor data, the mean for specifed groups of channels are reported as (G(1-4), Mean)"));
      myfile.println(F("After watermark group means, DS18b20 temperature sensor data is reported as (T(# of sensor), Value (C))"));
      myfile.println();

      eeprom_object.firstTime = false;                                        //Only include this header when first writing to file
      eeprom_address = 0;                                                     //clear eeprom_address
      EEPROM.put(eeprom_address, eeprom_object);                              //Store the new firsttime flag to eeprom
      eeprom_address = 0;
    }
    //myfile.print(other things to add from gateway?, like ID and battery voltage);
    myfile.print(F("GatewayID:  "));
    myfile.print(eeprom_object.gatewayID);
    myfile.print(',');
    myfile.print(F("Gateway battV:  "));

    if (battV <= lowBatt) {
      myfile.print(F("Low Battery Voltage!: "));                      //flag to indicate low battery voltage
    }
    myfile.print(battV);

    myfile.print(',');
    //myfile.println(data);
    all_data.remove(all_data.length() - 1, 1);                    //Remove the character that indicated the end of a transmission "]"
    myfile.println(all_data);                                     //print the all_data string
    delay(100);                                                   //allow time to write data
    myfile.close();
    delay(200);                                                   //allow time for file to close
    Serial.print(F("The data string written:  "));
    Serial.println(all_data);
    Serial.println(F("Data Written Successfully."));
  } else {
    Serial.println(F("Error opening file."));                     //cannot find "myfile"
  }
}

//------Calc battery voltage (0- ~ 10V) with voltage divider circuit-------------
float calcbattV() {
  pinMode(pin_mBatt, OUTPUT);             //Set digital pin as an output to activate the circuit
  pinMode(pin_battV, INPUT);              //Set the other pin as input to read the voltage

  digitalWrite(pin_mBatt, HIGH);          //Turn the gate of the MOSFET HIGH, closing the circuit
  delayMicroseconds(500);                 //delay time for stabilization

  uint16_t vInt;
  vInt = analogRead(pin_battV);           //read vInt on analog pin connected where the resistors in the divider circuit meet.
  digitalWrite(pin_mBatt, LOW);           //Turn the gate of the MOSFET LOW, deactivating the circuit

  float vout = 0.00;
  float vin = 0.00;
  float R1 = 10000;                       //1st Resistor in divider circuit taking load from battery
  float R2 = 4700;                        //2nd Resistor in divider circuit leading to board GND
  //Serial.print(F("vInt: "));            //The raw integer returned by analogRead
  //Serial.println(vInt);
  vout = (vInt * ADC_REF_VOLTAGE) / ADC_RESOLUTION;             //voltage modified by divider circuit, voltage at dividing point
  vin = vout / (R2 / (R1 + R2));                                //Actual voltage coming into the divider circuit, i.e. battery level
  //Serial.print(F("vout: "));
  //Serial.println(vout);
  //Serial.print(F("vin:  "));
  //Serial.println(vin);

  return vin;
}
//---------- Clear buffer---------------
void clearBuffer() {
  all_data = "";
  delay(100);
}
//------Check Time-----------------------------------------------------
void readRTC() {
  //-----If using Adafruit RTC breakout PCF8532 -> works with DS3231
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
    Serial.read();                                          //clear serial input buffer
  }

  itoa(eeprom_object.IDnum, a, 10);               //convert IDnum to character array, see itoa documentation <http://www.nongnu.org/avr-libc/user-manual/group__avr__stdlib.html#ga5a3fce5fbd20140584619ba9aed09f75>


  if (eeprom_object.IDnum < 10) {                                 // for naming filename
    filename[0] = '0';                              // put into filename[] array
    filename[1] = '0';
    filename[2] = a[0];
  } else if (eeprom_object.IDnum < 100) {
    filename[0] = '0';                              // put into filename[] array
    filename[1] = a[0];
    filename[2] = a[1];
  } else {
    filename[0] = a[0];                              // put into filename[] array
    filename[1] = a[1];
    filename[2] = a[2];
  }
/*
 *   memcpy(a, eeprom_object.projectID, sizeof(eeprom_object.projectID)); //copy projectID to "a". projectID is size 32 fyi

  int count = 0;
  for (int i = 0; i <= 32; i++) {
    if (a[i] != NULL && a[i] != '\0' && a[i] != '\n') {
      count++;
    }
  }
  Serial.println(count);

  //But if you're inserting items sequentially, you should just be able to calculate carlist.length - (lastID + 1).

  for (int i = 0; i <= 32; i++) {
    filename[i] = a[i];
  }

 */


  Serial.print(F("test  "));
  Serial.println(filename);

  Serial.println();                                         //print out board information
  Serial.println(F("Arduino Drought Stress Manager"));
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
  Serial.print(hrs);                                 // time
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

  Serial.println(F("Menu Actions:"));                            // Define menu Actions--------------------------------------------
  Serial.println(F("   b  <--  Toggle bluetooth mode"));         // Toggle node bluetooth mode
  Serial.println(F("   c  <--  Set clock"));
  Serial.println(F("   i  <--  Set ID numbers"));                // set IDnum, nodeID, gatewayID
  Serial.println(F("   f  <--  Display sdcard information"));    // Get sdcard information
  Serial.println(F("   d  <--  Download all data"));             // get all data
  Serial.println(F("   e  <--  Erase all data"));                // erase all data
  Serial.println(F("   l  <--  Display LoRa radio settings"));   // display LoRa radio information
  Serial.println(F("   x  <--  Exit"));                          // exit, turn off Bluetooth
  Serial.flush();

  menutimeout = millis() + 10000;                                //wait for user input
  while (millis() < menutimeout) {

    menuinput = 120;                                            //what does this do? why 120?
    if (Serial.available() > 0) {
      menuinput = Serial.read();                                //get user input
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }

  switch (menuinput) {                                          //switch cases for menu input, Note the case# corresponds to input in ASCII format

    case 98:                                                    // "b" for Toggle Bluetooth mode----------------------------------

      Serial.println(F("Toggle Bluetooth Mode."));
      get_integer_input(); //otherwise the first input is always 0?
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

    case 108:                                                   // "l" for Display LoRa radio information--------------------------
      Serial.println(F("Display all LoRa Settings."));
      driver.printRegisters();                                  // would need parsed to be more legible..


      menu();
      break;
    case 99:                                                    // "c" for set clock-----------------------------------------------

      Serial.println(F("Set clock:  "));
      get_integer_input(); //otherwise the first input is always 0?
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


    case 102:                                                       // "f" for checking files on sdcard...
      Serial.println(F("Getting sdcard information..."));
      sdCheck();
      menu();
      break;



    case 105:                                                    // "i" for set ID numbers-----------------------------------------

      Serial.println(F("Set network ID numbers (ProjectID, BoardID, and GatewayID):"));              // set ProjectID, BoardIDnum, and GatewayID numbers
      get_integer_input(); //otherwise the first input is always 0?
      Serial.flush();

      Serial.print(F(" Project ID:     "));                      // get projectID
      Serial.flush();
      // get_integer_input();                                                // decode user input
      // eeprom_object.projectID = indata;
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

      Serial.print(F(" Board ID:     "));                        // get BoardID
      Serial.flush();
      get_integer_input();                                                // decode user input
      eeprom_object.IDnum = indata;

      Serial.print((" Gateway ID:  "));                          // get GatewayID
      Serial.flush();
      get_integer_input();                                                // decode user input
      eeprom_object.gatewayID = indata;

      manager.setThisAddress(eeprom_object.gatewayID);           // set the gatewayID for radio manager

      menu();                                                    // go back to menu
      break;

    case 100:                                                     // "d" for Download all data in sd card (prints to serial port)---------------------------------------------------------------
      Serial.println(F("Download all data:"));
      delay(100);

      myfile = SD.open(filename);                                 // open file
      //delay(1000);

      if (myfile) {

        while (myfile.available()) {                              // read file and print to screen (COMPORT)
          Serial.write(myfile.read());
        }
        myfile.close();

      } else {
        Serial.println(F("Error opening file"));
      }
      delay(10000);                                               //Give time for user to copy data...
      menu();
      break;

    case 101:                                                      // "e" for erase data on sd card-------------------------------------------
      Serial.println(F("Erase data on sd card..."));
      Serial.print(F("Currently Writing to: "));
      Serial.println(filename);
      Serial.print(F("Do you want to delete: "));
      Serial.print(filename);
      Serial.println(F("? Type YES to confirm deletion of this file."));

      get_integer_input();                                        //For first time through case or wont work as 0 is always returned...
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
          eeprom_object.firstTime = true;                         //Return firsttime to true so header is printed when first writing to file
        } else {
          Serial.println(F("If condition not satisfied"));
        }
      }
      menu();
      break;

    case 120:                                                     //"x" for Exit ---------------------------------------------------
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;


    default:                                                      //Define default case, here exit if no valid user input, leave menu
      Serial.println(F("Exit"));
      Serial.println();
      delay(10);
      break;

  }
  eeprom_address = 0;                                        //clear eeprom_address
  EEPROM.put(eeprom_address, eeprom_object);                 //store new settings (eeprom_object with structure eeprom_struct) to chip EEPROM
  //at the end of every menu() call
  eeprom_address = 0;                                        //clear eeprom_address
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
  menutimeout = millis() + 10000;                       // time to wait for user to input something

  indata = 0;                                       // initialize
  while (millis() < menutimeout)                        // wait for user to input something
  {
    if (Serial.available() > 0)                     // something came in to BT serial buffer
    {
      delay(100);                                   // give time for everything to come in
      numincoming = Serial.available();             // number of incoming bytes
      for (i = 1; i <= numincoming; i++)            // read in everything
      {
        incoming[i] = Serial.read();                // read from buffer
        if (incoming[i] == 13 || incoming[i] == 10) // ignore CR or LF  -> carriage return & line feed
        {
        }
        else                                        // otherwise
        {
          input = incoming[i] - 48;                 // convert ASCII value to decimal
          indata = indata * 10 + input;             // assemble to get total value
        }
      }
      break;                                        // exit before menutimeout
    }
  }
  Serial.println(indata); delay(10);
}

//-----Get User Input for Character variables etc.-----------------------------------
void charinput() {
  memset(charInput, 0, sizeof(charInput));
  delay(50);
  long timeout;
  timeout = millis() + 10000;                           //length of time to wait for user input
  byte numincoming;                                     //for # of bytes incoming
  //indata = 0;

  while (millis() < timeout) {
    if (Serial.available() > 0) {
      delay(100);
      numincoming = Serial.available();
      //Serial.println(numincoming);

      for (byte i = 0; i <= numincoming; i++) {
        incomingChar[i] = Serial.read();
        if (incomingChar[i] == 13 || incomingChar == 10) {
          //ignore carriage return and linefeed
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

//------get nist time--------------------------------------
void get_nist_time();

//-----time_sync-------------------------------------------
void time_sync();

//-----Set ALARM1 ---------------------------------                             // Alarm 1 is to wake and recieve data from nodes. 1/7/2022 Note that this is NOT functional for the gateway at this time. Continuous power must be provided to gateway as it is always listening
void Set_ALARM_1_Interval() {
  alarm_1_Mins = mins + eeprom_object.ALARM_1_Interval;

  //For troubleshooting
  //Serial.print(F("eeprom_object.ALARM_1_Interval: "));
  //Serial.println(eeprom_object.ALARM_1_Interval);

  //Serial.print(F("alarm_1_Mins: "));
  //Serial.println(alarm_1_Mins);

  if (alarm_1_Mins >= 60) {
    alarm_1_Mins = (alarm_1_Mins % 60);   //if >=60, modulo 60 to get remainding minutes
  }
  RTC.setAlarm(ALM1_MATCH_MINUTES, 0, alarm_1_Mins, 0, 0);

  Serial.print(F("Alarm at: "));
  Serial.print(alarm_1_Mins);
  Serial.println(F("  minutes past the hour."));

}


//-----Return sdCard information----------------------------------------- Does not work correctly as of 12/03/2021. maybe use SdFat library or switch to use flash mem?

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
    // Serial.println(F("Suspending run."));
    // while (1);
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

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB

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
//----- Bluetooth Mode ON Routine -------------------------------------
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
  //Serial.end();
  // bluetooth.write(F("Something here."));
  //while(bluetooth.available()){
  //  Serial.println(F("BT available."));
}

//____________________________________________________________________________________________________________________________________
//____________________________________________________________________________________________________________________________________
//____________________________________________________________________________________________________________________________________
