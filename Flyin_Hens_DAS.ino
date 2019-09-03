//__________________________LIBRARIES
#include <SPI.h>
#include "SdFat.h"   //SD card library
#include <Time.h>
#include <RH_RF95.h> //LoRa Radio library
#include <Adafruit_GPS.h> //GPS library
#include <Servo.h> //Servo Library
#define GPSSerial Serial1   //Hardware Serial using pins 18 and 19
Adafruit_GPS GPS(&GPSSerial);
#include <Wire.h>     //I2C library
#include <Adafruit_MPL3115A2.h> //Altimeter library
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();


//__________________________SERVO
Servo servo_HabR;
Servo servo_HabL;
Servo servo_CDA;
Servo servo_WB;

int HabR_pos = 0;
int HabL_pos = 0;
int CDA_pos = 0;
int Wb_pos = 0;
String dropped = "_";
bool water_drop = false;
bool habitat_drop = false;
bool cda_drop =   false;

//__________________________LEDS
int No_Signal = 32;
int Low_Signal = 34;
int Signal_lock = 36;
int SD_Write = 38;

//__________________________ALTITUDE
float initial_alt = 0;
float feet = 0;
float altm = 0;
String current_alt;

//__________________________GPS
#define GPSECHO false
uint32_t timer = millis();
char radiopacket[13];
String altitude;

//__________________________SD
#define FILE_BASE_NAME "DASLOG"  // Log file base name.  Must be six characters or less.
SdFat sd;  // File system object.
SdFile file;  // Log file.
uint32_t logTime;  // Time in micros for next data record.
const int cardSelect = 10;
const uint32_t SAMPLE_INTERVAL_MS = 100;

//__________________________RADIO
#define RFM95_CS 9
#define RFM95_RST 8
#define RFM95_INT 3
#define RF95_FREQ 433
RH_RF95 rf95(RFM95_CS, RFM95_INT);
String response;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

//__________________________
ArduinoOutStream cout(Serial);

//------------------------------------------------------------------------------

//==============================================================================
// User functions.  Edit writeHeader() and logData() for your SD Logging requirements.
//------------------------------------------------------------------------------
String getAltitude() {
  altm = baro.getAltitude();
  feet = 3.28084 * altm;
  feet = feet - initial_alt;
  altitude = String(feet);
  return altitude;
}

//------------------------------------------------------------------------------
//Set time of file creation
void dateTime(uint16_t* date, uint16_t* time) {


  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE((GPS.year,DEC), (GPS.month,DEC), (GPS.day,DEC));

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(GPS.hour, GPS.minute, GPS.seconds);
}
//------------------------------------------------------------------------------

// Write data header.
void writeHeader() {
  file.print(F("Time"));
  file.print(F(",altitude(feet)"));
  file.print(F(",latitude"));
  file.print(F(",longitude"));
  file.print(F(",speed(knots)"));
  file.print(F(",angle"));
  file.print(F(",dropped?"));
  file.println();
}
//------------------------------------------------------------------------------
// Log a data record.
void logData() {
  current_alt = getAltitude();
  int gst = GPS.hour;  //Global Standard Time
  int est = gst - 5; //Eastern Standard Time
  file.print(est);    //Log Time
  file.write(':');
  file.print(GPS.minute);
  file.write(':');
  file.print(GPS.seconds);
  file.write('.');
  file.print(GPS.milliseconds);

  file.write(',');
  file.print(current_alt);  //Altitude
  file.write(',');
  file.print(GPS.latitude, 4); //Latitude
  file.print(GPS.lat);
  file.write(',');
  file.print(GPS.longitude, 4); //Longitude
  file.print(GPS.lon);
  file.write(',');
  file.print(GPS.speed);
  file.write(',');
  file.print(GPS.angle);
   file.write(',');
  file.print(dropped);
  if (dropped != "_") {
    dropped = "_";
  }
  digitalWrite(SD_Write, HIGH);

  file.println();
  SdFile::dateTimeCallback(dateTime);  //set time of last entry
}
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
//------------------------------------------------------------------------------
//GPS Fix and SD Write indicator lights
void lights() {
  if (GPS.fix == 0) {      ///Shuffle through colors
    digitalWrite(No_Signal, HIGH);
    digitalWrite(Low_Signal, LOW);     ///Red
    digitalWrite(Signal_lock, LOW);
    digitalWrite(SD_Write, LOW);
    delay(250);
    digitalWrite(No_Signal, LOW);
    digitalWrite(Low_Signal, HIGH);  // Yellow
    delay(250);
    digitalWrite(Low_Signal, LOW);  //Green
    digitalWrite(Signal_lock, HIGH);
    delay(250);
    digitalWrite(Signal_lock, LOW); //Blue
    digitalWrite(SD_Write, HIGH);
    delay(250);
    digitalWrite(Signal_lock, HIGH); //Green
    digitalWrite(SD_Write, LOW);
    delay(250);
    digitalWrite(Low_Signal, HIGH); //Yellow
    digitalWrite(Signal_lock, LOW);
    delay(250);
  }
  if ((GPS.fix != 0) & (int)GPS.fixquality == 1) {
    digitalWrite(No_Signal, LOW);
    digitalWrite(Low_Signal, HIGH);
    digitalWrite(Signal_lock, LOW);
    digitalWrite(SD_Write, LOW);
  }
  if ((GPS.fix != 0) & (int)GPS.fixquality == 2) {
    digitalWrite(No_Signal, LOW);
    digitalWrite(Low_Signal, LOW);
    digitalWrite(Signal_lock, HIGH);
    digitalWrite(SD_Write, LOW);
  }
}
void setup()
{
  ////-------------------------------------------------------------------------//////
  ///////////////////ASSIGN I/O PINS/////////////////
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";
  pinMode(19, INPUT_PULLUP);
  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_CS, HIGH);
  pinMode(cardSelect, OUTPUT);
  digitalWrite(cardSelect, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);//Pulling Reset High Enables GPS
  delay(10);
  pinMode(No_Signal, OUTPUT);
  digitalWrite(No_Signal, HIGH);
  pinMode(Low_Signal, OUTPUT);
  digitalWrite(Low_Signal, HIGH);
  pinMode(Signal_lock, OUTPUT);
  digitalWrite(Signal_lock, HIGH);
  pinMode(SD_Write, OUTPUT);
  digitalWrite(SD_Write, LOW);

  ////------------------------------------------------------------------------------//////
  ///////////////////////////BEGIN/////////////////////////
  Serial.begin(115200);  //Serial
  delay(100);
  Wire.begin();        //I2C
  delay(100);
  GPS.begin(9600);     //GPS
  Serial.println("FLYING HENS DATA ACQUISITION SYSTEM");

  ////////////////SD CARD/////////////////////
  if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
    digitalWrite(SD_Write, HIGH);
    digitalWrite(No_Signal, HIGH);
    digitalWrite(Low_Signal, LOW);
    digitalWrite(Signal_lock, LOW);
    sd.initErrorHalt();
  }
  ////////////////ALTIMETER///////////////////
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    digitalWrite(Low_Signal, HIGH);  //Triggered as error warning
    digitalWrite(No_Signal, HIGH);
    digitalWrite(SD_Write, LOW);
    digitalWrite(Signal_lock, LOW);
    return;
  }

  ////------------------------------------------------------------------------------/////
  ///////////////////GPS/////////////////
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY); //Sets output to only RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //Sets the output to 1/second. If you want you can go higher/lower
  GPS.sendCommand(PGCMD_ANTENNA); //Can report if antenna is connected or not
  delay(500);
  // GPSSerial.println(PMTK_Q_RELEASE);
  ////------------------------------------------------------------------------------/////

  ///////////////////SD CARD/////////////////
  // Find an unused file name.
  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }
  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else {
      digitalWrite(SD_Write, HIGH);
      digitalWrite(No_Signal, HIGH);
      digitalWrite(Low_Signal, LOW);
      digitalWrite(Signal_lock, LOW);
      error("Can't create file name");


    }
  }
  SdFile::dateTimeCallback(dateTime);
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL)) {
    digitalWrite(SD_Write, HIGH);
    digitalWrite(No_Signal, HIGH);
    digitalWrite(Low_Signal, LOW);
    digitalWrite(Signal_lock, LOW);
    error("file.open");
  }
  // Read any Serial data.
  do {
    delay(10);
  }
  while (Serial.available() && Serial.read() >= 0);
  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  writeHeader();    // Write data header.
  // Start on a multiple of the sample interval.
  logTime = micros() / (1000UL * SAMPLE_INTERVAL_MS) + 1;
  logTime *= 1000UL * SAMPLE_INTERVAL_MS;
  ////------------------------------------------------------------------------------/////
  ///////////////////RADIO/////////////////
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    digitalWrite(Signal_lock, HIGH);  //Triggered as error warning
    digitalWrite(No_Signal, HIGH);
    digitalWrite(SD_Write, LOW);
    digitalWrite(Low_Signal, LOW);
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);


  /////------------------------------------------------------------------
  ///////////////////SERVOS/////////////////
  servo_HabR.attach(4);  //assign Servo Pin locations
  servo_HabL.attach(5);
  servo_WB.attach(6);
  servo_CDA.attach(7);

  servo_HabR.write(180); //180
  servo_HabL.write(0); //180      ///Initialize payload
  servo_WB.write(0);            //servo positions
  servo_CDA.write(135); //135
  /////------------------------------------------------------------------

}

void loop()
{
  ////------------------------------------------------------------------------------/////
  ///////////////////GPS/////////////////
  //Serial.print("here");
  char c = GPS.read();  //read GPS characters
  if (GPSECHO)
    if (c) GPSSerial.print(c);  //print buffer if GPSECHO==true
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))  //parse buffer
      return;
  }
  //   if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
  if (millis() - timer > 1000) {  //Check buffer every second
    lights(); //change light status every second
    timer = millis(); // reset the timer
    if (GPS.fix) {

      logData();  //log data to SD card
      if (!file.sync() || file.getWriteError()) {
        error("write error");
      }

      if (Serial.available()) {
        // Close file and stop.
        file.close();
        Serial.println(F("Done"));
        SysCall::halt();
      }

      sendAlt();  //Transmit altitude to ground station
    }
//**********CHANGED TO PASS INSPECTION*************
    //Uncomment these two lines to immediately acquire altitude data
      //without waiting for GPS lock    
 // logData();  
 // sendAlt(); 
  }
//*************************************************
  ///////////////////DROP RECEIVED?/////////////////
  if (rf95.recv(buf, &len)) {
    Serial.print("Got reply: ");
    Serial.println((char*)buf);
    response = buf;
    drop(response);  //send drop command to appropiate servo
  }
  //  getAltitude();
  //  sendAlt();
}//loop

void sendAlt() { //Send altitude in feet to Ground Station every second
  dtostrf(feet, 12, 4, radiopacket);
  Serial.println(radiopacket);  //print current altitude
  radiopacket[12] = 0;

  //Serial.println("Sending...");
  //delay(10);
  rf95.send((uint8_t *)radiopacket, 13);
}

void drop (String response) { //Drop appropriate payload when button pressed on ground station
  if (response == "Hab") {
    Serial.println("Dropped Habitats");
    servo_HabR.write(0);
    delay(10);
    servo_HabL.write(180);

    dropped = "Habitats";
  }
  if (response == "WB") {
    Serial.println("Dropped Water Bottle");
    servo_WB.write(180);
    dropped = "Water";
  }
  if (response == "CDA") {
    Serial.println("dropped CDA");
    servo_CDA.write(20);
    dropped = "CDA";
  }
  else {}
  Serial.print("RSSI: "); //print received signal strength
  Serial.println(rf95.lastRssi(), DEC);
}
