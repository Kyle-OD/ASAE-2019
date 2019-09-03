

// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX
/////////////////////////////////
#include <Wire.h> 
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_7segment plane_alt = Adafruit_7segment();  //initialize main altitude
Adafruit_7segment cda_alt = Adafruit_7segment();  //initialize cda altitude
Adafruit_7segment water_alt = Adafruit_7segment();  //initialize water altitude
Adafruit_7segment habitat_alt = Adafruit_7segment();  //initialize habitat altitude
////////////////////////Seven Segment Display Library
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 2
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13
String reply;               //reply from plane
const int cda_pin = 8;      //pin of button
const int cda_led = 9;
const int habitat_pin = 4;  //pin of button
const int habitat_led = 5;
const int water_led = 7;    
const int water_pin = 6;    //pin of button
int current_height=0.0;      //current altitude of plane in floating point
int initial_height;
bool cda_button=LOW;           //status of cda button
bool habitat_button=LOW;       //status of habitat button
bool water_button=LOW;         //status of water button
bool cda_drop= false;       //cda button pressed?
bool water_drop =false;     //water button pressed?
bool habitat_drop =false;   //habitat button pressed?
bool cda_dropped = false;
bool water_dropped = false; 
bool habitat_dropped = false;
int cda_height = 0;
int water_height = 0;
int habitat_height = 0;
void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(cda_pin, INPUT);  
  pinMode(cda_pin, INPUT_PULLUP);
  pinMode(habitat_pin, INPUT);
  pinMode(habitat_pin, INPUT_PULLUP);
  pinMode(water_pin, INPUT);   
  pinMode(water_pin, INPUT_PULLUP);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(cda_led, OUTPUT);
  pinMode(water_led, OUTPUT);
  pinMode(habitat_led, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  //while (!Serial);  //COMMENT LINE IF NOT CONNECTING ARDUINO TO COMPUTER
  #ifndef __AVR_ATtiny85__
  Serial.begin(9600);
  delay(100);
  #endif
  plane_alt.begin(0x70); //address of main altitude display
  cda_alt.begin(0x71); //address of CDA altitude display
  water_alt.begin(0x72); //address of water altitude display
  habitat_alt.begin(0x73); //address of habitat altitude display
  //Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(water_led, LOW);
  digitalWrite(cda_led, LOW);
  digitalWrite(habitat_led, LOW);
  
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
 
    plane_alt.print(0000);
    /*plane_alt.writeDigitRaw(0,0x02);
    
    plane_alt.writeDigitRaw(0,0x04);
    plane_alt.writeDigitRaw(0,0x06);
    plane_alt.writeDigitRaw(0,0x08);
    plane_alt.writeDigitRaw(0,0x10);
    */
    cda_alt.print(10000);
    water_alt.print(10000);
    habitat_alt.print(10000);
    //plane_alt.print(42414,HEX);
    plane_alt.writeDisplay();
    cda_alt.writeDisplay();
    
    water_alt.writeDisplay();
    
    habitat_alt.writeDisplay();
    
    cda_alt.blinkRate(2);
    water_alt.blinkRate(2);
   
    habitat_alt.blinkRate(2);
    
}

void loop()
{
    
     
  cda_button = digitalRead(cda_pin);                      //read button pin
   if(cda_button == LOW && cda_drop == false){           //if button is pressed for first time
        uint8_t cda[] = "CDA";                            //create string to send to plane
        rf95.send(cda, sizeof(cda));                      //send drop command to plane
        rf95.waitPacketSent();                            //wait for command to send
        Serial.println("CDA dropped");                    //write to serial monitor that cda command has been sent 
        cda_drop = true;                                  //set button pressed equal to true
        //digitalWrite(cda_led, HIGH);
       // cda_height = current_alt;
        cda_alt.print(cda_height);                       //display current altitude on CDA display
        cda_alt.writeDisplay();                          //write altitude the CDA was dropped at to seven segment display
        delay(1000);                                       //delay a 1/10 of a second
      }
          else{}                                                //if button has not been pressed do nothing
  
 
  water_button = digitalRead(water_pin);                  //same as above
   if(water_button== LOW && water_drop == false){
        uint8_t water[] = "WB";
        rf95.send(water, sizeof(water));
        rf95.waitPacketSent();
        Serial.println("Water dropped");
        water_drop = true;
        //digitalWrite(water_led, HIGH);  
      //  water_height = current_alt;
        water_alt.print(water_height);
        water_alt.writeDisplay();
        delay(100);  
      }
  else{}
  
 habitat_button = digitalRead(habitat_pin);              //read button pin
   if(habitat_button == LOW && habitat_drop == false){   //if button is pressed for first time
        uint8_t habitat[] = "Hab";                        //create string to send to plane
        rf95.send(habitat, sizeof(habitat));              //send drop command to plane
        rf95.waitPacketSent();                            //wait for command to send
        Serial.println("Habitat dropped");                //write to serial monitor that cda command has been sent
        habitat_drop = true;                              //set button pressed equal to true
        //digitalWrite(habitat_led, HIGH);
     //   habitat_height = current_alt;
        habitat_alt.print(habitat_height);                   //display current altitude on habitat display
        habitat_alt.writeDisplay();                       //write altitude the habitat was dropped at to seven segment display
          delay(100);                                       //delay a 1/10 of a second
      }
  else{}                                                  //if button has not been pressed do nothing
  
 
  if (rf95.available()){                                  //If radio is receiving a message
  //  Serial.println("Receiving");                          //alert user 
    // Should be a message for us now                   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];                 //create buffer array for bytes
   // uint8_t len = sizeof(buf);
    uint8_t len = 13;                                     //assign length(bytes) of return message
    cda_alt.blinkRate(0);
    water_alt.blinkRate(0);
    habitat_alt.blinkRate(0);
    
    if (rf95.recv(buf, &len))                             //if message was received and length was correct
    {
      //digitalWrite(LED, HIGH);
     // RH_RF95::printBuffer("Received: ", buf, len);
      //Serial.print("Got: ");                            //((char*)buf) is a character buffer containing the altitude transmitted from the plane
      String(current_alt) = ((char*)buf);                         //current_alt is a decimal number that is assigned to the number received
      
      if (initial_height ==0){
        initial_height = round(current_alt.toFloat());
      }
      current_height = round(current_alt.toFloat())-initial_height;
      
      //plane_alt.print(current_alt.toFloat());
      plane_alt.print(current_height);
      plane_alt.writeDisplay();
      cda_height = current_height;
      water_height = current_height;
      habitat_height = current_height;
      
      Serial.println(current_alt);    //print current altitude in the serial monitor
       //Serial.print("RSSI: ");                          //RSSI is a measure of signal loss
      //Serial.println(rf95.lastRssi(), DEC);             //print signal loss for last message
    
    }
    //for (uint16_t counter = 0; counter < 9999; counter++) {
   
      
    //delay(10);
  //}

                           //assign current altitude to main display
                                //display current altitude on seven segment display
  }
}
