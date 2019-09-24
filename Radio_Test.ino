#include <RH_RF95.h> //LoRa Radio library

/* Radio Module Pinout (Not Arduino Pin Numbers)
    PIN 1 - GND
    PIN 2 - MISO        : SPI Data Output
    PIN 3 - MOSI        : SPI Data Input
    PIN 4 - SCK         : SPI Clock Input
    PIN 5 - NSS         : SPI Chip Select Input
    PIN 6 - RESET       : Reset Trigger Input
    PIN 7 - DIO5        : Digital I/O, software configured
    PIN 8 - GND
    PIN 9 - ANT         : RF Signal Output
    PIN 10 - GND
    PIN 11 - DIO3       : Digital I/O, software configured
    PIN 12 - DIO4       : Digital I/O, software configured
    PIN 13 - 3.3V       : Supply Voltage
    PIN 14 - DIO0       : Digital I/O, software configured
    PIN 15 - DIO1       : Digital I/O, software configured
    PIN 16 - DIO2       : Digital I/O, software configured
 */