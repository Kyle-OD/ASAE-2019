#include <RH_RF95.h> //LoRa Radio library

/* Radio Module Pinout (Not Arduino Pin Numbers)
    VIN - Power in
    GND - Ground
    EN - Enable Pin, High on default, Low to cut power
    G0 - GPIO 0 Pin, used for IRQ (Interrupt Request) used for interrupt request
        from radio to microcontroller
    SCK - SPI Clock Pin
    MISO -  Master In Slave Out pin, data sent from processor to radio,
        3.3V logic level
    MOSI - Master Out Slave In pin, data sent from processor to radio
    CS - Chip Select pin, drop to low for SPI transaction
    RST - Reset pin, pulled high by default for reset, pull Low to turn on
    G1-G5 - GPIO pins
 */