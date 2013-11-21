#ifndef _RADIO_H
#define _RADIO_H

#include "mbed.h"

#define RX_BUFFER_SIZE 4



class Radio
{
public:
    Radio(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName irq);
    void reset();
    void transmit(uint32_t data);
    int getRegister(int address);
    int getStatus();
    
    uint32_t rx_controller;
    uint32_t rx_robot[RX_BUFFER_SIZE];
    int controller;

private:
    void setRegister(int address, int data);
    void receive();
    void clear();

    SPI _spi;
    DigitalOut _csn;
    DigitalOut _ce;
    InterruptIn _irq;
    Timeout clearTimeout;
    unsigned rx_robot_pos;
};



class RadioController
{
public:
    RadioController(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce);
    void reset();
    void transmit(uint32_t data);
    
    int controller;

private:
    void setRegister(int address, int data);
    
    SPI _spi;
    DigitalOut _csn;
    DigitalOut _ce;
};



#endif // _RADIO_H