#pragma once

#ifndef _APPNOSTIC_SC16IS7XX_H_
#define _APPNOSTIC_SC16IS7XX_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else // if ARDUINO >= 100
#include "WProgram.h"
#endif // if ARDUINO >= 100

#if !defined(SC16IS7XX_USE_SPI) && !defined(SC16IS7XX_USE_I2C) ||  defined(SC16IS7XX_USE_SPI) && defined(SC16IS7XX_USE_I2C)
#error "No protocol for SC161S7XX given, define SC16IS7XX_USE_SPI or SC16IS7XX_USE_I2C"
#endif

#if defined(SC16IS7XX_USE_SPI)
    #include <SPI.h>
#elif defined(SC16IS7XX_USE_I2C)
    #include <Wire.h>
    // Device Address

    // A:VDD
    // B:GND
    // C:SCL
    // D:SDA
    #define SC16IS7XX_ADDRESS_AA (0X90)
    #define SC16IS7XX_ADDRESS_AB (0X92)
    #define SC16IS7XX_ADDRESS_AC (0X94)
    #define SC16IS7XX_ADDRESS_AD (0X96)
    #define SC16IS7XX_ADDRESS_BA (0X98)
    #define SC16IS7XX_ADDRESS_BB (0X9A)
    #define SC16IS7XX_ADDRESS_BC (0X9C)
    #define SC16IS7XX_ADDRESS_BD (0X9E)
    #define SC16IS7XX_ADDRESS_CA (0XA0)
    #define SC16IS7XX_ADDRESS_CB (0XA2)
    #define SC16IS7XX_ADDRESS_CC (0XA4)
    #define SC16IS7XX_ADDRESS_CD (0XA6)
    #define SC16IS7XX_ADDRESS_DA (0XA8)
    #define SC16IS7XX_ADDRESS_DB (0XAA)
    #define SC16IS7XX_ADDRESS_DC (0XAC)
    #define SC16IS7XX_ADDRESS_DD (0XAE)
#else
    #error "invalid protocol for SC16157XX. Set SC16IS7XX_PROTOCOL to SC16IS7XX_PROTOCOL_I2C or SC16IS7XX_PROTOCOL_SPI"
#endif


// General Registers
#define SC16IS7XX_REG_RHR (0x00)
#define SC16IS7XX_REG_THR (0X00)
#define SC16IS7XX_REG_IER (0X01)
#define SC16IS7XX_REG_FCR (0X02)
#define SC16IS7XX_REG_IIR (0X02)
#define SC16IS7XX_REG_LCR (0X03)
#define SC16IS7XX_REG_MCR (0X04)
#define SC16IS7XX_REG_LSR (0X05)
#define SC16IS7XX_REG_MSR (0X06)
#define SC16IS7XX_REG_SPR (0X07)
#define SC16IS7XX_REG_TCR (0X06)
#define SC16IS7XX_REG_TLR (0X07)
#define SC16IS7XX_REG_TXLVL (0X08)
#define SC16IS7XX_REG_RXLVL (0X09)
#define SC16IS7XX_REG_IODIR (0X0A)
#define SC16IS7XX_REG_IOSTATE (0X0B)
#define SC16IS7XX_REG_IOINTENA (0X0C)
#define SC16IS7XX_REG_IOCONTROL (0X0E)
#define SC16IS7XX_REG_EFCR (0X0F)

// Special Registers
#define SC16IS7XX_REG_DLL (0x00)
#define SC16IS7XX_REG_DLH (0X01)

// Enhanced Registers
#define SC16IS7XX_REG_EFR (0X02)
#define SC16IS7XX_REG_XON1 (0X04)
#define SC16IS7XX_REG_XON2 (0X05)
#define SC16IS7XX_REG_XOFF1 (0X06)
#define SC16IS7XX_REG_XOFF2 (0X07)

//
#define SC16IS7XX_INT_CTSRTS (0X20)
#define SC16IS7XX_INT_GPIO (0x30)
#define SC16IS7XX_INT_XOFF (0X10)
#define SC16IS7XX_INT_TIMEOUT (0X0c)
#define SC16IS7XX_INT_MODEM (0X00)
#define SC16IS7XX_INT_LINE (0X06)
#define SC16IS7XX_INT_THR (0X02)
#define SC16IS7XX_INT_RHR (0X04)

// Application Related
#define SC16IS7XX_XTAL_FREQ (1843200UL) //common value in datasheet of sc16is7xx
#define SC16IS7XX_PROTOCOL_I2C (0)
#define SC16IS7XX_PROTOCOL_SPI (1)

typedef enum{
    MODEM_PIN_GPIO_0 = 0,
    MODEM_PIN_GPIO_1 = 1
} modem_gpio_t;

class Appnostic_SC16IS7XX : public Stream{
private:
    uint8_t device_protocol;
    uint8_t device_address;
    uint32_t crystal_frequency = SC16IS7XX_XTAL_FREQ;

    // methods that need to be implemented by derived classes
    virtual bool ping();
    virtual void resetDevice();

protected:
    static bool _initialized;

public:
    // constructor
    Appnostic_SC16IS7XX() {};
    ~Appnostic_SC16IS7XX() {};

    // stream reading
    virtual int read();
    virtual int available();
    virtual int peek();
    // virtual String readStringUntil(char teminator);

    // stream writing
    virtual size_t write(uint8_t val);
    virtual size_t write(const uint8_t *buf, size_t size);
    using Print::write; // write(str) and write(buf, size)
    virtual void flush();

    #if defined(SC16IS7XX_USE_I2C)
    bool begin_i2c(uint8_t addr=SC16IS7XX_ADDRESS_AA, int sda=SDA, int scl=SCL);
    
    #elif defined(SC16IS7XX_USE_SPI)
    bool begin_spi(uint8_t cs);

    bool begin_spi();
    #endif

    // configuration
    void setCrystalFrequency(uint32_t frequency);
    uint32_t getCrystalFrequency();

    // registers
    void writeRegister(uint8_t reg_addr, uint8_t val);
    uint8_t readRegister(uint8_t reg_addr);

    // gpio
    virtual void pinMode(uint8_t pin, uint8_t mode);
    virtual void digitalWrite(uint8_t pin, uint8_t state);
    virtual uint8_t digitalRead(uint8_t pin);
    void enableInterruptControl(bool enabled);
    void setPinInterrupt(uint8_t pin, bool enabled);
    uint8_t getPinInterrupt(uint8_t pin);
    int getLastInterruptPin();
    uint8_t isr();
    void setPortState(uint8_t state);
    uint8_t getPortState();
    void setPortMode(uint8_t mode);
    uint8_t getPortMode();
    void setModemPin(modem_gpio_t gpio);
    void setGPIOLatch(bool enabled);
    void enableGpioMode(uint8_t which);
    void disableGpioMode(uint8_t which);
};

#endif