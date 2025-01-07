/**
 * This is an Arduino library for the NOS8007 based on the
 * SC16IS752 dual UART from NXP.
 *
 * It is possible that this library may work with other vendor
 * devices using I2C or SPI but its primary purpose is for
 * Appnostic devices. Pull requests to improve compatibility are
 * welcomed but issues regarding other vendor devices may not receive
 * priority.
 *
 * Credits:
 * @SandboxElectronics for most of the code
 * @TD-er for the SC16IS752 patches
 *
 * Made with love by the Appnostic team!
 */

#include "Appnostic_SC16IS7XX.h"

#ifdef __AVR__
#define WIRE Wire
#define SPI_SS PIN_SPI_SS
#elif defined(ARDUINO_MINIMA) // nos8007 minima r4 support
#define WIRE Wire
#define SPI_SS PIN_SPI_CS
#elif defined(ESP8266) || defined(ESP32) // ESP8266/ESP32
#define WIRE Wire
#define SPI_SS PIN_SPI_SS
#else // Arduino Due
#define WIRE Wire1
#define SPI_SS PIN_SPI_SS
#endif // ifdef __AVR__

bool Appnostic_SC16IS7XX::_initialized = false;

/*** REGISTERS *****************************************************/

/**
 * @brief writes to the register of the device via i2c or spi
 * @param channel
 * @param reg_addr
 * @param val
 */
void Appnostic_SC16IS7XX::writeRegister(uint8_t reg_addr, uint8_t val)
{
    if (device_protocol == SC16IS7XX_PROTOCOL_I2C)
    {
        WIRE.beginTransmission(device_address);
        WIRE.write(reg_addr);
        WIRE.write(val);
        WIRE.endTransmission(true);
    }
    else
    {
        ::digitalWrite(device_address, LOW);
        delayMicroseconds(10);
        SPI.transfer(reg_addr);
        SPI.transfer(val);
        delayMicroseconds(10);
        ::digitalWrite(device_address, HIGH);
    }
}

/**
 * @brief reads a register from the device via i2c or spi
 * @param channel
 * @param reg_addr
 * @return
 */
uint8_t Appnostic_SC16IS7XX::readRegister(uint8_t reg_addr)
{
    uint8_t result = 0;

    if (device_protocol == SC16IS7XX_PROTOCOL_I2C)
    {
        WIRE.beginTransmission(device_address);
        WIRE.write(reg_addr);
        WIRE.endTransmission(false);
        WIRE.requestFrom(device_address, (uint8_t)1);
        result = WIRE.read();
    }
    else
    {
        ::digitalWrite(device_address, LOW);
        delayMicroseconds(10);
        SPI.transfer(0x80 | reg_addr);
        result = SPI.transfer(0xff);
        delayMicroseconds(10);
        ::digitalWrite(device_address, HIGH);
    }

    return result;
}

/*** CONFIG *******************************************************/

/**
 * @brief sets the crystal frequency in hertz. nos8007 has a 14.7456MHz XTAL
 * @note not normally called for nos8007. defaults to 147456000 (Hz)
 * @param frequency
 */
void Appnostic_SC16IS7XX::setCrystalFrequency(uint32_t frequency)
{
    crystal_frequency = frequency;
}

/**
 * @brief gets the xtal frequency in hertz.
 * @return
 */
uint32_t Appnostic_SC16IS7XX::getCrystalFrequency()
{
    return crystal_frequency;
}

/*** DEVICE *******************************************************/

/**
 * @brief derived function to reset the device
 */
void Appnostic_SC16IS7XX::resetDevice()
{
    uint8_t reg;

    reg = readRegister(SC16IS7XX_REG_IOCONTROL << 3);
    reg |= 0x08;
    writeRegister(SC16IS7XX_REG_IOCONTROL << 3, reg);
}

/*** I2C *********************************************************/

/**
 * @brief begins an i2c session for the target address
 * @param addr
 * @return
 */
bool Appnostic_SC16IS7XX::begin_i2c(uint8_t addr)
{

    if ((addr >= 0x48) && (addr <= 0x57))
    {
        device_address = addr;
    }
    else
    {
        device_address = (addr >> 1);
    }
    device_protocol = SC16IS7XX_PROTOCOL_I2C;

    if (_initialized == true)
    {
        return true; // i2c already running
    }

    WIRE.begin(); // start i2c
    WIRE.setClock(400000);
    resetDevice();
    delayMicroseconds(100); // let things settle
    _initialized = true;
    return ping();
}

/**
 * @brief shorthand method to start i2c as nos8007 default address
 * @return
 */
bool Appnostic_SC16IS7XX::begin_i2c()
{
    return begin_i2c(SC16IS7XX_ADDRESS_AA);
}

/*** SPI *********************************************************/

/**
 * @brief sets up SPI
 * @note untested, but assumed working. nos8007 uses i2c
 * @param cs
 * @return
 */
bool Appnostic_SC16IS7XX::begin_spi(uint8_t cs)
{
    device_protocol = SC16IS7XX_PROTOCOL_SPI;
    device_address = cs;

    if (_initialized == true)
    {
        return true; // spi already running
    }

    ::pinMode(device_address, OUTPUT);
    ::digitalWrite(device_address, HIGH);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    SPI.begin();
    resetDevice();
    delayMicroseconds(100); // let things settle
    _initialized = true;
    return ping();
}

/**
 * @brief shorthand to start spi at default CS pin
 * @return
 */
bool Appnostic_SC16IS7XX::begin_spi()
{
    return begin_spi(SPI_SS);
}

/*** GPIO **************************************************/

/**
 * @brief sets the io direction of a gpio pin
 * @param pin 0 - 7
 * @param mode INPUT, OUTPUT
 */
void Appnostic_SC16IS7XX::pinMode(uint8_t pin, uint8_t mode)
{
    uint8_t tmp_iodir;

    tmp_iodir = readRegister(SC16IS7XX_REG_IODIR << 3);

    if (mode == OUTPUT)
    {
        tmp_iodir |= (0x01 << pin);
    }
    else
    {
        tmp_iodir &= (uint8_t) ~(0x01 << pin);
    }

    writeRegister(SC16IS7XX_REG_IODIR << 3, tmp_iodir);
}

/**
 * @brief sets the state of the gpio pin
 * @param pin 0 - 7
 * @param state 0 = LOW, 1 = HIGH
 */
void Appnostic_SC16IS7XX::digitalWrite(uint8_t pin, uint8_t state)
{
    uint8_t tmp_iostate;

    tmp_iostate = readRegister(SC16IS7XX_REG_IOSTATE << 3);

    if (state == 1)
    {
        tmp_iostate |= (0x01 << pin);
    }
    else
    {
        tmp_iostate &= (uint8_t) ~(0x01 << pin);
    }

    writeRegister(SC16IS7XX_REG_IOSTATE << 3, tmp_iostate);
}

/**
 * @brief returns the state of a gpio pin
 * @param pin 0 - 7
 * @return 0 = LOW, 1 = HIGH
 */
uint8_t Appnostic_SC16IS7XX::digitalRead(uint8_t pin)
{
    uint8_t tmp_iostate;

    tmp_iostate = readRegister(SC16IS7XX_REG_IOSTATE << 3);

    if ((tmp_iostate & (0x01 << pin)) == 0)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief sets the interrupt enable register to enable interrupts
 * @note enables all six types of interrupts
 * @param enabled
 */
void Appnostic_SC16IS7XX::enableInterruptControl(bool enabled)
{
    writeRegister(SC16IS7XX_REG_IER << 3, enabled);
}

/**
 * @brief   configures the io interrupt register to generate an interrupt
 *          on pin state change
 * @param pin the pin to configure an interrupt on
 * @param enabled true enables the interrupt, false disables it
 */
void Appnostic_SC16IS7XX::setPinInterrupt(uint8_t pin, bool enabled)
{
    uint8_t tmp_iostate;

    tmp_iostate = readRegister(SC16IS7XX_REG_IOINTENA << 3);

    if (enabled == true)
    {
        tmp_iostate |= (0x01 << pin);
    }
    else
    {
        tmp_iostate &= (uint8_t) ~(0x01 << pin);
    }

    writeRegister(SC16IS7XX_REG_IOINTENA << 3, tmp_iostate);
}

/**
 * @brief returns the interrupt status of a pin
 * @param pin
 * @return
 */
uint8_t Appnostic_SC16IS7XX::getPinInterrupt(uint8_t pin)
{
    uint8_t tmp_iostate;

    tmp_iostate = readRegister(SC16IS7XX_REG_IOINTENA << 3);

    if ((tmp_iostate & (0x01 << pin)) == 0)
    {
        return 0;
    }
    return 1;
}

/**
 * @brief   This will need some sort of manual tracking. Perhaps keep a record
 *          of interrupt-enabled pins and track their changes.
 * @return
 */
int Appnostic_SC16IS7XX::getLastInterruptPin()
{
    return -1;
}

/**
 * @brief   used to determine interrupt source. it should really be
 *          fleshed out better with callbacks.
 */
uint8_t Appnostic_SC16IS7XX::isr()
{
    uint8_t irq_src;

    irq_src = readRegister(SC16IS7XX_REG_IIR << 3);
    // irq_src = (irq_src >> 1);
    // irq_src &= 0x3F;

    switch (irq_src)
    {
    case SC16IS7XX_INT_LINE: // Receiver Line Status Error
        break;
    case SC16IS7XX_INT_TIMEOUT: // Receiver time-out interrupt
        break;
    case SC16IS7XX_INT_RHR: // RHR interrupt
        break;
    case SC16IS7XX_INT_THR: // THR interrupt
        break;
    case SC16IS7XX_INT_MODEM: // modem interrupt;
        break;
    case SC16IS7XX_INT_GPIO: // input pin change of state
        break;
    case SC16IS7XX_INT_XOFF: // XOFF
        break;
    case SC16IS7XX_INT_CTSRTS: // CTS,RTS
        break;
    default:
        break;
    }

    return irq_src;
}

void Appnostic_SC16IS7XX::setPortState(uint8_t state)
{
    writeRegister(SC16IS7XX_REG_IOSTATE << 3, state);
}

uint8_t Appnostic_SC16IS7XX::getPortState()
{
    return readRegister(SC16IS7XX_REG_IOSTATE << 3);
}

void Appnostic_SC16IS7XX::setPortMode(uint8_t mode)
{
    writeRegister(SC16IS7XX_REG_IODIR << 3, mode);
}

uint8_t Appnostic_SC16IS7XX::getPortMode()
{
    return readRegister(SC16IS7XX_REG_IODIR << 3);
}

void Appnostic_SC16IS7XX::setModemPin(modem_gpio_t gpio)
{
    uint8_t tmp_iocontrol;

    tmp_iocontrol = readRegister(SC16IS7XX_REG_IOCONTROL << 3);
    if (gpio == MODEM_PIN_GPIO_0)
    {
        tmp_iocontrol |= 0x02;
    }
    else
    {
        tmp_iocontrol &= 0xFD;
    }
    writeRegister(SC16IS7XX_REG_IOCONTROL << 3, tmp_iocontrol);
}

void Appnostic_SC16IS7XX::setGPIOLatch(bool enabled)
{
    uint8_t tmp_iocontrol;

    tmp_iocontrol = readRegister(SC16IS7XX_REG_IOCONTROL << 3);
    if (enabled == false)
    {
        tmp_iocontrol &= 0xFE;
    }
    else
    {
        tmp_iocontrol |= 0x01;
    }
    writeRegister(SC16IS7XX_REG_IOCONTROL << 3, tmp_iocontrol);
}
