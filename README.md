# Appnostic SC16IS7XX Arduino Library

This is a library for the SC16IS750/SC16IS751/SC16IS752 series of UART interfaces by NXP as used on the NOS8007 interface module. Although the library is designed with NOS8007 in mind it may work with other implementations requiring only basic changes in I2C addresses and crystal frequencies.

Made with love by the team at Appnostic!

To install, use the Arduino IDE Library Manager.

## Using This Library With Other Hardware

This library was tested with Appnostic's NOS8007 UART interface module with the NOS10001 baseboard on Arduino Uno and Minima R4 variants. As the library interfaces with the SC16IS75XX via I2C or SPI it should communicate just fine with other hardware variants provided the following is observed:

- The default crystal frequency is 14.7456MHz (147456000Hz). If a different crystal is used you must call the `setCrystalFrequency(frequency)` method before `setBaudRate` so that the correct divisor can be calculated.
- If using interrupts take note that only pins 2 and 3 on the Arduino Uno facilitate external interrupts. The Minima has all digital pins available.
- If using SPI the default pin is the CS pin as denoted by the board support package, usually pin 10 on the Uno.
- YMMV for STM32 or ESP32 hardware. Pull requests welcomed.

## Submitting Issues

This library is mainly written to support the NOS8007. Issues submitted for other hardware may receive less attention as it is based on our time and ability to test on the specific hardware.

## Submitting Pull Requests

If you woild like to submit patches to support your specific hardware and use cases please feel free :)


