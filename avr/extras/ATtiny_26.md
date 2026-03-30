# ATtiny26

| Pinout diagram                       | Minimal setup schematic                            |
|--------------------------------------|----------------------------------------------------|
|<img src="Pinout_26.png" width="350"> | <img src="ATtiny26_minimal_setup.png" width="250"> |


| Specification                    | ATtiny26                |
|----------------------------------|-------------------------|
| Bootloader support               | No                      |
| Flash available user             | 2048 bytes              |
| RAM                              | 128 bytes               |
| EEPROM                           | 128 bytes               |
| GPIO Pins                        | 16 (15 usable)          |
| ADC Channels                     | 11                      |
| Differential ADC                 | 12 pairs, 4 with 20x gain for offset, 1x-only channel, others selectable 1x/20x |
| PWM Channels                     | 2                       |
| Interfaces                       | USI, high-speed timer   |
| Int. Oscillator (MHz)            | 16, 8, 4, 2, 1          |
| External Crystal                 | All Standard            |
| External Clock                   | All Standard            |
| Int. WDT Oscillator              | 128 kHz                 |
| LED_BUILTIN                      | PIN_PB6                 |

## Table of contents
- [Overview](#overview)
- [No bootloader support](#no-bootloader-is-possible)
- [Internal oscillator calibration](#internal-oscillator-calibration)
- [Features](#features)
  - [PLL clock](#pll-clock)
  - [I2C support](#i2c-support)
  - [SPI support](#spi-support)
  - [UART (Serial) support](#uart-serial-support)
  - [Tone](#tone-support)
  - [Servo](#servo-support)
- [ADC features](#adc-features)
  - [ADC Reference options](#adc-reference-options)
  - [Internal sources](#internal-sources)
  - [Differential ADC](#differential-adc)
  - [ADC differential pair matrix](#adc-differential-pair-matrix)

### Overview
The ATtiny26 is the predecessor to the ATtiny261/461/861 family, sharing a similar architecture and differential ADC. As one of the earliest ATtiny devices, its peripheral set is less advanced than its successors.

### No bootloader support
Self programming is not supported on these parts, making it impossible to use a bootloader.

### Internal oscillator calibration
The internal 8 MHz oscillator and 16 MHz PLL is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail.

TinyCore provides an oscillator calibration sketch, but this is not compatible with the ATtiny26, due to the limited PCINT functionality this chip offers. However the [avrCalibrate](https://github.com/felias-fogg/avrCalibrate) library supports the ATtiny26, and avrCalibrate can also calibrate the internal voltage reference. When the new calibration value for OSCCAL is calculated and stored to EEPROM, or can be retrieved like so, given that the new calibration value is stored in the last EEPROM byte:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL = cal;
```

## Features

### PLL clock
The ATtiny26 features an on-chip PLL clocked from the internal oscillator, running at a nominal frequency of 64 MHz when enabled. It can be used in two ways, either independently or in combination.  
The system clock can be derived from one quarter of the PLL output, providing a 16 MHz clock source without an external crystal. This option inherits the accuracy limitations of the internal oscillator driving the PLL. Timer1 can be clocked directly from the PLL, enabling high-speed PWM and other timer functions independent of the system clock speed.

### I2C support
The ATtiny26 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**. The small flash of these parts may require use of more tightly optimized (and API-incompatible) library.

### SPI support
The ATtiny26 does not feature a dedicated SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library. The small flash of these parts may require use of more tightly optimized (and API-incompatible) library.

### UART (Serial) support
The ATtiny26 does not feature a hardware UART. When operating from the internal oscillator, clock calibration may be necessary to achieve sufficient timing accuracy for reliable serial communication.

The core provides a built-in software serial implementation exposed as `Serial`, using the Analog Comparator pins and their dedicated interrupt to avoid conflicts with libraries that rely on pin change interrupts (PCINTs). The default pin assignment is AIN0 for TX and AIN1 for RX. Being a software implementation, `Serial` cannot transmit and receive simultaneously. Note that both an upper and a lower baud rate limit apply.

The `SoftwareSerial` library is not supported on the ATtiny26, as it relies on PCINTs, which are not readily usable on this device due to the absence of a `PCMSK` register.

### Tone support
`tone()` is implemented using Timer1. For best results, use PB0 as the tone output pin, where `tone()` drives Timer1's output compare unit directly rather than toggling the pin via interrupt, extending the usable frequency range into the MHz region.

### Servo support
The Servo library is not supported on the ATtiny26. The available flash is too limited and the library too large to leave meaningful space for user code.

## ADC features
The ATtiny261/461/861 has a surprisingly sophisticated ADC with many differential channels, most with selectable gain. These are available through analogRead. When used to read a pair of analog pins in differential mode, the ADC normally runs in unipolar mode: The voltage on the positive pin must be higher than that on the negative one, but the difference is measured to the full precision of the ADC. It can be put into bipolar mode, where the voltage on the negative side can go below the voltage on the positive side and generate meaningful measurements (it will return a signed value, which costs 1 bit of accuracy for the sign bit). This can be enabled by calling the helper function `setADCBipolarMode(true or false)`.

### ADC Reference options
The ATtiny26 provides two internal reference voltages, one of which supports connection of an external capacitor on the AREF pin for improved stability. An external reference voltage and the supply voltage (Vcc) are also available as reference sources.

| Reference Option   | Reference Voltage           | Uses AREF Pin        | Aliases/synonyms                         |
|--------------------|-----------------------------|----------------------|------------------------------------------|
| `DEFAULT`          | Vcc                         | No, pin available    |                                          |
| `EXTERNAL`         | Voltage applied to AREF pin | Yes, ext. voltage    |                                          |
| `INTERNAL2V56`     | Internal 2.56V reference    | No, pin available    | `INTERNAL2V56_NO_CAP` `INTERNAL2V56NOBP` |
| `INTERNAL2V56_CAP` | Internal 2.56V reference    | Yes, w/cap. on AREF  |                                          |

### Internal sources
| Voltage Source  | Description                            |
|-----------------|----------------------------------------|
| ADC_INTERNAL1V1 | Reads the 1.18v bandgap reference <br\> which can't be used as AREF |
| ADC_GROUND      | Reads ground - for offset correction   |

### Differential ADC
The ATtiny26 provides 20 differential channel configurations. Four of these measure each possible negative pin against itself, intended for gain stage offset calibration at 20x gain only. One pair is available at 1x gain only, while all remaining pairs offer both 1x and 20x gain options.

| Positive   | Negative   |   Gain  | Channel| Name 1x/20x mode | Notes            |
|------------|------------|---------|--------|------------------|------------------|
| ADC0 (PA0) | ADC1 (PA1) |     20x |   0x0B | DIFF_A0_A1_20X   |                  |
| ADC0 (PA0) | ADC1 (PA1) |      1x |   0x0C | DIFF_A0_A1_1X    |                  |
| ADC1 (PA1) | ADC1 (PA1) |     20x |   0x0D | DIFF_A1_A1_20X   | For offset cal.  |
| ADC2 (PA2) | ADC1 (PA1) |     20x |   0x0E | DIFF_A2_A1_20X   |                  |
| ADC2 (PA2) | ADC1 (PA1) |      1x |   0x0F | DIFF_A2_A1_1X    |                  |
| ADC2 (PA2) | ADC3 (PA4) |      1x |   0x10 | DIFF_A2_A3_1X    |                  |
| ADC3 (PA4) | ADC3 (PA4) |     20x |   0x11 | DIFF_A3_A3_20X   | For offset cal.  |
| ADC4 (PA5) | ADC3 (PA4) |     20x |   0x12 | DIFF_A4_A3_20X   |                  |
| ADC4 (PA5) | ADC3 (PA4) |      1x |   0x13 | DIFF_A4_A3_1X    |                  |
| ADC4 (PA5) | ADC5 (PA6) |     20x |   0x14 | DIFF_A4_A5_20X   |                  |
| ADC4 (PA5) | ADC5 (PA6) |      1x |   0x15 | DIFF_A4_A5_1X    |                  |
| ADC5 (PA6) | ADC5 (PA6) |     20x |   0x16 | DIFF_A5_A5_20X   | For offset cal.  |
| ADC6 (PA7) | ADC5 (PA6) |     20x |   0x17 | DIFF_A6_A5_20X   |                  |
| ADC6 (PA7) | ADC5 (PA6) |      1x |   0x18 | DIFF_A6_A5_1X    |                  |
| ADC8 (PB5) | ADC9 (PB6) |     20x |   0x19 | DIFF_A8_A9_20X   |                  |
| ADC8 (PB5) | ADC9 (PB6) |      1x |   0x1A | DIFF_A8_A9_1X    |                  |
| ADC9 (PB6) | ADC9 (PB6) |     20x |   0x1B | DIFF_A9_A9_20X   | For offset cal.  |
| ADC10(PB7) | ADC9 (PB6) |     20x |   0x1C | DIFF_A10_A9_20X  |                  |
| ADC10(PB7) | ADC9 (PB6) |      1x |   0x1D | DIFF_A10_A9_1X   |                  |

#### ADC Differential pair matrix
|  N\P  |   0   |   1   |   2   |   3   |   4   |   5   |   6   |   8   |   9   |  10   |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|   0   |       |       |       |       |       |       |       |       |       |       |
|   1   | 1/20x | 20x*  | 1/20x |       |       |       |       |       |       |       |
|   2   |       |       |       |       |       |       |       |       |       |       |
|   3   |       |       |   1x  | 20x*  | 1/20x |       |       |       |       |       |
|   4   |       |       |       |       |       |       |       |       |       |       |
|   5   |       |       |       |       | 1/20x | 20x*  | 1/20x |       |       |       |
|   6   |       |       |       |       |       |       |       |       |       |       |
|   9   |       |       |       |       |       |       |       | 1/20x | 20x*  | 1/20x |

`*` this option is used to measure the offset of that gain stage (applicable to that negative pin only), in order to correct offset gain to within 1 LSB. Doing this is the responsibility of the user.
