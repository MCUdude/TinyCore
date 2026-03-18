# ATtiny 24/44/84(a)
![x4 pin mapping](Pinout_x4.png "Arduino Pin Mapping for ATtiny x4-family")

| Specification                    | ATtiny84         | ATtiny44         | ATtiny24         |
|----------------------------------|------------------|------------------|------------------|
| Bootloader (occupies 256 bytes)  | Urboot           | Urboot           | Urboot           |
| Flash available user             | 7936 / 8192 bytes| 3840 / 4096 bytes| 1792 / 2048 bytes|
| RAM                              | 512 bytes        | 256 bytes        | 128 bytes        |
| EEPROM                           | 512 bytes        | 256 bytes        | 128 bytes        |
| GPIO Pins                        | 11 + RESET       | 11 + RESET       | 11 + RESET       |
| ADC Channels                     | 12 (incl RST)    | 12 (incl RST)    | 12 (incl RST)    |
| Differential ADC                 | 1/20x gain       | 1/20x gain       | 1/20x gain       |
| PWM Channels                     | 4: PA5-7, PB2    | 4: PA5-7, PB2    | 4: PA5-7, PB2    |
| Interfaces                       | USI              | USI              | USI              |
| Bootloader pins (RX/TX)          | PA2/PA1          | PA2/PA1          | PA2/PA1          |
| Int. Oscillator (MHz)            | 8, 4, 2, 1       | 8, 4, 2, 1       | 8, 4, 2, 1       |
| External Crystal                 | All standard     | All standard     | All standard     |
| External Clock                   | All standard     | All standard     | All standard     |
| Int. WDT Oscillator              | 128 kHz          | 128 kHz          | 128 kHz          |
| LED_BUILTIN                      | PIN_PB2          | PIN_PB2          | PIN_PB2          |

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny24/44/84, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 1792, 3840, or 7936 bytes available for user code on the ATtiny24, 44, and 84 respectively. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The default serial upload pins for these chips are PA1 (TX) and PA2 (RX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader can be configured to apply an oscillator correction factor. This is exposed as a Tools menu option, with adjustable compensation ranging from -5.00% to +5.00%.

### Internal oscillator calibration
The internal 8 MHz oscillator is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail.

To address this, TinyCore provides an [Oscillator calibration sketch](../libraries/TinyCore/examples/OscillatorCalibration/OscillatorCalibration.ino) that calculates a corrected OSCCAL value based on characters received over UART. It uses the default UART pins, **TX = PA1** and **RX = PA2**. Before uploading the sketch, ensure the target is running from its internal 8 MHz oscillator and that EEPROM preservation is enabled. Once uploaded, open the serial monitor at 115200 baud, select "No line ending", and repeatedly send the character `x`. After a few attempts, readable text should begin to appear in the serial monitor. Once the calibration value has stabilised, it is automatically stored in the last byte of EEPROM for future use. This value is not loaded automatically and must be applied explicitly in your sketch:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL = cal;
```

Another approach is to use the [avrCalibrate](https://github.com/felias-fogg/avrCalibrate) library, which uses a host microcontroller along with the target to perform the calibraion. avrCalibrate can also calibrate internal voltage references.


## Features

### PWM frequency
TC0 is always run in Fast PWM mode: We use TC0 for millis, and phase correct mode can't be used on the millis timer - you need to read the count to get micros, but that doesn't tell you the time in phase correct mode because you don't know if it's upcounting or downcounting in phase correct mode.

| F_CPU  | F_PWM<sub>TC0</sub> | F_PWM<sub>TC1</sub>   | Notes                        |
|--------|---------------------|-----------------------|------------------------------|
| 1  MHz | 1/8/256=     488 Hz |  1/8/256=      488 Hz |                              |
| 2  MHz | 2/8/256=     977 Hz |  2/8/256=      977 Hz |                              |
| <4 MHz | x/8/256= 488 * x Hz |  x/8/512=  244 * x Hz | Phase correct TC1            |
| 4  MHz | 4/8/256=    1960 Hz |  4/8/512=      977 Hz | Phase correct TC1            |
| <8 MHz | x/64/256= 61 * x Hz |  x/8/512=  244 * x Hz | Between 4 and 8 MHz, the target range is elusive | Phase correct TC1 |
| 8  MHz | 8/64/256=    488 Hz |  8/64/256=     488 Hz |                              |
| >8 MHz | x/64/256= 61 * x Hz |  x/64/256=  61 * x Hz |                              |
| 12 MHz | 12/64/256=   735 Hz | 12/64/256=     735 Hz |                              |
| 16 MHz | 16/64/256=   977 Hz | 16/64/256=     977 Hz |                              |
|>16 MHz | x/64/256= 61 * x Hz |  x/64/512=  31 * x Hz | Phase correct TC1            |
| 20 MHz | 20/64/256=  1220 Hz | 20/64/512=     610 Hz | Phase correct TC1            |

Where speeds above or below a certain speed are specified, it's implied that the other end of the range is the next marked value. So >16 in that table is for 16-20 MHz clocks. The formula is given as a constant times x where x is expressed as MHz (the division above gets the time in megahertz - in the interest of readability I did not include the MHz to Hz conversion - I'm sure you all know how to multiply by a million)

Phase correct PWM counts up to 255, turning the pin off as it passes the compare value, updates it's double-buffered registers at TOP, then it counts down to 0, flipping the pin back as is passes the compare value. This is considered preferable for motor control applications, though the "Phase and Frequency Correct" mode is better if the period is ever adjusted by a large amount at a time, because it updates the doublebuffered registers at BOTTOM, and thus produces a less problematic glitch in the duty cycle, but doesn't have any modes that don't require setting ICR1 too.

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C support
The ATtiny24/44/84 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI support
The ATtiny24/44/84 does not feature a dedicated SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library.

Only the built-in SPI library is officially supported. If you encounter issues when using a third-party SPI library, please contact that library's author or maintainer, compatibility with USI-based hardware is their responsibility.

### UART (Serial) support
The ATtiny24/44/84 does not feature a hardware UART. When operating from the internal oscillator, clock calibration may be necessary to achieve sufficient timing accuracy for reliable UART communication.

The core provides a built-in software serial implementation exposed as `Serial`. To avoid conflicts with libraries and applications that rely on pin change interrupts (PCINTs), it uses the analog comparator pins and their dedicated interrupt. The default pin assignment is AIN0 for TX and AIN1 for RX.

Being a software implementation, `Serial` cannot transmit and receive simultaneously. The `SoftwareSerial` library may be used alongside the built-in `Serial`, but the same half-duplex limitation applies, only one channel across both instances can be active at a time. If simultaneous TX/RX or the use of multiple serial interfaces is required, a device with a hardware UART should be used instead. Note that software serial has both an [upper and a lower baud rate limit](Ref_TinySoftSerial.md).

The TX pin can be reassigned to any pin on PORTB using `Serial.setTxBit(n)`, where n corresponds to the pin number in PBn notation. To disable the RX channel and use TX only, select TX only from the Software Serial menu under Tools. To disable TX, refrain from printing to Serial and configure the pin to the desired mode after calling `Serial.begin()`.

### Tone support
`tone()` uses Timer1. For best results, use PA6 and PA5, as this will use the hardware output compare to generate the square wave instead of using interrupts.

### Servo support
A built-in Servo library is included with this core, with full support for the ATtiny24/44/84. Note that any ongoing software serial activity, whether through `SoftwareSerial` or the built-in `Serial`, will cause glitches in the servo signal during transmission or reception. Like `tone()`, the Servo library disables PWM on PA6 and PA7. See the [Servo/Servo_TinyCore library](../libraries/Servo/README.md) for further details.

## ADC features
The ATtiny24/44/84 has a differential ADC. Gain of 1x or 20x is available, and two differential pairs are available. The ADC supports both bipolar mode (-512 to 511) and unipolar mode (0-1023) when taking differential measurements; you can set this using `setADCBipolarMode(true or false)`.

### ADC Reference options
The ATtiny24/44/84 has both the 1.1V and 2.56V reference options, a rarity among the classic tinyAVR parts. It also supports an external reference.

| Reference option   | Reference voltage           | Uses AREF pin        | Aliases/synonyms                         |
|--------------------|-----------------------------|----------------------|------------------------------------------|
| `DEFAULT`          | Vcc                         | No, pin available    |                                          |
| `EXTERNAL`         | Voltage applied to AREF pin | Yes, ext. voltage    |                                          |
| `INTERNAL1V1`      | Internal 1.1V reference     | No, pin available    | `INTERNAL`                               |
| `INTERNAL2V56`     | Internal 2.56V reference    | No, pin available    | `INTERNAL2V56_NO_CAP` `INTERNAL2V56NOBP` |
| `INTERNAL2V56_CAP` | Internal 2.56V reference    | Yes, w/cap. on AREF  |                                          |

### Internal Sources
| Voltage Source  | Description                            |
|-----------------|----------------------------------------|
| ADC_INTERNAL1V1 | Reads the INTERNAL1V1 reference        |
| ADC_GROUND      | Measures ground (offset cal?)          |
| ADC_TEMPERATURE | Internal temperature sensor            |

### Differential Channels
The ADC on the x5-series can act as a differential ADC with selectable 1x or 20x gain. It can operate in either unipolar or bipolar mode, and the polarity of the two differential pairs can be inverted in order to maximize the resolution available in unipolar mode. TinyCore wraps the IPR bit into the name of the differential ADC channel. Note the organization of the channels - there are two differential pairs A0/A1, and A2/A3 - but there is no way to take a differential measurement between A0 and A2 or A3 for example.

### Differential ADC


The ATtiny24/44/84 provides 12 differential ADC pairs, all with selectable gain. On A0, A3, and A7, the positive and negative inputs can be set to the same pin, allowing the gain stage offset error to be measured and subtracted from subsequent readings. All pairs also support swapped inputs.

Differential channels are read using `analogRead()` with the named constants listed below. When a channel must be referenced by its numeric value rather than a named constant, use the `ADC_CH()` macro. for example, `analogRead(ADC_CH(0x08))` is equivalent to `analogRead(DIFF_A0_A1_20X)`. This macro sets the high bit to distinguish analog channels from digital pin numbers (`#define ADC_CH(x) (0x80 | (x))`). Passing a raw channel number without this macro will cause it to be interpreted as a digital pin number.

Where a gain value has two possible configurations, the variant that does not use the `GSEL` bit is suffixed with `A` (e.g. `DIFF_A6_A5_20XA`).

### Differential ADC channels
|  Positive  |  Negative  |Gain 1x|Gain 20x| Name (1x Gain) | Name (20x Gain) |
|------------|------------|-------|--------|----------------|-----------------|
| ADC0 (PA0) | ADC0 (PA0) |       |  0x23  |                |  DIFF_A0_A0_20X |
| ADC0 (PA0) | ADC1 (PA1) | 0x08  |  0x09  |  DIFF_A0_A1_1X |  DIFF_A0_A1_20X |
| ADC0 (PA0) | ADC3 (PA3) | 0x0A  |  0x0B  |  DIFF_A0_A3_1X |  DIFF_A0_A3_20X |
| ADC1 (PA1) | ADC0 (PA0) | 0x28  |  0x29  |  DIFF_A1_A0_1X |  DIFF_A1_A0_20X |
| ADC1 (PA1) | ADC2 (PA2) | 0x0C  |  0x0D  |  DIFF_A1_A2_1X |  DIFF_A1_A2_20X |
| ADC1 (PA1) | ADC3 (PA3) | 0x0E  |  0x0F  |  DIFF_A1_A3_1X |  DIFF_A1_A3_20X |
| ADC2 (PA2) | ADC1 (PA1) | 0x2C  |  0x2D  |  DIFF_A2_A1_1X |  DIFF_A2_A1_20X |
| ADC2 (PA2) | ADC3 (PA3) | 0x10  |  0x11  |  DIFF_A2_A3_1X |  DIFF_A2_A3_20X |
| ADC3 (PA3) | ADC0 (PA0) | 0x2A  |  0x2B  |  DIFF_A3_A0_1X |  DIFF_A3_A0_20X |
| ADC3 (PA3) | ADC1 (PA1) | 0x2E  |  0x2F  |  DIFF_A3_A1_1X |  DIFF_A3_A1_20X |
| ADC3 (PA3) | ADC2 (PA2) | 0x30  |  0x31  |  DIFF_A3_A2_1X |  DIFF_A3_A2_20X |
| ADC3 (PA3) | ADC3 (PA3) | 0x24  |  0x25  |  DIFF_A3_A3_1X |  DIFF_A3_A3_20X |
| ADC3 (PA3) | ADC4 (PA4) | 0x12  |  0x13  |  DIFF_A3_A4_1X |  DIFF_A3_A4_20X |
| ADC3 (PA3) | ADC5 (PA5) | 0x14  |  0x15  |  DIFF_A3_A5_1X |  DIFF_A3_A5_20X |
| ADC3 (PA3) | ADC6 (PA6) | 0x16  |  0x17  |  DIFF_A3_A6_1X |  DIFF_A3_A6_20X |
| ADC3 (PA3) | ADC7 (PA7) | 0x18  |  0x19  |  DIFF_A3_A7_1X |  DIFF_A3_A7_20X |
| ADC4 (PA4) | ADC3 (PA3) | 0x32  |  0x33  |  DIFF_A4_A3_1X |  DIFF_A4_A3_20X |
| ADC4 (PA4) | ADC5 (PA5) | 0x1A  |  0x1B  |  DIFF_A4_A5_1X |  DIFF_A4_A5_20X |
| ADC5 (PA5) | ADC3 (PA3) | 0x34  |  0x35  |  DIFF_A5_A3_1X |  DIFF_A5_A3_20X |
| ADC5 (PA5) | ADC4 (PA4) | 0x3A  |  0x3B  |  DIFF_A5_A4_1X |  DIFF_A5_A4_20X |
| ADC5 (PA5) | ADC6 (PA6) | 0x1C  |  0x1D  |  DIFF_A5_A6_1X |  DIFF_A5_A6_20X |
| ADC6 (PA6) | ADC3 (PA3) | 0x36  |  0x37  |  DIFF_A6_A3_1X |  DIFF_A6_A3_20X |
| ADC6 (PA6) | ADC5 (PA5) | 0x3C  |  0x3D  |  DIFF_A6_A5_1X |  DIFF_A6_A5_20X |
| ADC6 (PA6) | ADC7 (PA7) | 0x1E  |  0x1F  |  DIFF_A6_A7_1X |  DIFF_A6_A7_20X |
| ADC7 (PA7) | ADC3 (PA3) | 0x38  |  0x39  |  DIFF_A7_A3_1X |  DIFF_A7_A3_20X |
| ADC7 (PA7) | ADC6 (PA6) | 0x3E  |  0x3F  |  DIFF_A7_A6_1X |  DIFF_A7_A6_20X |
| ADC7 (PA7) | ADC7 (PA7) | 0x26  |  0x27  |  DIFF_A7_A7_1X |  DIFF_A7_A7_20X |

#### ADC Differential Pair Matrix

 | N\P |  0 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |
 |-----|----|----|----|----|----|----|----|----|
 |   0 |  X |  X |    |  X |    |    |    |    |
 |   1 |  X |    |  X |  X |    |    |    |    |
 |   2 |    |  X |    |  X |    |    |    |    |
 |   3 |  X |  X |  X |  X |  X |  X |  X |  X |
 |   4 |    |    |    |  X |    |  X |    |    |
 |   5 |    |    |    |  X |  X |    |  X |    |
 |   6 |    |    |    |  X |    |  X |    |  X |
 |   7 |    |    |    |  X |    |    |  X |  X |

By default, differential measurements are taken in unipolar mode, requiring the positive input to exceed the negative input. If the polarity is known in advance, this is generally the preferred mode. When the signal polarity cannot be guaranteed, bipolar mode is available, providing a signed 9-bit result that accommodates negative values.

### Temperature Measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.
