# ATtiny 87/167
![x7 pin mapping](Pinout_x7.png "Arduino Pin Mapping for ATtiny x7-family")

| Specification                    | ATtiny167          | ATtiny87           |
|----------------------------------|--------------------|--------------------|
| Bootloader (occupies 256 bytes)  | Urboot             | Urboot             |
| Flash available user             | 16128 / 16384 bytes| 7936 / 8192 bytes  |
| RAM                              | 512 bytes          | 512 bytes          |
| EEPROM                           | 512 bytes          | 512 bytes          |
| GPIO Pins                        | 15                 | 15                 |
| ADC Channels                     | 11                 | 11                 |
| PWM Channels                     | 1 fixed, 2 flex    | 1 fixed, 2 flex    |
| Differential ADC                 | 8x/20x gain        | 8x/20x gain        |
| Interfaces                       | LIN/UART, USI, SPI | LIN/UART, USI, SPI |
| Int. Oscillator (MHz)            | 8, 4, 2, 1         | 8, 4, 2, 1         |
| External Crystal                 | All standard       | All standard       |
| External Clock                   | All standard       | All standard       |
| Int. WDT Oscillator              | 128 kHz            | 128 kHz            |
| LED_BUILTIN                      | PIN_PA6            | PIN_PA6            |

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny87/167, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 7936 or 16128 bytes available for user program. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The serial upload pins for these chips are PA0 (RX) and PA1 (TX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader has "autobaud functionality", which means that it will try to adjust and match the host baud rate.

### Internal oscillator calibration
The internal 8 MHz oscillator is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail.

To address this, TinyCore provides an [Oscillator calibration sketch](../libraries/TinyCore/examples/OscillatorCalibration/OscillatorCalibration.ino) that calculates a corrected OSCCAL value based on characters received over UART. It uses the default UART pins, **TX = PA1** and **RX = PA0**. Before uploading the sketch, ensure the target is running from its internal 8 MHz oscillator and that EEPROM preservation is enabled. Once uploaded, open the serial monitor at 115200 baud, select "No line ending", and repeatedly send the character `x`. After a few attempts, readable text should begin to appear in the serial monitor. Once the calibration value has stabilised, it is automatically stored in the last byte of EEPROM for future use. This value is not loaded automatically and must be applied explicitly in your sketch:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL = cal;
```

Another approach is to use the [avrCalibrate](https://github.com/felias-fogg/avrCalibrate) library, which uses a host microcontroller along with the target to perform the calibraion. avrCalibrate can also calibrate internal voltage references.


## Features

### Flexible PWM support
Timer1 provides two channels, each capable of driving multiple pins simultaneously. However, all pins sharing a channel output the same duty cycle. On the ATtiny87/167, the OCR1Ax and OCR1Bx channels drive the even and odd pins of PORTB respectively, while PA3 is driven independently by Timer0.

This means that while all 8 pins support PWM, only two independent duty cycles are available across the PORTB pins. For example, calling `analogWrite(PIN_PB0, 64)` sets PB0 to 25%, but a subsequent `analogWrite(PIN_PB2, 128)` will update the shared OCR1AU and OCR1AW channel to 50%, affecting PB0 as well. Pins on different channels are independent: `analogWrite(PIN_PB0, 64)` and `analogWrite(PIN_PB1, 128)` will correctly output 25% and 50% respectively, as PB0 and PB1 belong to OCR1AU and OCR1BU.

To stop PWM output on a pin, call `digitalWrite()` or `analogWrite()` with a value of 0 or 255.

#### PWM frequency
TC0 is always run in Fast PWM mode: We use TC0 for millis, and phase correct mode can't be used on the millis timer - you need to read the count to get micros, but that doesn't tell you the time in phase correct mode because you don't know if it's upcounting or downcounting in phase correct mode.

| F_CPU  | F_PWM<sub>TC0</sub> | F_PWM<sub>TC1</sub>   | Notes                        |
|--------|---------------------|-----------------------|------------------------------|
| 1  MHz | 1/8/256=     488 Hz |  1/8/256=      488 Hz |                              |
| 2  MHz | 2/8/256=     977 Hz |  2/8/256=      977 Hz |                              |
| <4 MHz | x/8/256= 488 * x Hz |  x/8/512=  244 * x Hz | Phase correct TC1            |
| 4  MHz | 4/32/256=    488 Hz |  4/8/512=      977 Hz | /32 prescale on TC0 x7 only. Phase correct TC1 |
| <8 MHz | x/32/256=122 * x Hz |  x/8/512=  244 * x Hz | /32 prescale on TC0 x7 only. Between 4 and 8 MHz, the target range is elusive ||
| 8  MHz | 8/32/256=    977 Hz |  8/64/256=     488 Hz | /32 prescale on TC0 x7 only. |
| >8 MHz | x/64/256= 61 * x Hz |  x/64/256=  61 * x Hz |                              |
| 12 MHz | 12/64/256=   732 Hz | 12/64/256=     732 Hz |                              |
| 16 MHz | 16/64/256=   977 Hz | 16/64/256=     977 Hz |                              |
|>16 MHz | x/128/256=30 * x Hz |  x/64/512=  30 * x Hz | Phase correct TC1            |
| 20 MHz | 20/128/255=  610 Hz | 20/64/512=     610 Hz | Phase correct TC1            |

Where the /32 or /128 prescaler, not available on most parts, is used, it significantly improves the output frequency in the most desirable range for typical applications - unfortunately timer0 has only a single output (it's not the standard TC0 - it has much in common the the ATmega TC2 async, potentially externally-clocked timer.

Where speeds above or below a certain speed are specified, it's implied that the other end of the range is the next marked value. So >16 in that table is for 16-20 MHz clocks. The formula is given as a constant times x where x is expressed as MHz (the division above gets the time in megahertz - in the interest of readability I did not include the MHz to Hz conversion - I'm sure you all know how to multiply by a million)

Phase correct PWM counts up to 255, flipping the pin off (er... that may not be the best choice of words, I wasn't thinking about the colloquial sense...) as it passes the compare value, updates it's double-buffered registers at TOP, then it counts down to 0, flipping the pin back as is passes the compare value. This is considered preferable for motor control applications, though the "Phase and Frequency Correct" mode is better if the period is ever adjusted by a large amount at a time, because it updates the doublebuffered registers at BOTTOM, and thus produces a less problematic glitch in the duty cycle, but doesn't have any modes that don't require setting ICR1 too.

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C support
The ATtiny87/167 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI support
The ATtiny87/167 features a full hardware SPI peripheral, supported by the standard SPI library. Third-party libraries targeting tinyAVR parts that rely on USI-based implementations such as USIWire should not be used, as the ATtiny87/167 is among the few tinyAVR devices with a dedicated SPI peripheral and does not use the USI for SPI communication.

### UART with LIN support
The ATtiny87/167 features a full hardware UART with LIN (Local Interconnect Network) support, exposed as `Serial` and fully compatible with the standard Arduino Serial API. Unlike the software serial implementations required on most other ATtiny devices, this is a true hardware peripheral.

LIN support is unique to the ATtiny87/167 among the classic ATtiny lineup, and is widely used in automotive and industrial applications. A notable benefit of the LIN-capable baud rate generator is its ability to match target baud rates more closely than a standard UART module.

### Tone support
`tone()` uses Timer1. For best results, a pin on PORTB, as this will use the hardware output compare to generate the square wave instead of using interrupts. Using `tone()` will disable all PWM pins except PIN_PA2.

### Servo support
A built-in Servo library is included with this core, with full support for the ATtiny87/167. Note that any ongoing software serial activity, whether through `SoftwareSerial` or the built-in `Serial`, will cause glitches in the servo signal during transmission or reception. The Servo library disables all PWM pins except PA2, and cannot be used simultaneously with `tone()`. See the [Servo/Servo_TinyCore library](../libraries/Servo/README.md) for more details.

## ADC features
The ATtiny87/167 features a mid-range ADC with a second internal reference voltage, a built-in voltage divider on AVCC, and a modest selection of differential pairs. AVCC is nominally tied to VCC and may optionally be filtered to reduce noise, though this is rarely done in practice. Note that the datasheet requires AVCC to remain within 0.3V of VCC.

The device has the unusual ability to drive its internal reference voltage onto the AREF pin, allowing it to be used as a reference source for other external devices. However, the interaction between this feature and other uses of the AREF pin is ambiguous in the datasheet. The note that "the internal voltage reference options may not be used if an external voltage is being applied to the AREF pin" leaves it unclear whether outputting the internal reference onto the pin is still permissible when an external reference is also present. Until this is clarified, caution is advised when sharing the AREF pin with other devices.

| Reference Option    | Reference Voltage           | Uses AREF Pin                 |
|---------------------|-----------------------------|-------------------------------|
| `DEFAULT`           | Vcc                         | No, pin available             |
| `EXTERNAL`          | Voltage applied to AREF pin | Yes, ext. voltage             |
| `INTERNAL1V1`       | Internal 1.1V reference     | Unclear, looks like yes?      |
| `INTERNAL2V56`      | Internal 2.56V reference    | Unclear, looks like yes?      |
| `INTERNAL1V1_XREF`  | Internal 1.1V reference     | Yes, reference output on AREF |
| `INTERNAL2V56_XREF` | Internal 2.56V reference    | Yes, reference output on AREF |
| `INTERNAL`          | Same as `INTERNAL1V1`       | No, pin available             |

### Internal Sources
| Voltage Source    | Description                            |
|-------------------|----------------------------------------|
| `ADC_INTERNAL1V1` | Reads the INTERNAL1V1 reference        |
| `ADC_GROUND`      | Reads ground - for offset correction?  |
| `ADC_AVCCDIV4`    | Reads AVCC divided by 4                |
| `ADC_TEMPERATURE` | Reads internal temperature sensor      |

### Differential ADC
The ATtiny87/167 provides a modest selection of differential ADC pairs with selectable 8x and 20x gain. Differential channels are read using `analogRead()` with the named constants listed below. When a channel must be referenced by its numeric value rather than a named constant, use the `ADC_CH()` macro. For example, `analogRead(ADC_CH(0x11))` is equivalent to `analogRead(DIFF_A0_A1_20X)`. Passing a raw channel number without this macro will cause it to be interpreted as a digital pin number. The ADC supports both bipolar mode (-512 to 511) and unipolar mode (0-1023) when taking differential measurements; you can set this using `setADCBipolarMode(true or false)`.

|  Positive  |   Negative  | 8X Gain |20X Gain|  Name (8x Gain)  |  Name (20x Gain)  |    Digispark 8x   |   Digispark 20x    |
|------------|-------------|---------|--------|------------------|-------------------|-------------------|--------------------|
| ADC0 (PA0) |  ADC1 (PA1) |   0x10  |  0x11  |  `DIFF_A0_A1_8X` |  `DIFF_A0_A1_20X` |   `DIFF_A6_A7_8X` |   `DIFF_A6_A7_20X` |
| ADC1 (PA1) |  ADC2 (PA2) |   0x12  |  0x13  |  `DIFF_A1_A2_8X` |  `DIFF_A1_A2_20X` |  `DIFF_A7_A13_8X` |  `DIFF_A7_A13_20X` |
| ADC2 (PA2) |  ADC3 (PA3) |   0x14  |  0x15  |  `DIFF_A2_A3_8X` |  `DIFF_A2_A3_20X` |  `DIFF_A13_A9_8X` |  `DIFF_A13_A9_20X` |
| ADC4 (PA4) |  ADC5 (PA5) |   0x16  |  0x17  |  `DIFF_A4_A5_8X` |  `DIFF_A4_A5_20X` | `DIFF_A10_A11_8X` | `DIFF_A10_A11_20X` |
| ADC5 (PA5) |  ADC6 (PA6) |   0x18  |  0x19  |  `DIFF_A5_A6_8X` |  `DIFF_A5_A6_20X` | `DIFF_A11_A12_8X` | `DIFF_A11_A12_20X` |
| ADC6 (PA6) |  ADC7 (PA7) |   0x1A  |  0x1B  |  `DIFF_A6_A7_8X` |  `DIFF_A6_A7_20X` |  `DIFF_A12_A5_8X` |  `DIFF_A12_A5_20X` |
| ADC8 (PB5) |  ADC9 (PB6) |   0x1C  |  0x1D  |  `DIFF_A8_A9_8X` |  `DIFF_A8_A9_20X` |   `DIFF_A8_A3_8X` |   `DIFF_A8_A3_20X` |
| ADC9 (PB6) | ADC10 (PB7) |   0x1E  |  0x1F  | `DIFF_A9_A10_8X` | `DIFF_A9_A10_20X` |  `DIFF_A3_A15_8X` |  `DIFF_A3_A15_20X` |

### Temperature Measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.
