# ATtiny 261/461/861(a)
![x61 pin mapping](Pinout_x61.png "Arduino Pin Mapping for ATtiny x61-family")

| Specification                    | ATtiny861               | ATtiny461               | ATtiny261               |
|----------------------------------|-------------------------|-------------------------|-------------------------|
| Bootloader (occupies 256 bytes)  | Urboot                  | Urboot                  | Urboot                  |
| Uploading uses                   | ISP/SPI pins            | ISP/SPI pins            | ISP/SPI pins            |
| Flash available user             | 7936 / 8192 bytes       | 3840 / 4096 bytes       | 1792 / 2048 bytes       |
| RAM                              | 512 bytes               | 256 bytes               | 128 bytes               |
| EEPROM                           | 512 bytes               | 256 bytes               | 128 bytes               |
| GPIO Pins                        | 15 + RESET              | 15 + RESET              | 15 + RESET              |
| ADC Channels                     | 11 (incl RST)           | 11 (incl RST)           | 11 (incl RST)           |
| Differential ADC                 | Yes, 1x/8x/20x/32x gain | Yes, 1x/8x/20x/32x gain | Yes, 1x/8x/20x/32x gain |
| PWM Channels                     | 3                       | 3                       | 3                       |
| Interfaces                       | USI                     | USI                     | USI                     |
| Int. Oscillator (MHz)            | 16, 8, 4, 2, 1          | 16, 8, 4, 2, 1          | 16, 8, 4, 2, 1          |
| External Crystal                 | All standard            | All standard            | All standard            |
| External Clock                   | All standard            | All standard            | All standard            |
| Int. WDT Oscillator              | 128 kHz                 | 128 kHz                 | 128 kHz                 |
| LED_BUILTIN                      | PIN_PB6                 | PIN_PB6                 | PIN_PB6                 |

## Overview
The ATtiny261/461/861 is a microcontroller designed with brushless DC (BLDC) motor control in mind. It features an on-chip PLL and a high-speed Timer1 capable of generating three complementary PWM signals with configurable dead time, as required for three-phase BLDC motor drives. It also includes a notably capable ADC, second only to the ATtiny441/841 in the number of differential pairs and programmable gain options.

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny261/461/861, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 1792, 3840, or 7936 bytes available for user code on the ATtiny261, 461, and 861 respectively. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The default serial upload pins for these chips are PA6 (TX) and PA7 (RX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader can be configured to apply an oscillator correction factor. This is exposed as a Tools menu option, with adjustable compensation ranging from -5.00% to +5.00%.

### Internal oscillator calibration
The internal 8 MHz oscillator and 16 MHz PLL is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail.

To address this, TinyCore provides an [Oscillator calibration sketch](../libraries/TinyCore/examples/OscillatorCalibration/OscillatorCalibration.ino) that calculates a corrected OSCCAL value based on characters received over UART. It uses the default UART pins, **TX = PA1** and **RX = PA2**. Before uploading the sketch, ensure the target is running from its internal 8 MHz oscillator or 16 MHz PLL, and EEPROM preservation is enabled. Once uploaded, open the serial monitor at 115200 baud, select "No line ending", and repeatedly send the character `x`. After a few attempts, readable text should begin to appear in the serial monitor. Once the calibration value has stabilised, it is automatically stored in the last byte of EEPROM for future use. This value is not loaded automatically and must be applied explicitly in your sketch:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL = cal;
```

Another approach is to use the [avrCalibrate](https://github.com/felias-fogg/avrCalibrate) library, which uses a host microcontroller along with the target to perform the calibraion. avrCalibrate can also calibrate internal voltage references.


## Features

### PLL clock
The ATtiny261/461/861 features an on-chip PLL clocked from the internal oscillator, running at a nominal frequency of 64 MHz when enabled. It can be used in two ways, either independently or in combination.  
The system clock can be derived from one quarter of the PLL output, providing a 16 MHz clock source without an external crystal. This option inherits the accuracy limitations of the internal oscillator driving the PLL. Timer1 can be clocked directly from the PLL, enabling high-speed PWM and other timer functions independent of the system clock speed.

### Timer1 is a high speed timer
See the [TinyCore library](../libraries/TinyCore/README.md)

### PWM frequency
TC0 does not support PWM, and is used for millis. TC1 gives 3 outputs, and can operate in phase correct or fast pwm mode, or variations on those that operate only in PWM6 mode, which is designed specifically for controlling brushless DC (BLDC) motors. We always use phase correct when we can reach the target frequency with it. Which, on these parts, is all the time.

| F_CPU  | No PWM from TC0     | F_PWM<sub>TC1</sub>   | Notes                        |
|--------|---------------------|-----------------------|------------------------------|
| 1  MHz |                   - |  1/4/512=      488 Hz | Phase & Freq correct TC1     |
| 2  MHz |                   - |  2/8/512=      488 Hz | Phase & Freq correct TC1     |
| <4 MHz |                   - |  x/8/512=  244 * x Hz | Phase & Freq correct TC1     |
| 4  MHz |                   - |  4/8/512=      977 Hz | Phase & Freq correct TC1     |
| <8 MHz |                   - |  x/16/512= 122 * x Hz | Phase & Freq correct TC1     |
| 8  MHz |                   - |  8/32/512=     488 Hz | Phase & Freq correct TC1     |
| >8 MHz |                   - |  x/32/512=  61 * x Hz | Phase & Freq correct TC1     |
| 12 MHz |                   - | 12/32/512=     735 Hz | Phase & Freq correct TC1     |
| 16 MHz |                   - | 16/32/512=     977 Hz | Phase & Freq correct TC1     |
|>16 MHz |                   - |  x/64/512=  31 * x Hz | Phase & Freq correct TC1     |
| 20 MHz |                   - | 20/64/512=     610 Hz | Phase & Freq correct TC1     |

Phase and frequency correct PWM counts up to 255, turning the pin off as it passes the compare value, then it counts down to 0, flipping the pin back as is passes the compare value, and then updates it's double-buffered registers at BOTTOM. Phase and frequency correct PWM is almost never worse than normal PWM - as long as the

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C support
The ATtiny261/461/861 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI support
The ATtiny261/461/861 does not feature a dedicated SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library.

Only the built-in SPI library is officially supported. If you encounter issues when using a third-party SPI library, please contact that library's author or maintainer, compatibility with USI-based hardware is their responsibility.

### UART (Serial) support
The ATtiny261/461/861 does not feature a hardware UART. When operating from the internal oscillator, clock calibration may be necessary to achieve sufficient timing accuracy for reliable UART communication.

The core provides a built-in software serial implementation exposed as `Serial`. To avoid conflicts with libraries and applications that rely on pin change interrupts (PCINTs), it uses the analog comparator pins and their dedicated interrupt. The default pin assignment is AIN0 for TX and AIN1 for RX.

Being a software implementation, `Serial` cannot transmit and receive simultaneously. The `SoftwareSerial` library may be used alongside the built-in `Serial`, but the same half-duplex limitation applies, only one channel across both instances can be active at a time. If simultaneous TX/RX or the use of multiple serial interfaces is required, a device with a hardware UART should be used instead. Note that software serial has both an [upper and a lower baud rate limit](Ref_TinySoftSerial.md).

The TX pin can be reassigned to any pin on PORTA using `Serial.setTxBit(n)`, where n corresponds to the pin number in PAn notation. To disable the RX channel and use TX only, select TX only from the Software Serial menu under Tools. To disable TX, refrain from printing to Serial and configure the pin to the desired mode after calling `Serial.begin()`.

### Tone
`tone()` is implemented using Timer1. If Timer1 has been configured for high-speed operation, `tone()` will produce frequencies two or four times higher than expected. Normal speed operation is recommended when using `tone()`.

For best results, use pin PA1 or PA4 as the tone output pin. On these pins, `tone()` drives Timer1's output compare unit directly rather than toggling the pin via interrupt, which extends the usable frequency range into the MHz region. When `SoftwareSerial` or the built-in `Serial` is active, `tone()` remains functional on PWM pins but is unavailable on all other pins. As Timer1 is the only timer capable of hardware PWM on the ATtiny261/461/861, using `tone()` will disable all PWM functionality for the duration of its use.

### Servo support
A built-in Servo library is included with this core, with full support for the ATtiny261/461/861. Note that any ongoing software serial activity, whether through `SoftwareSerial` or the built-in `Serial`, will cause glitches in the servo signal during transmission or reception. The Servo library disables PWM on pin 4 regardless of which pin is used for output, and cannot be used simultaneously with `tone()`.

If a version of the Servo library has been installed through the Library Manager, use `#include <Servo_TinyCore.h>` explicitly to avoid pulling in the incompatible library manager version.

## ADC features
The ATtiny261/461/861 has a surprisingly sophisticated ADC with many differential channels, most with selectable gain. These are available through analogRead. When used to read a pair of analog pins in differential mode, the ADC normally runs in unipolar mode: The voltage on the positive pin must be higher than that on the negative one, but the difference is measured to the full precision of the ADC. It can be put into bipolar mode, where the voltage on the negative side can go below the voltage on the positive side and generate meaningful measurements (it will return a signed value, which costs 1 bit of accuracy for the sign bit). This can be enabled by calling the helper function `setADCBipolarMode(true or false)`.

### Differential ADC
The ATtiny261/461/861 features a sophisticated differential ADC, with a large selection of differential pairs and selectable gain on most channels. Differential channels are accessible through `analogRead()`.

By default, the ADC operates in unipolar mode, requiring the positive input to exceed the negative input, with the full ADC precision applied to the difference. Bipolar mode is also available, allowing the negative input to exceed the positive and returning a signed result at the cost of one bit of resolution. Bipolar mode is enabled and disabled by calling `setADCBipolarMode(true)` and `setADCBipolarMode(false)` respectively.

### ADC reference options
The ATtiny261/461/861 provides two internal reference voltages, one of which supports connection of an external capacitor on the AREF pin for improved stability. An external reference voltage and the supply voltage (Vcc) are also available as reference sources.

| Reference Option   | Reference Voltage           | Uses AREF Pin        | Aliases/synonyms                         |
|--------------------|-----------------------------|----------------------|------------------------------------------|
| `DEFAULT`          | Vcc                         | No, pin available    |                                          |
| `EXTERNAL`         | Voltage applied to AREF pin | Yes, ext. voltage    |                                          |
| `INTERNAL1V1`      | Internal 1.1V reference     | No, pin available    | `INTERNAL`                               |
| `INTERNAL2V56`     | Internal 2.56V reference    | No, pin available    | `INTERNAL2V56_NO_CAP` `INTERNAL2V56NOBP` |
| `INTERNAL2V56_CAP` | Internal 2.56V reference    | Yes, w/cap. on AREF  |                                          |

### Internal sources
| Voltage Source  | Description                            |
|-----------------|----------------------------------------|
| ADC_INTERNAL1V1 | Reads the INTERNAL1V1 reference        |
| ADC_GROUND      | Reads ground - for offset correction?  |
| ADC_TEMPERATURE | Reads internal temperature sensor      |

### Differential ADC
The ATtiny261/461/861 provides 24 differential pairs. Seven of these measure the same pin on both inputs, allowing the gain stage offset error to be determined and subtracted from subsequent measurements taken at the same gain setting. In total, 81 channel and gain combinations are available: 31 with selectable gain (typically offering both 1x/8x and 20x/32x options per pair), and 19 with fixed gain, 10 of which duplicate selectable-gain configurations.

Differential channels are read using `analogRead()` with the named constants listed below. When a channel must be referenced by its numeric value rather than a named constant, use the `ADC_CH()` macro, passing the gain selection as bit 6 of the channel value. For example, `analogRead(ADC_CH(0x20 | 0x40))` is equivalent to `analogRead(DIFF_A0_A1_32X)`. Passing a raw channel number without this macro will cause it to be interpreted as a digital pin number.

Where a gain value has two possible configurations, the variant that does not use the `GSEL` bit is suffixed with `A` (e.g. `DIFF_A6_A5_20XA`).

| Positive   | Negative   |   Gain  | Channel| Name 1x/20x mode | Name 8x/32x mode | Notes            |
|------------|------------|---------|--------|------------------|------------------|------------------|
| ADC2 (PA2) | ADC3 (PA4) |      1x |   0x10 | DIFF_A2_A3_1X    |                  |                  |
| ADC3 (PA4) | ADC3 (PA4) |     20x |   0x11 | DIFF_A3_A3_20X   |                  | For offset cal.  |
| ADC4 (PA5) | ADC3 (PA4) |     20x |   0x12 | DIFF_A4_A3_20X   |                  |                  |
| ADC4 (PA5) | ADC3 (PA4) |      1x |   0x13 | DIFF_A4_A3_1X    |                  |                  |
| ADC8 (PB5) | ADC9 (PB6) |     20x |   0x19 | DIFF_A8_A9_20X   |                  |                  |
| ADC8 (PB5) | ADC9 (PB6) |      1x |   0x1A | DIFF_A8_A9_1X    |                  |                  |
| ADC9 (PB6) | ADC9 (PB6) |     20x |   0x1B | DIFF_A9_A9_20X   |                  | For offset cal.  |
| ADC10(PB7) | ADC9 (PB6) |     20x |   0x1C | DIFF_A10_A9_20X  |                  |                  |
| ADC10(PB7) | ADC9 (PB6) |      1x |   0x1D | DIFF_A10_A9_1X   |                  |                  |
| ADC0 (PA0) | ADC1 (PA1) | 20x/32x |   0x20 | DIFF_A0_A1_20X   | DIFF_A0_A1_32X   |                  |
| ADC0 (PA0) | ADC1 (PA1) |   1x/8x |   0x21 | DIFF_A0_A1_1X    | DIFF_A0_A1_8X    |                  |
| ADC1 (PA1) | ADC0 (PA0) | 20x/32x |   0x22 | DIFF_A1_A0_20X   | DIFF_A1_A0_32X   |                  |
| ADC1 (PA1) | ADC0 (PA0) |   1x/8x |   0x23 | DIFF_A1_A0_1X    | DIFF_A1_A0_8X    |                  |
| ADC1 (PA1) | ADC2 (PA2) | 20x/32x |   0x24 | DIFF_A1_A2_20X   | DIFF_A1_A2_32X   |                  |
| ADC1 (PA1) | ADC2 (PA2) |   1x/8x |   0x25 | DIFF_A1_A2_1X    | DIFF_A1_A2_8X    |                  |
| ADC2 (PA2) | ADC1 (PA1) | 20x/32x |   0x26 | DIFF_A2_A1_20X   | DIFF_A2_A1_32X   |                  |
| ADC2 (PA2) | ADC1 (PA1) |   1x/8x |   0x27 | DIFF_A2_A1_1X    | DIFF_A2_A1_8X    |                  |
| ADC2 (PA2) | ADC0 (PA0) | 20x/32x |   0x28 | DIFF_A2_A0_20X   | DIFF_A2_A0_32X   |                  |
| ADC2 (PA2) | ADC0 (PA0) |   1x/8x |   0x29 | DIFF_A2_A0_1X    | DIFF_A2_A0_8X    |                  |
| ADC0 (PA0) | ADC2 (PA2) | 20x/32x |   0x2A | DIFF_A0_A2_20X   | DIFF_A0_A2_32X   |                  |
| ADC0 (PA0) | ADC2 (PA2) |   1x/8x |   0x2B | DIFF_A0_A2_1X    | DIFF_A0_A2_8X    |                  |
| ADC4 (PA5) | ADC5 (PA6) | 20x/32x |   0x2C | DIFF_A4_A5_20X   | DIFF_A4_A5_32X   |                  |
| ADC4 (PA5) | ADC5 (PA6) |   1x/8x |   0x2D | DIFF_A4_A5_1X    | DIFF_A4_A5_8X    |                  |
| ADC5 (PA6) | ADC4 (PA5) | 20x/32x |   0x2E | DIFF_A5_A4_20X   | DIFF_A5_A4_32X   |                  |
| ADC5 (PA6) | ADC4 (PA5) |   1x/8x |   0x2F | DIFF_A5_A4_1X    | DIFF_A5_A4_8X    |                  |
| ADC5 (PA6) | ADC6 (PA7) | 20x/32x |   0x30 | DIFF_A5_A6_20X   | DIFF_A5_A6_32X   |                  |
| ADC5 (PA6) | ADC6 (PA7) |   1x/8x |   0x31 | DIFF_A5_A6_1X    | DIFF_A5_A6_8X    |                  |
| ADC6 (PA7) | ADC5 (PA6) | 20x/32x |   0x32 | DIFF_A6_A5_20X   | DIFF_A6_A5_32X   |                  |
| ADC6 (PA7) | ADC5 (PA6) |   1x/8x |   0x33 | DIFF_A6_A5_1X    | DIFF_A6_A5_8X    |                  |
| ADC6 (PA7) | ADC4 (PA5) | 20x/32x |   0x34 | DIFF_A6_A4_20X   | DIFF_A6_A4_32X   |                  |
| ADC6 (PA7) | ADC4 (PA5) |   1x/8x |   0x35 | DIFF_A6_A4_1X    | DIFF_A6_A4_8X    |                  |
| ADC4 (PA5) | ADC6 (PA7) | 20x/32x |   0x36 | DIFF_A4_A6_20X   | DIFF_A4_A6_32X   |                  |
| ADC4 (PA5) | ADC6 (PA7) |   1x/8x |   0x37 | DIFF_A4_A6_1X    | DIFF_A4_A6_8X    |                  |
| ADC0 (PA0) | ADC0 (PA0) | 20x/32x |   0x38 | DIFF_A0_A0_20X   | DIFF_A0_A0_32X   | For offset cal.  |
| ADC0 (PA0) | ADC0 (PA0) |   1x/8x |   0x39 | DIFF_A0_A0_1X    | DIFF_A0_A0_8X    | For offset cal.  |
| ADC1 (PA1) | ADC1 (PA1) | 20x/32x |   0x3A | DIFF_A1_A1_20X   | DIFF_A1_A1_32X   | For offset cal.  |
| ADC2 (PA2) | ADC2 (PA2) | 20x/32x |   0x3B | DIFF_A2_A2_20X   | DIFF_A2_A2_32X   | For offset cal.  |
| ADC4 (PA5) | ADC4 (PA5) | 20x/32x |   0x3C | DIFF_A4_A4_20X   | DIFF_A4_A4_32X   | For offset cal.  |
| ADC5 (PA6) | ADC5 (PA6) | 20x/32x |   0x3D | DIFF_A5_A5_20X   | DIFF_A5_A5_32X   | For offset cal.  |
| ADC6 (PA7) | ADC6 (PA7) | 20x/32x |   0x3E | DIFF_A6_A6_20X   | DIFF_A6_A6_32X   | For offset cal.  |
| ADC0 (PA0) | ADC1 (PA1) |     20x |   0x0B | DIFF_A0_A1_20XA  |                  | Duplicate        |
| ADC0 (PA0) | ADC1 (PA1) |      1x |   0x0C | DIFF_A0_A1_1XA   |                  | Duplicate        |
| ADC1 (PA1) | ADC1 (PA1) |     20x |   0x0D | DIFF_A1_A1_20XA  |                  | Duplicate        |
| ADC2 (PA2) | ADC1 (PA1) |     20x |   0x0E | DIFF_A2_A1_20XA  |                  | Duplicate        |
| ADC2 (PA2) | ADC1 (PA1) |      1x |   0x0F | DIFF_A2_A1_1XA   |                  | Duplicate        |
| ADC4 (PA5) | ADC5 (PA6) |     20x |   0x14 | DIFF_A4_A5_20XA  |                  | Duplicate        |
| ADC4 (PA5) | ADC5 (PA6) |      1x |   0x15 | DIFF_A4_A5_1XA   |                  | Duplicate        |
| ADC5 (PA6) | ADC5 (PA6) |     20x |   0x16 | DIFF_A5_A5_20XA  |                  | Duplicate        |
| ADC6 (PA7) | ADC5 (PA6) |     20x |   0x17 | DIFF_A6_A5_20XA  |                  | Duplicate        |
| ADC6 (PA7) | ADC5 (PA6) |      1x |   0x18 | DIFF_A6_A5_1XA   |                  | Duplicate        |

### Differential ADC channel layout
The first set of differential channels is a direct carry-over from the ATtiny26, with identical ADMUX values to allow existing code and hardware designs to be used without modification. These channels offer only 1x and 20x gain and do not support GSEL, nor are the inputs reversible. if the polarity is incorrect, bipolar input mode is the only recourse, at the cost of one bit of resolution.

Beyond this legacy set, two groups of three pins (ADC0/ADC1/ADC2 and ADC4/ADC5/ADC6) are available in both directions with GSEL support, enabling 8x and 32x gain options. Additionally, ADC0 measured against itself is available with all gain options, and the remaining channels in the second half of the differential ADC support 20x and 32x gain.

The overlap between the two halves raises some questions about the intent of the design. The pairs involving ADC1 and ADC5 are accessible through both halves, potentially via different signal pathways. If the pathways differ, one might expect different offset characteristics between, for example, `DIFF_A1_A1_20XA` (channel 0x0D) and `DIFF_A1_A1_20X` (channel 0x3A with GSEL=0). If they share the same pathway, the duplication appears to be a pragmatic reuse of existing mux channels rather than an opportunity to extend gain selection to additional pins.

#### ADC differential pair matrix
* **bold** indicates that an option has all gain options available.
* *italic* indicates only 20x/32x gain
* Unstyled text indicates that only 1x and 20x are available
|  N\P  |   0   |   1   |   2   |   3   |   4   |   5   |   6   |   8   |   9   |  10   |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|   0   | **X** | **X** | **X** |       |       |       |       |       |       |       |
|   1   | **X** |  *X*  | **X** |       |       |       |       |       |       |       |
|   2   | **X** | **X** |  *X*  |       |       |       |       |       |       |       |
|   3   |       |       |   1x  |  20x  |   X   |       |       |       |       |       |
|   4   |       |       |       |       |  *X*  | **X** | **X** |       |       |       |
|   5   |       |       |       |       | **X** |  *X*  | **X** |       |       |       |
|   6   |       |       |       |       | **X** | **X** |  *X*  |       |       |       |
|   9   |       |       |       |       |       |       |       |   X   |  20x  |   X   |

### Temperature measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.
