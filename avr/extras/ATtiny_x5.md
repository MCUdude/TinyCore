# ATtiny 25/45/85
![x5 pin mapping](Pinout_x5.png "Arduino Pin Mapping for ATtiny x5-family")

| Specification                    | ATtiny85         | ATtiny45         | ATtiny25         |
|----------------------------------|------------------|------------------|------------------|
| Bootloader (occupies 256 bytes)  | Urboot           | Urboot           | Urboot           |
| Flash available user             | 7936 / 8192 bytes| 3850 / 4096 bytes| 1792 / 2048 bytes|
| RAM                              | 512 bytes        | 256 bytes        | 128 bytes        |
| EEPROM                           | 512 bytes        | 256 bytes        | 128 bytes        |
| GPIO Pins                        | 5 + RESET        | 5 + RESET        | 5 + RESET        |
| ADC Channels                     | 4 (incl RST)     | 4 (incl RST)     | 4 (incl RST)     |
| Differential ADC                 | 1/20x gain       | 1/20x gain       | 1/20x gain       |
| PWM Channels                     | 3: PB0, PB1/2    | 3: PB0, PB1/2    | 3: PB0, PB1/2    |
| Interfaces                       | USI              | USI              | USI              |
| Bootloader pins (RX/TX)          | PB1/PB0          | PB1/PB0          | PB1/PB0          |
| Int. Oscillator or PLL (MHz)     | 16, 8, 4, 2, 1   | 16, 8, 4, 2, 1   | 16, 8, 4, 2, 1   |
| External Crystal                 | All standard     | All standard     | All standard     |
| External Clock                   | All standard     | All standard     | All standard     |
| Int. WDT Oscillator              | 128 kHz          | 128 kHz          | 128 kHz          |
| LED_BUILTIN                      | PIN_PB2          | PIN_PB2          | PIN_PB2          |

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny25/45/85, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 1792, 3840, or 7936 bytes available for user code on the ATtiny25, 45, and 85 respectively. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The default serial upload pins for these chips are PB0 (TX) and PB1 (RX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader can be configured to apply an oscillator correction factor. This is exposed as a Tools menu option, with adjustable compensation ranging from -5.00% to +5.00%.

### LED_BUILTIN is on PB2
This is different than on ATtinyCore, which uses PB1.

## Features

### PLL Clock
The ATtiny25/45/85 features an on-chip PLL clocked from the internal oscillator, running at a nominal frequency of 64 MHz when enabled. It can be used in two ways, either independently or in combination.  
The system clock can be derived from one quarter of the PLL output, providing a 16 MHz clock source without an external crystal. This option inherits the accuracy limitations of the internal oscillator driving the PLL. Timer1 can be clocked directly from the PLL, enabling high-speed PWM and other timer functions independent of the system clock speed.

### Timer1 is a high speed timer
See the [TinyCore library](../libraries/TinyCore/README.md)

### PWM
The core configures PWM output as follows.
Timer0 is a standard 8-bit timer, consistent with most classic AVR devices. It provides PWM output on PB0 and PB1 with independent duty cycles.

Timer1 is a more capable but also more complex timer. By default it is clocked from the main peripheral clock (`F_CPU`), but it can be reconfigured to use the on-chip PLL via the included [TinyCore library](../libraries/TinyCore/README.md). The PLL derives its clock from the internal oscillator multiplied by a factor of 8, yielding a nominal frequency of 64 MHz.

Inverted PWM outputs are not supported by the core, as the hardware provides no means to enable an inverted output independently of its corresponding non-inverted output. If inverted output is required, it must be configured manually via direct register writes. Note that calling `digitalWrite()` or `analogWrite()` on any pin associated with a manually configured timer may interfere with that configuration and should be avoided.

### PWM frequency
TC0 is always run in Fast PWM mode: We use TC0 for millis, and phase correct mode can't be used on the millis timer - you need to read the count to get micros, but that doesn't tell you the time in phase correct mode because you don't know if it's upcounting or downcounting in phase correct mode. TC1 has no phase correct mode. However because of the flexible prescaler, we can always keep the output from TC1 between 488 and 977 Hz, our target range.

| F_CPU  | F_PWM<sub>TC0</sub> | F_PWM<sub>TC1</sub>   | Notes                        |
|--------|---------------------|-----------------------|------------------------------|
| 1  MHz | 1/8/256=     488 Hz |  1/8/256=      488 Hz |                              |
| 2  MHz | 2/8/256=     977 Hz |  2/16/256=     488 Hz |                              |
| <4 MHz | x/8/256= 488 * x Hz |  x/16/256= 244 * x Hz |                              |
| 4  MHz | 4/8/256=    1960 Hz |  4/16/256=     977 Hz |                              |
| <8 MHz | x/64/256= 61 * x Hz |  x/32/256= 122 * x Hz |                              |
| 8  MHz | 8/64/256=    488 Hz |  8/64/256=     488 Hz |                              |
| >8 MHz | x/64/256= 61 * x Hz |  x/64/256=  61 * x Hz |                              |
| 12 MHz | 12/64/256=   735 Hz | 12/64/256=     735 Hz |                              |
| 16 MHz | 16/64/256=   977 Hz | 16/64/256=     977 Hz |                              |
|>16 MHz | x/64/256= 61 * x Hz | x/128/256=  31 * x Hz |                              |
| 20 MHz | 20/64/256=  1220 Hz | 20/128/256=    610 Hz |                              |

Where speeds above or below a certain speed are specified, it's implied that the other end of the range is the next marked value. So >16 in that table is for 16-20 MHz clocks. The formula is given as a constant times x where x is expressed as MHz (the division above gets the time in megahertz - in the interest of readability I did not include the MHz to Hz conversion - I'm sure you all know how to multiply by a million)
For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C support
The ATtiny25/45/85 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI Support
The ATtiny25/45/85 does not feature a dedicated SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library.

Only the built-in SPI library is officially supported. If you encounter issues when using a third-party SPI library, please contact that library's author or maintainer, compatibility with USI-based hardware is their responsibility.

### UART (Serial) Support
The ATtiny25/45/85 does not feature a hardware UART. When operating from the internal oscillator, clock calibration may be necessary to achieve sufficient timing accuracy for reliable UART communication.

The core provides a built-in software serial implementation exposed as `Serial`. To avoid conflicts with libraries and applications that rely on pin change interrupts (PCINTs), it uses the analog comparator pins and their dedicated interrupt. The default pin assignment is AIN0 for TX and AIN1 for RX.

Being a software implementation, `Serial` cannot transmit and receive simultaneously. The `SoftwareSerial` library may be used alongside the built-in `Serial`, but the same half-duplex limitation applies, only one channel across both instances can be active at a time. If simultaneous TX/RX or the use of multiple serial interfaces is required, a device with a hardware UART should be used instead. Note that software serial has both an [upper and a lower baud rate limit](Ref_TinySoftSerial.md).

The TX pin can be reassigned to any pin using `Serial.setTxBit(n)`, where n corresponds to the pin number in PBn notation. To disable the RX channel and use TX only, select TX only from the Software Serial menu under Tools. To disable TX, refrain from printing to Serial and configure the pin to the desired mode after calling `Serial.begin()`.

### Tone
`tone()` is implemented using Timer1. If Timer1 has been configured for high-speed operation, `tone()` will produce frequencies two or four times higher than expected, normal speed operation is recommended when using `tone()`.

For best results, use pin 1 or 4 as the tone output pin. On these pins, `tone()` drives Timer1's output compare unit directly rather than toggling the pin via interrupt, which extends the usable frequency range into the MHz region. When `SoftwareSerial` or the built-in `Serial` is active, `tone()` remains functional on pins 1 and 4 but is unavailable on all other pins. Note that `tone()` disables PWM on pins 1 and 4 for the duration of its use.

### Servo Support
A built-in Servo library is included with this core, with full support for the ATtiny25/45/85. Note that any ongoing software serial activity, whether through `SoftwareSerial` or the built-in `Serial`, will cause glitches in the servo signal during transmission or reception.

### Servo and Tone Conflicts
The Servo library and `tone()` both require exclusive control of Timer1, which has two important consequences. PWM is unavailable on the Timer1 pins (PB4 and, by default, PB1) whenever `tone()` is active or the Servo library is in use. Additionally, the Servo library and `tone()` cannot be used simultaneously.

## ADC Features
The ATtiny25/45/85 has a differential ADC. Gain of 1x or 20x is available, and two differential pairs are available. The ADC supports both bipolar mode (-512 to 511) and unipolar mode (0-1023) when taking differential measurements; you can set this using `setADCBipolarMode(true or false)`.

### ADC Reference options
The ATtiny25/45/85 has both the 1.1V and 2.56V reference options, a rarity among the classic tinyAVR parts. It even supports an external reference, not that you can usually spare the pins to use one.

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
The ADC can act as a differential ADC with selectable 1x or 20x gain. It can operate in either unipolar or bipolar mode, and the polarity of the two differential pairs can be inverted in order to maximize the resolution available in unipolar mode. TinyCore wraps the IPR bit into the name of the differential ADC channel. Note the organization of the channels - there are two differential pairs A0/A1, and A2/A3 - but there is no way to take a differential measurement between A0 and A2 or A3 for example.

| Positive   | Negative   |Gain | Name            | Notes            |
|------------|------------|-----|-----------------|------------------|
| ADC0 (PB5) | ADC0 (PB5) | 1x  | DIFF_A0_A0_1X   | For offset cal.  |
| ADC0 (PB5) | ADC0 (PB5) | 20x | DIFF_A0_A0_20X  | For offset cal.  |
| ADC0 (PB5) | ADC1 (PB2) | 1x  | DIFF_A0_A1_1X   |                  |
| ADC0 (PB5) | ADC1 (PB2) | 20x | DIFF_A0_A1_20X  |                  |
| ADC1 (PB2) | ADC0 (PB5) | 1x  | DIFF_A1_A0_1X   | Input Reversed   |
| ADC1 (PB2) | ADC0 (PB5) | 20x | DIFF_A1_A0_20X  | Input Reversed   |
| ADC2 (PB4) | ADC2 (PB4) | 1x  | DIFF_A2_A2_1X   | For offset cal.  |
| ADC2 (PB4) | ADC2 (PB4) | 20x | DIFF_A2_A2_20X  | For offset cal.  |
| ADC2 (PB4) | ADC3 (PB3) | 1x  | DIFF_A2_A3_1X   |                  |
| ADC2 (PB4) | ADC3 (PB3) | 20x | DIFF_A2_A3_20X  |                  |
| ADC3 (PB3) | ADC2 (PB4) | 1x  | DIFF_A3_A2_1X   | Input Reversed   |
| ADC3 (PB3) | ADC2 (PB4) | 20x | DIFF_A3_A2_20X  | Input Reversed   |

### Temperature Measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.

### Tuning Constant Locations
These are the recommended locations to store tuning constants. In the case of OSCCAL, they are what are checked during startup when a tuned configuration is selected. They are not otherwise used by the core.

ISP programming: Make sure to have EESAVE fuse set, stored in EEPROM

| Tuning Constant        | Location EEPROM |
|------------------------|-----------------|
| Temperature Offset     | E2END - 4       |
| Temperature Slope      | E2END - 3       |
| Unspecified            | N/A             |
| Tuned OSCCAL 12 MHz    | E2END   2       |
| Tuned OSCCAL 8.25 MHz  | E2END - 1       |
| Tuned OSCCAL 8 MHz     | E2END           |

