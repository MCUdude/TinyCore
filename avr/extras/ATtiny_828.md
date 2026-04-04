# ATtiny828/R

The ATtiny828 is an unusual tinyAVR device offering 28 GPIO pins (incl. RST), a hardware UART, hardware SPI, and a hardware I2C slave; a peripheral combination not found on any other classic ATtiny. It is available in a 32-pin TQFP or QFN package only. A notable silicon errata affects PD3, which cannot function as a digital input unless the ULP oscillator is running; this also impacts the hardware SPI and I2C slave peripherals that share that pin. Despite its rich peripheral set, the ATtiny828 lacks support for an external crystal, but can be clocked by an external clock. The ATtiny1634R has a more accurate internal oscillator.

| Pinout diagram                        | Minimal setup schematic                             |
|---------------------------------------|-----------------------------------------------------|
|<img src="Pinout_828.png" width="180"> | <img src="ATtiny828_minimal_setup.png" width="300"> |

## Table of contents
- [Specifications](#specifications)
- [Urboot bootloader](#urboot-bootloader)
- [Warning: PD3 Non-functional as input without watchdog timer](#warning-pd3-non-functional-as-input-without-watchdog-timer)
- [Internal oscillator calibration](#internal-oscillator-calibration)
- [Features](#features)
  - [PWM frequency](#pwm-frequency)
  - [Tone support](#tone-support)
  - [Servo support](#servo-support)
  - [I2C support](#i2c-support)
  - [SPI support](#spi-support)
  - [UART (Serial) support](#uart-serial-support)
  - [ADC support](#adc-support)
  - [ADC reference options](#adc-reference-options)
- [Weird I/O-pin related features](#weird-io-pin-related-features)
  - [PD3 silicon errata](#pd3-silicon-errata)
  - [Special "high sink" port](#special-high-sink-port)
  - [Separate pullup-enable register](#separate-pullup-enable-register)

### Specifications

| Specification                     | ATtiny828            |
|-----------------------------------|----------------------|
| Bootloader (occupies 256 bytes)   | Urboot               |
| Flash available (no bootloader)   | 8192 bytes           |
| Flash available (with bootloader) | 7936 bytes           |
| RAM                               | 512 bytes            |
| EEPROM                            | 256 bytes            |
| GPIO Pins                         | 28 (incl RST)        |
| ADC Channels                      | 28 (incl RST)        |
| PWM Channels                      | 4                    |
| Interfaces                        | UART, SPI, I2C slave |
| Int. Oscillator (MHz)             | 8, 4, 2, 1           |
| External Crystal                  | Not supported        |
| External Clock                    | All standard         |
| Int. WDT Oscillator               | Not supported        |
| LED_BUILTIN                       | PIN_PB0              |

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny828, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 7936 bytes available for user program. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. This chip has a hardware serial port, UART0, so Urboot is configured to use this.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The serial upload pins for this chip is PC2 (RX) and PC3 (TX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader has "autobaud functionality", which means that it will try to adjust and match the host baud rate.

### Warning: PD3 non-functional as input without watchdog timer
A hardware errata documented in the datasheet describes a significant limitation of PD3: it cannot function as an input unless the ULP oscillator (used by the watchdog timer, among other things) is running. When the ULP oscillator is inactive, PD3 is internally pulled low. This has an additional consequence: if PD3 is configured as an output and driven high, it will continuously draw current even with no external load. Power-saving sleep mode should never be used with PD3 configured as an output driven high.

A software workaround is available, but the pin remains less capable than it should be and is best restricted to active output while the processor is awake, such as through its PWM functionality, unless the WDT/ULP oscillator is kept running.

This errata is particularly unfortunate because PD3 is the USI clock pin, used by both I2C slave and SPI interfaces. The unwanted pull-down renders I2C inoperable when the WDT is disabled, as the open-drain bus cannot be driven high reliably. For this reason, when the Wire library is used in master-only mode (which uses a software I2C implementation), the I2C pins are remapped to PA4 and PA5, avoiding PORTD entirely and sidestepping this issue.

### Internal oscillator calibration
The internal 8 MHz oscillator is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail.

The Arduino IDE Tools menu lets you select `(Vcc > 4.5V)` and `(Vcc < 4.5V)` clock options, where the `(Vcc > 4.5V)` option just subtract 6 counts from the `OSCCAL0` register, to compensate for the clock (most likely) being too fast at more than 4.5V. However, subtracting six counts is just an educated guess, and proper tuning may be necessary if accuracy is important

To address this, TinyCore provides an [Oscillator calibration sketch](../libraries/TinyCore/examples/OscillatorCalibration/OscillatorCalibration.ino) that calculates a corrected OSCCAL value based on characters received over UART. It uses the default UART pins, **TX = PC3** and **RX = PC2**. Before uploading the sketch, ensure the target is running from its internal 8 MHz oscillator and that EEPROM preservation is enabled. Once uploaded, open the serial monitor at 115200 baud, select "No line ending", and repeatedly send the character `x`. After a few attempts, readable text should begin to appear in the serial monitor. Once the calibration value has stabilised, it is automatically stored in the last byte of EEPROM for future use. This value is not loaded automatically and must be applied explicitly in your sketch:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL0 = cal;
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

Phase correct PWM counts up to 255, turning the pin off as it passes the compare value, updates it's double-buffered registers at TOP, then it counts down to 0, flipping the pin back as is passes the compare value. This is considered preferable for motor control applications, though the "Phase and Frequency Correct" mode is better if the period is ever adjusted by a large amount at a time, because it updates the doublebuffered registers at BOTTOM, and thus produces a less problematic glitch in the duty cycle, but doesn't have any modes that don't require setting ICR1 too.

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### Tone support
Tone() uses Timer1. For best results, use pin 21 or 23 (PC5, PC7), as this will use the hardware output compare to generate the square wave instead of using interrupts. Any use of tone() will disable PWM on pins 21 and 23.

### Servo support
A built-in Servo library is included with this core, with full support for the ATtiny828. Note that any ongoing software serial activity, whether through `SoftwareSerial` or the built-in `Serial`, will cause glitches in the servo signal during transmission or reception. The Servo library disables PWM on PC5 and PC7, and cannot be used simultaneously with `tone()`. See the [the Servo/Servo_TinyCore library](../libraries/Servo/README.md) for further details.

### I2C support
Slave I2C functionality is provided in hardware, but a software implementation must be used for master functionality. This is done automatically with the included version of the Wire.h library. **You must have external pullup resistors installed** in order for I2C functionality to work reliably. Furthermore, the slave functionality requires the WDT to be enabled, otherwise the SCL pin will be pulled low due to a silicon bug. Slave and master I2C use different pins because of this godawful bug. We only support use of the builtin universal Wire.h library. If you try to use other libraries and encounter issues, please contact the author or maintainer of that library - there are too many of these poorly written libraries for us to provide technical support for.

| Pins | I2C master | I2C slave          |
|------|------------|--------------------|
| SDA  | PA4        | PD0                |
| SCL  | PA5        | PD3 (WDT required) |

### SPI support
There is full Hardware SPI support. However, PD3 is one of the pins used by the hardware SPI; you must use the WDT workaround for the PD3 silicon bug if using SPI. Third party SPI libraries designed for tinyAVRs are not supported by the hardware and will not work.

### UART (Serial) support
There is one full hardware serial port, `Serial`. It works the same as `Serial` on any normal Arduino - it is not a software implementation. Be aware that due to the limited memory on these chips the buffers are quite small.

To use only TX or only RX channel, after Serial.begin(), one of the following commands will disable the TX or RX channels
```c
UCSRB &= ~(1 << TXEN); // disable TX
UCSRB &= ~(1 << RXEN); // disable RX
```


### ADC support
There is abundant evidence that this device was intended to have a differential ADC; the register layout matches that of the ATtiny841 which has a vere nice differential ADC, except that all the register bits that would be involved in that are marked reserved. If that wasn't enough to convince you, reading that chapter of the datasheet, it is clear that references to a differential ADC were scrubbed in a hurry at the last minute. I suspect it was found to be afflicted by a fatal flaw, whose workaround if any was to onerouos or the nature of the mistake too humiliating to present to customers, managemnt denied the request for a respin to fix it, and they responded by removing it on paper only - and is still actually in the silicon (if they had time to respin, they'd have fixed it, I suspect).  See the link at the top of this page - if you like poking at "reserved" registers and trying to find secret features, and have time on your hands, it could be a lot of fun, and if you solve the mystery, I'll mail you some free boards of your choice from my store.

#### ADC reference options
Despite having 28 ADC input channels, the 828 only has the two basic reference options.

| Reference Option | Description               |
|------------------|---------------------------|
| `DEFAULT`        | Vcc                       |
| `INTERNAL1V1`    | Internal 1.1V reference   |

### Weird I/O-pin related features
There are a few strange features relating to the GPIO pins on the ATtinyx41 family which are found only in a small number of other parts released around the same time.

#### PD3 silicon errata
As mentioned above, the t828 has a serious silicon bug PD3, made all the worse by the important alternative functions of that pin.

If you have no need to use the WDT, but do have a need to use PD3 as an input, you can keep the WDT running by putting it into interrupt mode, with an empty interrupt, at the cost of just 10b of flash, an ISR that executes in 11 clock cycles every 8 seconds, and an extra 1-4uA of power consumption (negligible compared to what the chip consumes when not sleeping, and you'll turn it off while sleeping anyway - see below) - so the real impact of this issue is in fact very low, assuming you know about it and don't waste hours or days trying to figure out what is going on.

```c
//put these lines in setup
CCP=0xD8; //write key to configuration change protection register
WDTCSR=(1<<WDP3)|(1<<WDP0)|(1<<WDIE); //enable WDT interrupt with longest prescale option (8 seconds)
//put this empty WDT ISR outside of all functions
EMPTY_INTERRUPT(WDT_vect) //empty ISR to work around bug with PB3. EMPTY_INTERRUPT uses 26 bytes less than ISR(WDT_vect){;}
```
If you are using sleep modes, you also need to turn the WDT off while sleeping (both because the interrupts would wake it, and because the WDT is consuming power, and presumably that's what you're trying to avoid by sleeping). Doing so as shown below only uses an extra 12-16 bytes if you call it from a single place, 20 if called from two places, and 2 bytes when you call it thereafter, compared to calling sleep_cpu() directly in those places, as you would on a part that didn't need this workaround.
```c
void startSleep() { //call instead of sleep_cpu()
  CCP=0xD8; //write key to configuration change protection register
  WDTCSR=0; //disable WDT interrupt
  sleep_cpu();
  CCP=0xD8; //write key to configuration change protection register
  WDTCSR=(1<<WDP3)|(1<<WDP0)|(1<<WDIE); //enable WDT interrupt
}
```


#### Special "high sink" port
All pins on PORTC have unusually high sink capability - when sinking a given amount of current, the voltage on these pins is about half that of typical pins. Using the `PHDE` register, these can be set to sink even more aggressively.

```c
PHDE=(1<<PHDEC);
```

This is no great shakes - the Absolute Maximum current rating of 40mA still applies and all... but it does pull closer to ground with a a "large" 10-20mA load. A very strange feature of these parts. The PWM outputs of go on this this port as well, making it of obvious utility for driving LEDs and similar. This also means that, if you are attempting to generate an analog voltage with a PWM pin and an RC filter, your result may be lower than expected, as the pin drivers are not symmetric.

#### Separate pullup-enable register
Like the ATtinyx41 and ATtiny1634, these have a fourth register for each port, PUEx, which controls the pullups (rather than PORTx when DDRx has pin set as input). Like the ATtiny1634, and unlike the ATtiny841, all of these pullup registers are located in the low IO space and sensibly relative to each other and to the other PORT-related registers.
