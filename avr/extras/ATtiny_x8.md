# ATtiny 48/88
![x8 Pin Mapping](Pinout_x8.png "Arduino Pin Mapping for ATtiny 88/48 in TQFP")
![x8 Pin Mapping](Pinout_x8_PU.png "Arduino Pin Mapping for ATtiny 88/48 in DIP")

| Specification                    | ATtiny88           | ATtiny48           |
|----------------------------------|--------------------|--------------------|
| Bootloader (occupies 256 bytes)  | Urboot             | Urboot             |
| Flash available user             | 7936 / 8192 bytes  | 3840 / 4096 bytes  |
| RAM                              | 512 bytes          | 256 bytes          |
| EEPROM                           | 64 bytes           | 64 bytes           |
| GPIO Pins                        | 26 + RESET         | 26 + RESET         |
| ADC Channels                     | 8 (6 in DIP pack.) | 8 (6 in DIP pack.) |
| PWM Channels                     | 2                  | 2                  |
| Interfaces                       | SPI, I2C           | SPI, I2C           |
| Int. Oscillator (MHz)            | 8, 4, 2, 1         | 8, 4, 2, 1         |
| External Crystal                 | Not supported      | Not supported      |
| External Clock                   | All Standard       | All Standard       |
| Int. WDT Oscillator              | 128 kHz            | 128 kHz            |
| LED_BUILTIN                      | PIN_PB5            | PIN_PB5            |

## Overview
The ATtiny48/88 is designed as a low-cost alternative compatible with the popular ATmega48/888 series, sharing a nearly identical pinout with a few additional GPIO pins available in the TQFP package. While the ATtiny48/88 includes full hardware I2C and SPI peripherals, it lacks both a hardware UART and support for an external crystal oscillator.

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny48/88, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 3840 or 7936 bytes available for user code on the ATtiny48 and 88 respectively. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The default serial upload pins for these chips are PD6 (TX) and PD7 (RX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader can be configured to apply an oscillator correction factor. This is exposed as a Tools menu option, with adjustable compensation ranging from -5.00% to +5.00%.

### Internal oscillator calibration
The internal 8 MHz oscillator is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail.

To address this, TinyCore provides an [Oscillator calibration sketch](../libraries/TinyCore/examples/OscillatorCalibration/OscillatorCalibration.ino) that calculates a corrected OSCCAL value based on characters received over UART. It uses the default UART pins, **TX = PD6** and **RX = PD7**. Before uploading the sketch, ensure the target is running from its internal 8 MHz oscillator and that EEPROM preservation is enabled. Once uploaded, open the serial monitor at 115200 baud, select "No line ending", and repeatedly send the character `x`. After a few attempts, readable text should begin to appear in the serial monitor. Once the calibration value has stabilised, it is automatically stored in the last byte of EEPROM for future use. This value is not loaded automatically and must be applied explicitly in your sketch:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL = cal;
```

Another approach is to use the [avrCalibrate](https://github.com/felias-fogg/avrCalibrate) library, which uses a host microcontroller along with the target to perform the calibraion. avrCalibrate can also calibrate internal voltage references.

### External Clock
The ATtiny48/88 does not support an external crystal oscillator, but does accept an external clock signal on PB6 (CLKI). External clock generators typically come in the same rectangular metal package as crystals, but can be distinguished by their pin count: clock generators use all four pins (Vcc, GND, CLKOUT, and Enable), whereas crystals use only two, or four with two unconnected. The Enable pin is generally active-high with an internal weak pull-up.

If "Burn Bootloader" is performed with an external clock source selected, but a crystal is connected instead, the chip will become unresponsive to programming until a valid clock signal is provided. This will manifest as a signature mismatch (0x000000 with verbose upload output enabled). To recover, remove the crystal and supply a clock signal of 2-8 MHz on PB6. The [Adafruit ArduinoAsISP sketch](https://github.com/adafruit/ArduinoISP) outputs a suitable clock signal on PB1 (digital pin 9) during bootloader burning.


## Features

### PWM frequency
TC0 is always run in Fast PWM mode: We use TC0 for millis, and phase correct mode can't be used on the millis timer - you need to read the count to get micros, but that doesn't tell you the time in phase correct mode because you don't know if it's upcounting or downcounting in phase correct mode.

| F_CPU  | No PWM from TC0     | F_PWM<sub>TC1</sub>   | Notes                        |
|--------|---------------------|-----------------------|------------------------------|
| 1  MHz |                   - |  1/8/256=      488 Hz |                              |
| 2  MHz |                   - |  2/8/256=      977 Hz |                              |
| <4 MHz |                   - |  x/8/512=  244 * x Hz | Phase correct TC1            |
| 4  MHz |                   - |  4/8/512=      977 Hz | Phase correct TC1            |
| <8 MHz |                   - |  x/8/512=  244 * x Hz | Between 4 and 8 MHz, the target range is elusive | Phase correct TC1 |
| 8  MHz |                   - |  8/64/256=     488 Hz |                              |
| >8 MHz |                   - |  x/64/256=  61 * x Hz |                              |
| 12 MHz |                   - | 12/64/256=     735 Hz |                              |
| 16 MHz |                   - | 16/64/256=     977 Hz |                              |
|>16 MHz |                   - |  x/64/512=  31 * x Hz | Phase correct TC1            |
| 20 MHz |                   - | 20/64/512=     610 Hz | Phase correct TC1            |

Phase correct PWM counts up to 255, turning the pin off as it passes the compare value, updates it's double-buffered registers at TOP, then it counts down to 0, flipping the pin back as is passes the compare value. This is considered preferable for motor control applications, though the "Phase and Frequency Correct" mode is better if the period is ever adjusted by a large amount at a time, because it updates the doublebuffered registers at BOTTOM, and thus produces a less problematic glitch in the duty cycle, but doesn't have any modes that don't require setting ICR1 too.

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C support
The ATtiny48/88 features a full hardware I2C peripheral, supported by the standard Wire library. Third-party I2C libraries targeting tinyAVR parts should not be used, as these are typically written for devices that implement I2C through the USI peripheral rather than a dedicated I2C module, and are incompatible with the ATtiny48/88 hardware.

### SPI support
The ATtiny48/88 features a full hardware SPI peripheral, supported by the standard SPI library. Third-party libraries targeting tinyAVR parts that rely on USI-based implementations such as USIWire should not be used, as the ATtiny48/88 is among the few tinyAVR devices with a dedicated SPI peripheral and does not use the USI for SPI communication.

### UART (Serial) support
The ATtiny48/88 does not feature a hardware UART. When operating from the internal oscillator, clock calibration may be necessary to achieve sufficient timing accuracy for reliable UART communication.

The core provides a built-in software serial implementation exposed as `Serial`. To avoid conflicts with libraries and applications that rely on pin change interrupts (PCINTs), it uses the analog comparator pins and their dedicated interrupt. The default pin assignment is AIN0 for TX and AIN1 for RX.

Being a software implementation, `Serial` cannot transmit and receive simultaneously. The `SoftwareSerial` library may be used alongside the built-in `Serial`, but the same half-duplex limitation applies, only one channel across both instances can be active at a time. If simultaneous TX/RX or the use of multiple serial interfaces is required, a device with a hardware UART should be used instead. Note that software serial has both an [upper and a lower baud rate limit](Ref_TinySoftSerial.md).

The TX pin can be reassigned to any pin on PORTD using `Serial.setTxBit(n)`, where n corresponds to the pin number in Pxn notation. To disable the RX channel and use TX only, select TX only from the Software Serial menu under Tools. To disable TX, refrain from printing to Serial and configure the pin to the desired mode after calling `Serial.begin()`.

### Tone support
`tone()` uses Timer1. For best results, use PB1 or PB2, as this will use the hardware output compare to generate the square wave instead of using interrupts. Using `tone()` will disable all PWM pins.

### Servo support
A built-in Servo library is included with this core, with full support for the ATtiny48/88. Note that any ongoing software serial activity, whether through `SoftwareSerial` or the built-in `Serial`, will cause glitches in the servo signal during transmission or reception. The Servo library disables all PWM pins, and cannot be used simultaneously with `tone()`. See the [Servo/Servo_TinyCore library](../libraries/Servo/README.md) for more details.

## ADC features
The ATtiny48/88 features a straightforward single-ended ADC with no differential input capability. It provides 8 channels in the TQFP package, or 6 channels in the DIP and 28-pin QFN packages, a single internal reference voltage, and a temperature sensor. This matches the ADC found on the ATmega48/88 series.

### ADC reference options
Two reference options on the ATtiny48/88.

| Reference Option   | Reference Voltage           | Uses AREF Pin        |
|--------------------|-----------------------------|----------------------|
| `DEFAULT`          | Vcc                         | There is no AREF pin |
| `INTERNAL1V1`      | Internal 1.1V reference     | n/a                  |
| `INTERNAL`         | Alias of INTERNAL1V1        | n/a                  |

### Internal Sources

| Voltage Source  | Description                            |
|-----------------|----------------------------------------|
| ADC_INTERNAL1V1 | Reads the INTERNAL1V1 reference        |
| ADC_GROUND      | Reads ground                           |
| ADC_TEMPERATURE | Reads internal temperature sensor      |

### Temperature measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.
