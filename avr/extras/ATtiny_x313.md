# ATtiny 2313/4313
![x313 pin mapping](Pinout_x313.png "Arduino Pin Mapping for ATtiny x313-family")

| Specification                    | ATtiny4313         | ATtiny2313/A       |
|----------------------------------|--------------------|--------------------|
| Bootloader (occupies 256 bytes)  | Urboot             | Urboot             |
| Flash available user             | 3840 / 4096 bytes  | 1792 / 2048 bytes  |
| RAM                              | 256 bytes          | 128 bytes          |
| EEPROM                           | 64 bytes           | 64 bytes           |
| GPIO Pins                        | 17 + RESET         | 17 + RESET         |
| ADC Channels                     | None               | None               |
| PWM Channels                     | 4                  | 4                  |
| Interfaces                       | USI, USART         | USI, USART         |
| Int. Oscillator (MHz)            | 8, 4, 2, 1         | 8, 4, 2, 1         |
| External Crystal                 | All standard       | All standard       |
| External Clock                   | All standard       | All standard       |
| Int. WDT Oscillator              | 128 kHz            | 128 kHz            |
| LED_BUILTIN                      | PIN_PB4            | PIN_PB4            |

### Overview
The ATtiny2313/4313 are one of the older chips in the ATtiny lineup. They have a hardware serial port and USI (Universal Serial Interface), but does not have an analog to digital converter. The ATtiny2313 (non-A) does not have PCINT interrupts on all pins, and you have to be spesific about if you're using the ATtiny2313 or ATtiny2313A for hardware debugging. Read more about the differences between these two chips [here](https://ww1.microchip.com/downloads/en/Appnotes/doc8261.pdf).

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny2313/4313, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 1792 or 3840 bytes available for user program. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The serial upload pins for these chips are PD0 (RX) and PD1 (TX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader has "autobaud functionality", which means that it will try to adjust and match the host baud rate.

### Internal oscillator calibration
The internal 8 MHz oscillator is not highly accurate, which is acceptable for many applications but insufficient for asynchronous protocols such as UART, where a frequency error of ±3-4% will cause communication to fail. The oscillator speed is highly voltage dependent, and is usually too fast when running the target at 5V.

To address this, TinyCore provides an [Oscillator calibration sketch](../libraries/TinyCore/examples/OscillatorCalibration/OscillatorCalibration.ino) that calculates a corrected OSCCAL value based on characters received over UART. It uses the default UART pins, **TX = PD1** and **RX = PD0**. Before uploading the sketch, ensure the target is running from its internal 8 MHz oscillator and that EEPROM preservation is enabled. Once uploaded, open the serial monitor at 115200 baud, select "No line ending", and repeatedly send the character `x`. After a few attempts, readable text should begin to appear in the serial monitor. Once the calibration value has stabilised, it is automatically stored in the last byte of EEPROM for future use. This value is not loaded automatically and must be applied explicitly in your sketch:

```cpp
  // Check if there exists any OSCCAL value in the last EEPROM byte
  // If not, run the oscillator tuner sketch first
  uint8_t cal = EEPROM.read(E2END);
  if (cal < 0xff)
    OSCCAL0 = cal;
```

Another approach is to use the [avrCalibrate](https://github.com/felias-fogg/avrCalibrate) library, which uses a host microcontroller along with the target to perform the calibraion.


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


### I2C support
The ATtiny2313/4313 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI support
There is no hardware SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library.

Only the built-in SPI library is officially supported. If you encounter issues when using a third-party SPI library, please contact that library's author or maintainer, compatibility with USI-based hardware is their responsibility.

### UART (Serial) support
There is one full hardware serial port, named `Serial`. It works the same as `Serial` on any normal Arduino - it is not a software implementation. Be aware that due to the limited memory on these chips the buffers are quite small.

To use only TX or only RX channel, after Serial.begin(), one of the following commands will disable the TX or RX channels
```c
UCSRB &= ~(1 << TXEN); // disable TX
UCSRB &= ~(1 << RXEN); // disable RX
```

### Tone support
`tone()` uses Timer1. For best results, use pin PB3 (12) or PB4 (13), as this will use the hardware output compare to generate the square wave instead of using interrupts. `tone()` will disable PWM on PB3 and PB4.

### Servo support
The standard Servo library is hardcoded to work on specific parts only, we include a builtin Servo library that supports the ATtiny2313/4313 though getting everything to fit may be challenging. As always, while a software serial port (including the builtin one, Serial, on these ports, see below) is receiving or transmitting, the servo signal will glitch. See [the Servo/Servo_TinyCore library](../libraries/Servo/README.md) for more details. Like `tone()`, it takes PWM on PB3 and PB4. It is not compatible with Tone.

### There is no ADC
analogRead() is not defined on these parts.
