# ATtiny43U
![43 pin mapping](Pinout_43.png "Arduino Pin Mapping for ATtiny 43")

| Specification                    | ATtiny43U          |
|----------------------------------|--------------------|
| Bootloader (occupies 256 bytes)  | Urboot             |
| Uploading uses                   | ISP/SPI pins       |
| Flash available user             | 3840 / 4096 bytes  |
| RAM                              | 256 bytes          |
| EEPROM                           | 64 bytes           |
| GPIO Pins                        | 16 (incl RST)      |
| ADC Channels                     | 4                  |
| Differential ADC                 | No                 |
| PWM Channels                     | 4                  |
| Interfaces                       | USI                |
| Int. Oscillator (MHz)            | 8, 4, 2, 1         |
| External Crystal                 | Not supported      |
| External Clock                   | All standard       |
| Int. WDT Oscillator              | 128 kHz            |
| LED_BUILTIN                      | PIN_PA5            |

## Overview
The ATtiny43 is an unusual device with an otherwise unremarkable peripheral set, distinguished by a single standout feature: a built-in boost converter that allows operation from as little as 1.1V at startup, continuing down to 0.7V, enabling a complete project to run from a single alkaline battery. The boost converter generates approximately 3V during active operation and can supply up to 30mA for peripherals. When the boost converter is in use, the clock speed must not exceed 4 MHz. The "Internal 4MHz" clock option configures the fuses to start at 1 MHz and switch to 4 MHz during startup. Refer to the datasheet for the required external components, PCB layout guidelines, and further details of boost converter operation.

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny261/461/861, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 1792, 3840, or 7936 bytes available for user code on the ATtiny261, 461, and 861 respectively. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
The default serial upload pins for these chips are PA4 (TX) and PA5 (RX). The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader can be configured to apply an oscillator correction factor. This is exposed as a Tools menu option, with adjustable compensation ranging from -5.00% to +5.00%.

### Boost Converter Operation
The boost converter starts up at a battery voltage of 1.2V according to the datasheet, though in practice startup has been observed at 1.1V. It generates a regulated 3V output and continues operating down to a battery voltage of 0.8V or potentially lower. It can supply up to 30mA to external devices provided VBat remains above 1.0V; below this threshold the maximum deliverable current decreases, and the 3V output is no longer guaranteed under load.

The primary advantage over an external boost converter becomes apparent in power-sensitive designs where the processor spends a significant portion of its time in sleep modes. In Active Low Current mode, the output voltage is permitted to droop during sleep, allowing the converter to operate at a very low duty cycle and consume minimal power. When the processor wakes, the converter ramps the supply back up. This behaviour is handled automatically by the chip, and results in substantially lower average current consumption compared to a continuously regulated external boost converter under the same conditions.

### PWM frequency
TC0 is always run in Fast PWM mode: We use TC0 for millis, and phase correct mode can't be used on the millis timer - you need to read the count to get micros, but that doesn't tell you the time in phase correct mode because you don't know if it's upcounting or downcounting in phase correct mode. On this part, the TC1 is uniquely bad - it has a different, shorter list of possible WGMs, and is only 8 bits.

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

Phase correct PWM counts up to 255, turning the pin off as it passes the compare value, updates it's double-buffered registers at TOP, then it counts down to 0, flipping the pin back as is passes the compare value. This is considered preferable for motor control applications; the "Phase and Frequency Correct" mode is not available on this part.

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C support
The ATtiny43 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI support
The ATtiny43 does not feature a dedicated SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library.

### UART (Serial) Support
The ATtiny43 does not feature a hardware UART. When operating from the internal oscillator, clock calibration may be necessary to achieve sufficient timing accuracy for reliable UART communication.

The core provides a built-in software serial implementation exposed as `Serial`. To avoid conflicts with libraries and applications that rely on pin change interrupts (PCINTs), it uses the analog comparator pins and their dedicated interrupt. The default pin assignment is AIN0 for TX (PA4) and AIN1 for RX (PA5).

Being a software implementation, `Serial` cannot transmit and receive simultaneously. The `SoftwareSerial` library may be used alongside the built-in `Serial`, but the same half-duplex limitation applies, only one channel across both instances can be active at a time. If simultaneous TX/RX or the use of multiple serial interfaces is required, a device with a hardware UART should be used instead. Note that software serial has both an [upper and a lower baud rate limit](Ref_TinySoftSerial.md).

The TX pin can be reassigned to any pin on PORTA using `Serial.setTxBit(n)`, where n corresponds to the pin number in PAn notation. To disable the RX channel and use TX only, select TX only from the Software Serial menu under Tools. To disable TX, refrain from printing to Serial and configure the pin to the desired mode after calling `Serial.begin()`.

### Tone Support
Tone() uses Timer1. For best results, use pin 5 or 6 (PB5, PB6) as this will use the hardware output compare to generate the square wave instead of using interrupts. Any use of tone() will take out PWM on pins PB5 and PB5. It doesn't do a great job because of the limitations of the timer these parts have.

### Servo support
The Servo library is not supported on the ATtiny43. Servos require 5-6V, which is incompatible with the 1.8-2V supply provided by the on-chip boost converter. Additionally, the ATtiny43 timer is not well suited to servo signal generation, and any implementation would be difficult to produce and unsatisfactory in practice.

### ADC Reference options
| Reference Option | Description               |
|------------------|---------------------------|
| `DEFAULT`        | Vcc                       |
| `INTERNAL1V1`    | Internal 1.1V reference   |
| `INTERNAL`       | Synonym for `INTERNAL1V1` |


### Temperature Measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.

### Tuning Constant Locations
These are the recommended locations to store tuning constants. In the case of OSCCAL, they are what are checked during startup when a tuned configuration is selected. They are not otherwise used by the core.

ISP programming: Make sure to have EESAVE fuse set, stored in EEPROM

|------------------------|-----------------|
| Temperature Offset     | E2END - 2       |
| Temperature Slope      | E2END - 1       |
| Tuned OSCCAL 8 MHz     | E2END           |
