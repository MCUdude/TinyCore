# ATtiny 441/841
![x41 Pin Mapping](Pinout_x41.png "Arduino Pin Mapping for ATtiny x41")

| Specification                    | ATtiny841                | ATtiny441                |
|----------------------------------|--------------------------|--------------------------|
| Bootloader (occupies 256 bytes)  | Urboot                   | Urboot                   |
| Flash available user             | 7936 / 8192 bytes        | 3840 / 4096 bytes        |
| RAM                              | 512 bytes                | 256 bytes                |
| EEPROM                           | 512 bytes                | 256 bytes                |
| GPIO Pins                        | 11 + RESET               | 11 + RESET               |
| ADC Channels                     | 12 (incl RST)            | 12 (incl RST)            |
| Differential ADC                 | Yes                      | Yes                      |
| PWM Channels                     | 6                        | 6                        |
| Interfaces                       | 2× USART, SPI, i2c slave | 2× USART, SPI, i2c slave |
| Int. Oscillator (MHz)            | 8, 4, 2, 1               | 8, 4, 2, 1               |
| External Crystal                 | All standard             | All standard             |
| External Clock                   | All standard             | All standard             |
| LED_BUILTIN                      | PIN_PB2                  | PIN_PB2                  |

### Overview
The ATtiny441/841 is an enhanced successor to the ATtiny24/44/84 family, offering pin compatibility and a significantly expanded peripheral set at a modest price premium. It is available in surface mount packages only. 

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny441/841, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 3840 or 7936 bytes available for user program. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
UART0 or UART1 can be selected in the Arduino IDE Tools menu. The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader has "autobaud functionality", which means that it will try to adjust and match the host baud rate.

## Features

### Internal Oscillator voltage dependence
Prior to 1.4.0, many users had encountered issues due to the voltage dependence of the oscillator. While the calibration is very accurate between 2.7 and 4v, as the voltage rises above 4.5v, the speed increases significantly. Although the magnitude of this is larger than on many of the more common parts, the issue is not as severe as had long been thought - the impact had been magnified by the direction of baud rate error, and the fact that many US ports actually supply 5.2-5.3v. As of 1.4.0, a simple solution was implemented to enable the same bootloader to work across the 8 MHz (Vcc < 4.5v) and 8 MHz (Vcc > 4.5 MHz ) board definitions, as well as the 16 MHz Internal option (albeit running at 8MHz) - it should generally work between 2.7v and 5.25v - though the extremes of that range may be dicey. We do still provide a >4.5v clock option in order to improve behavior of the running sketch - it will nudge the oscillator calibration down to move it closer to the nominal 8MHz clock speed; sketches uploaded with the higher voltage option. This is not perfect, but it is generally good enough to work with Serial on around 5v (including 5.25v often found on USB ports to facilitate chargeing powerhungry devices), and millis()/micros() will keep better time than in previous versions.


### PWM frequency
TC0 is always run in Fast PWM mode: We use TC0 for millis, and phase correct mode can't be used on the millis timer - you need to read the count to get micros, but that doesn't tell you the time in phase correct mode because you don't know if it's upcounting or downcounting in phase correct mode. Unique among the tinyAVRs, the x41 parts have a third timer. TC1 and TC2 are both the "good" timers, the 16-bit-capable ones.

| F_CPU  | F_PWM<sub>TC0</sub> | F_PWM<sub>TC1/2</sub> | Notes                        |
|--------|---------------------|-----------------------|------------------------------|
| 1  MHz | 1/8/256=     488 Hz |  1/8/256=      488 Hz |                              |
| 2  MHz | 2/8/256=     977 Hz |  2/8/256=      977 Hz |                              |
| <4 MHz | x/8/256= 488 * x Hz |  x/8/512=  244 * x Hz | Phase correct TC1/2          |
| 4  MHz | 4/8/256=    1960 Hz |  4/8/512=      977 Hz | Phase correct TC1/2          |
| <8 MHz | x/64/256= 61 * x Hz |  x/8/512=  244 * x Hz | Between 4 and 8 MHz, the target range is elusive | Phase correct TC1 |
| 8  MHz | 8/64/256=    488 Hz |  8/64/256=     488 Hz |                              |
| >8 MHz | x/64/256= 61 * x Hz |  x/64/256=  61 * x Hz |                              |
| 12 MHz | 12/64/256=   735 Hz | 12/64/256=     735 Hz |                              |
| 16 MHz | 16/64/256=   977 Hz | 16/64/256=     977 Hz |                              |
|>16 MHz | x/64/256= 61 * x Hz |  x/64/512=  31 * x Hz | Phase correct TC1/2          |
| 20 MHz | 20/64/256=  1220 Hz | 20/64/512=     610 Hz | Phase correct TC1/2          |

Phase correct PWM counts up to 255, turning the pin off as it passes the compare value, updates it's double-buffered registers at TOP, then it counts down to 0, flipping the pin back as is passes the compare value. This is considered preferable for motor control applications, though the "Phase and Frequency Correct" mode is better if the period is ever adjusted by a large amount at a time, because it updates the doublebuffered registers at BOTTOM, and thus produces a less problematic glitch in the duty cycle, but doesn't have any modes that don't require setting ICR1 too.

For more information see the [Changing PWM Frequency](Ref_ChangePWMFreq.md) reference.

### I2C
The ATtiny441/841 does not have a hardware I2C master. The included Wire library provides I2C master functionality through a software implementation, while the hardware I2C slave peripheral is used for slave functionality. It serves as a drop-in replacement for the standard Wire library, with the caveat that the clock speed cannot be configured. **External pull-up resistors are required** on the SDA and SCL lines for reliable operation. Note that error reporting from the software I2C master is unreliable.

### SPI support
There is hardware SPI support. Use the normal SPI module.
The ATtiny441/841 features a full hardware SPI peripheral, supported by the standard SPI library. Third-party libraries targeting tinyAVR parts that rely on USI-based implementations such as USIWire should not be used, as the ATtiny87/167 is among the few tinyAVR devices with a dedicated SPI peripheral and does not use the USI for SPI communication.

### UART (Serial) support
The ATtiny441/841 features two full hardware serial ports, exposed as `Serial` and `Serial1`, both fully compatible with the standard Arduino Serial API.

To use only the TX or RX channel, call `Serial.begin()` first, then disable the unwanted channel using the appropriate control register bit:
```c
UCSR0B &= ~(1 << TXEN0); // Disable TX on Serial
UCSR0B &= ~(1 << RXEN0); // Disable RX on Serial
```
For `Serial1`, substitute `UCSR1B` for `UCSR0B`.

### Tone support
The standard `tone()` function is supported on these parts. For best results, use PA5 or PA6, as this will use hardware output compare to generate the square wave, instead of interrupts.

### ADC features
The ATtiny441/841 features one of the most capable ADCs in the entire classic tinyAVR line. All pins are connected to the ADC and support `analogRead()`. In differential mode, every pin participates in at least three differential pairs, with selectable gain of 1x, 20x, or 100x. All differential channel combinations supported by the pin compatible ATtiny24/44/84 are available, along with a number of additional pairings.

### Reference options
The ATtiny441/841 provides three internal reference voltages, each of which can be connected to the AREF pin for external bypass capacitor filtering to improve stability. If the AREF pin is used in this way, a capacitor should be placed between AREF and GND, and the pin should not be used for any other purpose. Under typical operating conditions this is unlikely to be necessary, but becomes more relevant when using the differential ADC with gain enabled.

| Reference Option   | Reference Voltage           | Uses AREF Pin        |
|--------------------|-----------------------------|----------------------|
| `DEFAULT`          | Vcc                         | No, pin available    |
| `EXTERNAL`         | Voltage applied to AREF pin | Yes, ext. voltage    |
| `INTERNAL1V1`      | Internal 1.1V reference     | No, pin available    |
| `INTERNAL2V2`      | Internal 2.2V reference     | No, pin available    |
| `INTERNAL4V096`    | Internal 4.096V reference   | No, pin available    |
| `INTERNAL1V1_CAP`  | Internal 1.1V reference     | Yes, w/cap. on AREF  |
| `INTERNAL2V2_CAP`  | Internal 2.2V reference     | Yes, w/cap. on AREF  |
| `INTERNAL4V096_CAP`| Internal 4.096V reference   | Yes, w/cap. on AREF  |

Accuracy of voltage reference spec'ed as +/- 3% over rated temperature and voltage range for 1.1V and 2.2V references, and +/- 4%for the 4.096V

#### Synonyms for reference options
Due to the long and storied history of ATtinyCore, there are a large number of synonyms or aliases of these to ensure that old code can still be compiled.
| Reference Name         | Alias of                | Remarks                        |
|------------------------|-------------------------|--------------------------------|
| `INTERNAL1V1_NO_CAP`   | `INTERNAL1V1`           | Clear, but verbose             |
| `INTERNAL2V2_NO_CAP`   | `INTERNAL2V2`           | Clear, but verbose             |
| `INTERNAL4V096_NO_CAP` | `INTERNAL4V096`         | Clear, but verbose             |
| `INTERNAL4V1`          | `INTERNAL4V096`         | It's only accurate to +/- 4%   |
| `INTERNAL`             | `INTERNAL1V1`           | deprecated                     |
| `INTERNAL4V`           | `INTERNAL4V096`         | deprecated                     |
| `INTERNAL1V1_AREF`     | `INTERNAL1V1_CAP`       | deprecated                     |
| `INTERNAL2V2_AREF`     | `INTERNAL2V2_CAP`       | deprecated                     |
| `INTERNAL4V096_AREF`   | `INTERNAL4V096_CAP`     | deprecated                     |
| `INTERNAL1V1NOBP`      | `INTERNAL1V1`           | deprecated                     |
| `INTERNAL2V2NOBP`      | `INTERNAL2V2`           | deprecated                     |
| `INTERNAL4V096NOBP`    | `INTERNAL4V096`         | deprecated                     |


### Internal Sources
| Voltage Source  | Description                            |
|-----------------|----------------------------------------|
| ADC_INTERNAL1V1 | Reads the INTERNAL1V1 reference        |
| ADC_GROUND      | Reads ground                           |
| ADC_TEMPERATURE | Reads internal temperature sensor      |

### Differential ADC channels

| Pos. Chan | Pin | Neg. Chan | Pin |  Chan. Name | Channel |
|-----------|-----|-----------|-----|-------------|---------|
|      ADC0 | PA0 |      ADC1 | PA1 |  DIFF_A0_A1 |    0x10 |
|      ADC0 | PA0 |      ADC3 | PA3 |  DIFF_A0_A3 |    0x11 |
|      ADC1 | PA1 |      ADC2 | PA2 |  DIFF_A1_A2 |    0x12 |
|      ADC1 | PA1 |      ADC3 | PA0 |  DIFF_A1_A3 |    0x13 |
|      ADC2 | PA2 |      ADC3 | PA3 |  DIFF_A2_A3 |    0x14 |
|      ADC3 | PA3 |      ADC4 | PA4 |  DIFF_A3_A4 |    0x15 |
|      ADC3 | PA3 |      ADC5 | PA5 |  DIFF_A3_A5 |    0x16 |
|      ADC3 | PA3 |      ADC6 | PA6 |  DIFF_A3_A6 |    0x17 |
|      ADC3 | PA3 |      ADC7 | PA7 |  DIFF_A3_A7 |    0x18 |
|      ADC4 | PA4 |      ADC5 | PA5 |  DIFF_A4_A5 |    0x19 |
|      ADC4 | PA4 |      ADC6 | PA6 |  DIFF_A4_A6 |    0x1A |
|      ADC4 | PA4 |      ADC7 | PA7 |  DIFF_A4_A7 |    0x1B |
|      ADC5 | PA5 |      ADC6 | PA6 |  DIFF_A5_A6 |    0x1C |
|      ADC5 | PA5 |      ADC7 | PA7 |  DIFF_A5_A7 |    0x1D |
|      ADC6 | PA6 |      ADC7 | PA7 |  DIFF_A6_A7 |    0x1E |
|      ADC8 | PB2 |      ADC9 | PB3 |  DIFF_A8_A9 |    0x1F |
|      ADC0 | PA0 |      ADC0 | PA0 |  DIFF_A0_A0 |    0x20 |
|      ADC1 | PA1 |      ADC1 | PA1 |  DIFF_A1_A1 |    0x21 |
|      ADC2 | PA2 |      ADC2 | PA2 |  DIFF_A2_A2 |    0x22 |
|      ADC3 | PA3 |      ADC3 | PA3 |  DIFF_A3_A3 |    0x23 |
|      ADC4 | PA4 |      ADC4 | PA4 |  DIFF_A4_A4 |    0x24 |
|      ADC5 | PA5 |      ADC5 | PA5 |  DIFF_A5_A5 |    0x25 |
|      ADC6 | PA6 |      ADC6 | PA6 |  DIFF_A6_A6 |    0x26 |
|      ADC7 | PA7 |      ADC7 | PA7 |  DIFF_A7_A7 |    0x27 |
|      ADC8 | PB2 |      ADC8 | PB2 |  DIFF_A8_A8 |    0x28 |
|      ADC9 | PB3 |      ADC9 | PB3 |  DIFF_A9_A9 |    0x29 |
|     ADC10 | PB1 |      ADC8 | PB2 | DIFF_A10_A8 |    0x2A |
|     ADC10 | PB1 |      ADC9 | PB2 | DIFF_A10_A9 |    0x2B |
|     ADC11 | PB0 |      ADC8 | PB2 | DIFF_A11_A8 |    0x2C |
|     ADC11 | PB0 |      ADC9 | PB2 | DIFF_A11_A9 |    0x2D |
|      ADC1 | PA1 |      ADC0 | PA0 |  DIFF_A1_A0 |    0x30 |
|      ADC3 | PA3 |      ADC0 | PA0 |  DIFF_A3_A0 |    0x31 |
|      ADC2 | PA2 |      ADC1 | PA1 |  DIFF_A2_A1 |    0x32 |
|      ADC3 | PA3 |      ADC1 | PA1 |  DIFF_A3_A1 |    0x33 |
|      ADC3 | PA3 |      ADC2 | PA2 |  DIFF_A3_A2 |    0x34 |
|      ADC4 | PA4 |      ADC3 | PA3 |  DIFF_A4_A3 |    0x35 |
|      ADC5 | PA5 |      ADC3 | PA3 |  DIFF_A5_A3 |    0x36 |
|      ADC6 | PA6 |      ADC3 | PA3 |  DIFF_A6_A3 |    0x37 |
|      ADC7 | PA7 |      ADC3 | PA3 |  DIFF_A7_A3 |    0x38 |
|      ADC5 | PA5 |      ADC4 | PA4 |  DIFF_A5_A4 |    0x39 |
|      ADC6 | PA6 |      ADC4 | PA4 |  DIFF_A6_A4 |    0x3A |
|      ADC7 | PA7 |      ADC4 | PA4 |  DIFF_A7_A4 |    0x3B |
|      ADC6 | PA6 |      ADC5 | PA5 |  DIFF_A6_A5 |    0x3C |
|      ADC7 | PA7 |      ADC5 | PA5 |  DIFF_A7_A5 |    0x3D |
|      ADC7 | PA7 |      ADC6 | PA6 |  DIFF_A7_A6 |    0x3E |
|      ADC9 | PB3 |      ADC8 | PB2 |  DIFF_A9_A8 |    0x3F |


#### ADC Differential Pair Matrix
The t x41-series parts offer a superset of the tinyx4-series; in the below table,**bold** indicates that an option was not available on the ATtiny x4-series
|  N\P  |   0   |   1   |   2   |   3   |   4   |   5   |   6   |   7   |   8   |   9   |   10  |   11  |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|   0   |   X   |   X   |       |   X   |       |       |       |       |       |       |       |       |
|   1   |   X   | **X** |   X   |   X   |       |       |       |       |       |       |       |       |
|   2   |       |   X   | **X** |   X   |       |       |       |       |       |       |       |       |
|   3   |   X   |   X   |   X   |   X   |   X   |   X   |   X   |   X   |       |       |       |       |
|   4   |       |       |       |   X   | **X** |   X   | **X** | **X** |       |       |       |       |
|   5   |       |       |       |   X   |   X   | **X** |   X   | **X** |       |       |       |       |
|   6   |       |       |       |   X   | **X** |   X   | **X** |   X   |       |       |       |       |
|   7   |       |       |       |   X   | **X** | **X** |   X   |   X   |       |       |       |       |
|   8   |       |       |       |       |       |       |       |       | **X** | **X** | **X** | **X** |
|   9   |       |       |       |       |       |       |       |       | **X** | **X** | **X** | **X** |


### Temperature Measurement
To measure the temperature, select the 1.1v internal voltage reference, and `analogRead(ADC_TEMPERATURE)`; This value changes by approximately 1 LSB per degree C. This requires calibration on a per-chip basis to translate to an actual temperature, as the offset is not tightly controlled - take the measurement at a known temperature (we recommend 25C - though it should be close to the nominal operating temperature, since the closer to the single point calibration temperature the measured temperature is, the more accurate that calibration will be without doing a more complicated two-point calibration (which would also give an approximate value for the slope)) and store it in EEPROM (make sure that `EESAVE` fuse is set first, otherwise it will be lost when new code is uploaded via ISP) if programming via ISP, or at the end of the flash if programming via a bootloader (same area where oscillator tuning values are stored). See the section below for the recommended locations for these.

## Special I/O-pin related features
The ATtiny441/841 includes several unusual GPIO features that are found only in a small number of other AVR devices.

### Separate pull-up enable register
Like the ATtiny828 and ATtiny1634, the ATtiny441/841 provides a dedicated pull-up enable register (PUEx) for each port, rather than controlling pull-ups through PORTx when DDRx is configured for input. The Arduino digital I/O functions handle this transparently, but when using direct port manipulation, note that setting a bit in PORTx will not enable the pull-up. Instead, set the corresponding bit in `PUEA` or `PUEB`.

Once enabled, a pull-up remains active regardless of subsequent port configuration. Enabling a pull-up on a pin that is then configured as an output and driven low will cause it to continuously draw current, which the datasheet explicitly flags as not recommended. That said, for a bidirectional line that is idle-high, leaving the pull-up enabled and toggling DDR to drive the line low can be a legitimate signalling strategy.


### Enhanced sink capability on PA5 and PA7
PA5 and PA7 feature enhanced current sink capability compared to other GPIO pins, meaning their output voltage remains closer to ground when sinking larger currents. The standard absolute maximum rating of 40mA still applies, but under typical loads of 10-20mA these pins exhibit noticeably lower 
voltage drop than standard pins.

```c
PHDE=(1<<PHDEA0)|(1<<PHDEA1); //PHDEA0 controls PA5, PHDEA1 controls PA7.
```

This feature appears to be unique to the ATtiny441/841 and the closely related ATtiny828, both of which also share the distinction of being the only classic AVR devices that support remapping of timer PWM outputs to alternative pins.

### Break-Before-Make
The ATtiny441/841 supports a Break-Before-Make mode, configurable on a per-port basis via the `PORTCR` register. When enabled, any pin transitioning from input to output via a DDR bit change will be held in a tristated (high-impedance) state for one system clock cycle before the output driver is engaged. This feature is not used by the core. It appears to be intended for scenarios where output values are pre-loaded into PORTx and multiple pins are simultaneously switched from input to output via a single DDR write, though the specific application this was designed for is not immediately obvious.

```c
PORTCR=(1<<BBMA)|(1<<BBMB); //BBMA controls PORTA, BBMB controls PORTB.
```

## Tuning Constant Locations
The ATtiny441/841, owing to the incredible power of it's oscillator, can be run at many speeds from the internal oscillator with proper calibration. We support storage of 4 calibration values. The included tuner uses these 4 locations for OSCCAL tuning values. If tuning is enabled, the OSCCAL tuning locations are checked at startup if tuning is enabled.

**ISP programming (no bootloader)**: EESAVE fuse set, stored in EEPROM

| Tuning Constant         | Location EEPROM | Location Flash |
|-------------------------|-----------------|----------------|
| Temperature Offset      | E2END - 3       | FLASHEND - 7   |
| Temperature Slope       | E2END - 4       | FLASHEND - 6   |
| Tuned OSCCAL0 8 MHz/3V3 | E2END - 3       | FLASHEND - 5   |
| Tuned OSCCAL0 8 MHz/5V  | E2END - 2       | FLASHEND - 4   |
| Tuned OSCCAL0 12 MHz*   | E2END - 1       | FLASHEND - 3   |
| Tuned OSCCAL0 16 MHz*   | E2END - 0       | FLASHEND - 2   |

`*` Calibration at aprx. 5v is assumed and implied
