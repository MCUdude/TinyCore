# ATtiny1634
![#1634 Pin Mapping](Pinout_1634.png "Arduino Pin Mapping for ATtiny 1634")

| Specification                    | ATtiny1634             |
|----------------------------------|------------------------|
| Bootloader (occupies 256 bytes)  | Urboot                 |
| Flash available user             | 16128 / 16384 bytes    |
| RAM                              | 1024 bytes             |
| EEPROM                           | 256 bytes              |
| GPIO Pins                        | 18 (incl RST)          |
| ADC Channels                     | 12                     |
| PWM Channels                     | 4                      |
| Interfaces                       | 2xUART, USI, i2c slave |
| Int. Oscillator (MHz)            | 8, 4, 2, 1             |
| Int. ULP Oscillator              | 128 kHz                |
| External Crystal (up to 16 MHz)  | All standard           |
| External Clock(up to 16 MHz)     | All standard           |
| LED_BUILTIN                      | PIN_PC0                |

### Urboot bootloader
This core uses the [Urboot bootloader](https://github.com/stefanrueger/urboot/) for the ATtiny441/841, a modern replacement that addresses the fundamental shortcomings of Optiboot on these parts. The bootloader is configured to occupy only 256 bytes, less than half of what Optiboot required, leaving 3840 or 7936 bytes available for user program. Urboot can be reconfigured to include additional features at the cost of increased flash usage, though the 256-byte variant used here covers the needs of most users. These chips does not have a hardware serial port, so Urboot is configured to use software-based UART.

A critical improvement over Optiboot is that Urboot actively protects both itself and the reset vector from being overwritten during flash operations, preventing the bootloader from bricking itself. The bootloader remains intact regardless of what is uploaded, making it a reliable choice.

No pre-compiled bootloader binaries are distributed with this core, instead, Avrdude generates the appropriate bootloader on the fly during upload.
UART0 or UART1 can be selected in the Arduino IDE Tools menu. The WDT timeout, UART pins, baud rate, and other bootloader parameters can be customized by editing the relevant entries in boards.txt or in your platformio.ini project configuration file.

The AVR internal oscillator is neither highly accurate nor necessarily tightly calibrated from the factory. Since a stable system clock is essential for asynchronous protocols such as UART, the bootloader has "autobaud functionality", which means that it will try to adjust and match the host baud rate.

## Features

### Internal Oscillator voltage dependence
Like the tiny828 and x41, While the calibration of the internal oscillator on the 1634 is very accurate between 2.7 and 4v, as the voltage rises above 4.5v, the speed increases significantly. Although the magnitude of this is larger than on many of the more common parts, the issue is not as severe as had long been thought - the impact is  magnified by the fact that the bootloader used 57600 baud at 8 MHz - which is off by more than 2% even assuming a perfect oscillator due to baud rate math. The fact that many USB ports actually supply 5.2-5.3V to help charge cellphones of course only makes it that much worse. in 1.5.0 a workaround was implemented to try to make matters better on this front, but he new baud rate changes in 2.0.0 should provide substantial improvements in bootloader reliability here (see the Optiboot reference linked above).

We do still provide a >4.5v clock option in order to improve behavior of the running sketch - it will nudge the oscillator calibration down to move it closer to the nominal 8MHz clock speed; sketches uploaded with the higher voltage option. This is not perfect, but it is generally good enough to work with Serial on around 5v (including 5.25v often found on USB ports to facilitate chargeing powerhungry devices), and millis()/micros() will keep better time than in previous versions.

The internal oscillator is factory calibrated to +/- 10% or +/- 2% for the slightly more expensive 1634R. +/- 2% is good enough for serial communication. However, this spec is only valid below 4v - above 4v, the oscillator runs significantly faster; enough so that serial communication does not work absent the above-described countermeasures.

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
The ATtiny1634 does not feature a dedicated I2C peripheral. Instead, I2C functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the Wire library included with this core. **External pull-up resistors are required on the SDA and SCL lines for I2C to function**.

Only the built-in Wire library is officially supported. Issues arising from the use of third-party I2C libraries should be directed to the respective library's author or maintainer, as compatibility with USI-based implementations cannot be guaranteed.

### SPI Support
The ATtiny1634 does not feature a dedicated SPI peripheral. Instead, SPI functionality is implemented through the hardware USI (Universal Serial Interface), exposed transparently via the included SPI library.
Note that the USI uses DI (Data In) and DO (Data Out) rather than the conventional MISO/MOSI naming. The mapping depends on the operating mode: in master mode, DI corresponds to MISO and DO to MOSI; in slave mode, these are reversed. The MISO and MOSI #defines reflect master mode, as this is by far the most common use case and the only mode supported by the SPI library.

Only the built-in SPI library is officially supported. If you encounter issues when using a third-party SPI library, please contact that library's author or maintainer, compatibility with USI-based hardware is their responsibility.

### UART (Serial) Support
There are two hardware serial ports, `Serial` and `Serial1`. It works the same as Serial on any normal Arduino - it is not a software implementation.

To use only TX or only RX channel, after Serial.begin(), one of the following commands will disable the TX or RX channels (for Serial1, use UCSR1B instead)
```c
UCSR0B &= ~(1<<TXEN0); // disable TX
UCSR0B &= ~(1<<RXEN0); // disable RX
```

### Tone Support
Tone() uses Timer1. For best results, use pin 2 or 14 (PA6, PB3), as this will use the hardware output compare to generate the square wave instead of using interrupts. Any use of tone() will disable PWM on pins PA6 and PB3.

### Servo Support
The standard Servo library is hardcoded to work on specific parts only, we include a builtin Servo library that supports the Tiny1634 series. As always, while a software serial port is receiving or transmitting, the servo signal will glitch. See [the Servo/Servo_TinyCore library](../libraries/Servo/README.md). Tone and Servo both require the same hardware resources are cannot be used at the same time.

### ADC Reference options
Note that **when using the Internal 1.1v reference, you must not apply an external voltage to AREF pin** - this sometimes appears to work, but other times results in erroneous ADC readings. Unlike some parts, there is no option to use the internal reference without the AREF pin being connected to it!

| Reference Option | Description               |
|------------------|---------------------------|
| `DEFAULT`        | Vcc                       |
| `INTERNAL1V1`    | Internal 1.1V reference   |
| `INTERNAL`       | Synonym for `INTERNAL1V1` |

### Workaround for PD3 Input Errata
If PD3 is needed as an input but the WDT is not otherwise required, the simplest solution is to keep the WDT running in interrupt mode with an empty ISR. The overhead is minimal: 10 bytes of flash, an ISR that executes in 11 clock cycles every 8 seconds, and an additional 1-4µA of current consumption, negligible during active operation. The real cost of this errata is therefore low, provided you are aware of it.
```c
// In setup():
CCP = 0xD8; // Write key to Configuration Change Protection register
WDTCSR = (1 << WDP3) | (1 << WDP0) | (1 << WDIE); // Enable WDT interrupt, 8-second prescale

// Outside all functions:
EMPTY_INTERRUPT(WDT_vect) // Empty ISR to keep ULP oscillator running. 
                           // EMPTY_INTERRUPT uses 26 bytes less than ISR(WDT_vect){;}
```

If sleep modes are used, the WDT must be disabled before sleeping, both to prevent spurious wake events and to avoid unnecessary current consumption. The helper function below adds only 12-16 bytes of overhead when called from a single location, or 20 bytes when called from two locations, with each subsequent call costing just 2 bytes.
```c
void startSleep() { // Call instead of sleep_cpu()
  CCP = 0xD8;       // Write key to Configuration Change Protection register
  WDTCSR = 0;       // Disable WDT interrupt
  sleep_cpu();
  CCP = 0xD8;       // Write key to Configuration Change Protection register
  WDTCSR = (1 << WDP3) | (1 << WDP0) | (1 << WDIE); // Re-enable WDT interrupt
}
```
