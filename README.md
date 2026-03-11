# TinyCore

An Arduino core for classic ATtiny chips, with the option of running the [Urboot](https://github.com/stefanrueger/urboot) bootloader.  
Arduino IDE v1.8 and v2.x supported!

TinyCore is a fork of the [ATTinyCore 2.0.0 branch](https://github.com/SpenceKonde/ATTinyCore). It focuses on improving the overall user experience and other quality-of-life improvements. Compared to ATtinyCore, TinyCore has:  
* Simplified Arduino IDE Tools menu
* Rock-solid bootloader support using [Urboot](https://github.com/stefanrueger/urboot) in favour of Optiboot
* Hardware debugging support using [PyAvrOCD](https://pyavrocd.io)
* No Digispark (Micronucleus) support due to [USB timing costraints and the poor user experience caused by this](https://github.com/SpenceKonde/ATTinyCore?tab=readme-ov-file#vusb-is-not-supported-for-usb-functionality-within-the-sketch)
* Revised documentation

# Table of contents
* [Supported microcontrollers](#supported-microcontrollers)
* [Supported clock frequencies](#supported-clock-frequencies)
* [Bootloader](#bootloader)
* [BOD option](#bod-option)
* [EEPROM retain option](#eeprom-option)
* [Printf support](#printf-support)
* [Pin macros](#pin-macros)
* [Write to own flash](#write-to-own-flash)
* [Programmers](#programmers)
* **[How to install](#how-to-install)**
  - [Boards Manager Installation](#boards-manager-installation)
  - [Manual Installation](#manual-installation)
  - [Arduino CLI Installation](#arduino-cli-installation)
  - [PlatformIO](#platformio)
* **[Getting started with TinyCore](#getting-started-with-TinyCore)**
* [SPI, i2c and UART](#spi-i2c-and-uart)
  - [SPI](#spi)
  - [i2c](#i2c)
  - [Serial/UART](#serialuart)
* [ADC](#adc)


## Supported microcontrollers
Each chip family has different pinouts, peripherals, and capabilities. Some differences are obvious, while others are subtle and can affect how features behave. Please read the provided for the specific chip family you are using by clocking the links below. Here you'll find pin mappings, specs, and other important core-related details.

* [ATtiny25/45/85](/avr/extras/ATtiny_x5.md)
* [ATtiny24/44/84](/avr/extras/ATtiny_x4.md)
* [ATtiny441/841](/avr/extras/ATtiny_x41.md)
* [ATtiny261/461/861](/avr/extras/ATtiny_x61.md)
* [ATtiny87/167](/avr/extras/ATtiny_x7.md)
* [ATtiny48/88](/avr/extras/ATtiny_x8.md)
* [ATtiny2313/4313](/avr/extras/ATtiny_x313.md)
* [ATtiny1634](/avr/extras/ATtiny_1634.md)
* [ATtiny828](/avr/extras/ATtiny_828.md)
* [ATtiny43](/avr/extras/ATtiny_43.md)
* [ATtiny26](/avr/extras/ATtiny_26.md)

Can't decide what microcontroller to choose? Have a look at the specification table below:

| **ATtiny**        | 25/45/85     | 24/44/84     | 441/841  | 261/461/861  | 87/167  | 48/88    | 2313/4313 | 1634  | 828  | 43   | 26   |
|-------------------|--------------|--------------|----------|--------------|-------- |----------|-----------|-------|------|------|------|
| *Flash*           | 2/4/8kiB     | 2/4/8kiB     | 4/8kiB   | 2/4/8kiB     | 8/16kiB | 4/8kiB   | 2/4kiB    | 16kiB | 8kiB | 4kiB | 2kiB |
| *RAM*             | 128/256/512B | 128/256/512B | 256/512B | 128/256/512B | 512B    | 256/512B | 128/256B  | 1kiB  | 512B | 256B | 128B |
| *EEPROM*          | 128/256/512B | 128/256/512B | 256/512B | 128/256/512B | 512B    | 64B      | 128/256B  | 512B  | 256B | 64B  | 128B |
| *PWM pins*        | 3            | 4            | 6        | 3            | 3       | 2        | 4         | 4     | 4    | 4    | 2    |
| *HW serial ports* | 0            | 0            | 2        | 0            | 1       | 0        | 1         | 2     | 1    | 0    | 0    |

## Supported clock frequencies
TinyCore supports a variety of different clock frequencies. Select the microcontroller in the boards menu, then select the clock frequency. *You will have to hit "Burn bootloader" in order to set the correct fuses and upload the correct bootloader. This also has to be done if you want to change any of the fuse settings (BOD and EEPROM settings) regardless if a bootloader is installed or not. Make sure you connect an ISP programmer, and select the correct one in the "Programmers" menu.* 

For time-critical operations, an external crystal/oscillator is recommended. Urboot bootloader has automatic baud rate detection on devices that have a *hardware serial port*. You can change the default upload baud rate in the Tools menu. Note that not all baud rates will work with all clock frequency options, due to [UART baud rate error](https://wormfood.net/avrbaudcalc.php) being too high.

Note that the ATtiny48/88 and ATtiny828 require an external clock signal and are not able to drive a resonator circuit themselves. You may use a dedicated quartz crystal oscillator or a crystal driver.

| Frequency   | Oscillator type       | Urboot upload speed | Note                                              |
|-------------|-----------------------|---------------------|---------------------------------------------------|
| 16 MHz      | External crystal/osc. | 115200              |                                                   |
| 20 MHz      | External crystal/osc. | 115200              |                                                   |
| 18.4320 MHz | External crystal/osc. | 115200              |                                                   |
| 14.7456 MHz | External crystal/osc. | 115200              |                                                   |
| 12 MHz      | External crystal/osc. | 115200              |                                                   |
| 11.0592 MHz | External crystal/osc. | 115200              |                                                   |
| 9.216 MHz   | External crystal/osc. | 115200              |                                                   |
| 8 MHz       | External crystal/osc. | 38400               |                                                   |
| 7.3728 MHz  | External crystal/osc. | 115200              |                                                   |
| 6 MHz       | External crystal/osc. | 57600               |                                                   |
| 4 MHz       | External crystal/osc. | 57600               |                                                   |
| 3.6864 MHz  | External crystal/osc. | 115200              |                                                   |
| 2 MHz       | External crystal/osc. | 9600                |                                                   |
| 1 MHz       | External crystal/osc. | 9600                |                                                   |
| 16 MHz      | Internal osc. (PLL)   | 115200              | ATtiny25/45/85, 24/44/84, 261/461/861 and 26 only |
| 8 MHz       | Internal osc.         | 38400               | Might cause UART upload issues. See comment above |
| 4 MHz       | Internal osc.         | 9600                | Derived from the 8 MHz internal oscillator        |
| 1 MHz       | Internal osc.         | 9600                | Derived from the 8 MHz internal oscillator        |
| 128kHz      | Internal WDT osc.     | 1200                | 128kHz is not recommended for bootloader use      |


## Bootloader
TinyCore supports the ultra-lightweight and efficient [Urboot](https://github.com/stefanrueger/urboot) bootloader, written by [Stefan Rueger](https://github.com/stefanrueger). The bootloader makes it trivial to upload sketches using a USB-to-serial adapter, just like with a traditional AVR-based Arduino board. But unlike other bootloaders, Urboot only occupies 256 bytes of flash and protects its patched reset vector, which means that, unlike Optiboot, it's practically impossible to mess up the reset vector and "brick" the chip. Give the bootloader option a try, and you'll be amazed at how well it works!

The internal oscillator on the most ATtinys is usually slower than it should be according to the spec. Try burning the slower and faster ones (-1.25%, +1.25%, etc.) if the  "Bootloader: _Yes_" option doesn't work.

Note that the 128 kHz internal oscillator option is not recommended for use with a bootloader since the oscillator is too inaccurate for practical use with an asynchronous protocol like UART.


## BOD option
Brown-out detection, or BOD for short lets the microcontroller sense the input voltage and shut down if the voltage goes below the brown-out setting. To change the BOD settings you'll have to connect an ISP programmer and hit "Burn bootloader". Below is a table that shows the available BOD options:

| All other parts | ATtiny26 | ATtiny43 | 
|-----------------|----------|----------|
| 4.3V            | 4.3V     | 4.3V     |
| 2.7V            | 2.7V     | 2.7V     |
| 1.8V            | Disabled | 2.3V     | 
| Disabled        |          | 2.2V     |                               
|                 |          | 2.0V     |      
|                 |          | 1.9V     |    
|                 |          | 1.8V     | 
|                 |          | Disabled | 


## EEPROM option
If you want the EEPROM to be erased every time you burn the bootloader or upload using a programmer, you can turn off this option. You'll have to connect an ISP programmer and hit "Burn bootloader" to enable or disable "EEPROM retain". Note that when uploading using a bootloader, the EEPROM will always be retained.

If you're using an ISP programmer, data specified in the user program using the `EEMEM` attribute will be uploaded to EEPROM when you upload your program in the Arduino IDE. 

```cpp
#include <avr/eeprom.h>

volatile const char ee_data EEMEM = {"Data that's loaded straight into EEPROM\n"};

void setup() {
}

void loop() {
}
```


## Printf support
Unlike the official Arduino cores, TinyCore (and ATTinyCore for that matter) has printf support out of the box. If you're not familiar with printf, you should probably [read this first](https://www.tutorialspoint.com/c_standard_library/c_function_printf.htm). It's added to the Print class and will work with all libraries that inherit Print. Printf is a standard C function that lets you format text much easier than using Arduino's built-in print and println. Note that this implementation of printf will NOT print floats or doubles. This is disabled by default to save space, but can be enabled using a build flag if using PlatformIO.

If you're using a serial port, simply use `Serial.printf("Milliseconds since start: %ld\n", millis());`. You can also use the `F()` macro if you need to store the string in flash. Other libraries that inherit the Print class (and thus support printf) are the LiquidCrystal LCD library and the U8G2 graphical LCD library.


## Pin macros
Note that you don't have to use the digital pin numbers to refer to the pins. You can also use some predefined macros that map "Arduino pins" to the port and port number. This can result in code that's more portable across different chips and Arduino cores:

```c++
// Use PIN_PB0 macro to refer to pin PB0 (Arduino pin 0 on ATtiny25/45/85)
digitalWrite(PIN_PB0, HIGH);

// Results in the exact same compiled code
digitalWrite(0, HIGH);

```


## Write to own flash
TinyCore uses the excellent Urboot bootloader, written by [Stefan Rueger](https://github.com/stefanrueger). Urboot supports flash writing within the running application, meaning that content from e.g. a sensor can be stored in the flash memory directly without needing external memory. Flash memory is much faster than EEPROM, and can handle at least 10,000 write cycles before wear becomes an issue.
For more information on how it works and how you can use this in your own application, check out the [Serial_read_write](avr/libraries/Flash/examples/Serial_read_write/Serial_read_write.ino) for a simple proof-of-concept demo, and
[Flash_put_get](avr/libraries/Flash/examples/Flash_iterate/Flash_iterate.ino) + [Flash_iterate](/avr/libraries/Flash/examples/Flash_iterate/Flash_iterate.ino) for useful examples on how you can store strings, structs, and variables to flash and retrieve them afterward.


## Programmers
Select your microcontroller in the boards menu, then select the clock frequency. You'll have to hit "Burn bootloader" in order to set the correct fuses and upload the correct bootloader. <br/>
Make sure you connect an ISP programmer, and select the correct one in the "Programmers" menu. 


## How to install
#### Boards Manager Installation
This installation method requires Arduino IDE version 1.8.0 or greater.
* Open the Arduino IDE.
* Open the **File > Preferences** menu item.
* Enter the following URL in **Additional Boards Manager URLs**: `https://mcudude.github.io/TinyCore/package_MCUdude_TinyCore_index.json`
* Open the **Tools > Board > Boards Manager...** menu item.
* Wait for the platform indexes to finish downloading.
* Scroll down until you see the **TinyCore** entry and click on it.
* Click **Install**.
* After installation is complete, close the **Boards Manager** window.

#### Manual Installation
Click on the "Download ZIP" button in the upper right corner. Extract the ZIP file, and move the extracted folder to the location "**~/Documents/Arduino/hardware**". Create the "hardware" folder if it doesn't exist.
Open Arduino IDE, and a new category in the boards menu called "TinyCore" will show up.

Note that a manual installation will not download binary tools such as the most recent avrdude program and the debugging tool.

#### Arduino CLI Installation
Run the following command in a terminal:

```
arduino-cli core install TinyCore:avr --additional-urls https://mcudude.github.io/TinyCore/package_MCUdude_TinyCore_index.json
```

#### PlatformIO
[PlatformIO](http://platformio.org) is an open-source ecosystem for IoT and embedded systems. PlatformIO support is not ready just yet.


## Getting started with TinyCore
Ok, so you have downloaded and installed TinyCore, but how do you get the wheels spinning? Here's a quick start guide:
* Hook up your microcontroller as shown in the minimal setup schematic for the target you have selected.
* Open the **Tools > Board > TinyCore** menu item, and select the chip or chip family, e.g ATtiny25/45/85.
* Select your preferred BOD option. Read more about BOD [here](#bod-option).
* Select your preferred clock frequency. The **8 MHz internal oscillator** is the default setting.
* Select what kind of programmer you're using under the **Programmers** menu. Use one of the **slow** programmers if you're using the 128 kHz oscillator option, e.g., **USBasp slow**.
* Hit **Burn Bootloader** to set the fuses.
* Now that the correct fuse settings are set you can upload your code by using your programmer tool. Simply hit *Upload*, and the code will be uploaded to the microcontroller.


## SPI, i2c and UART
Most of these devices lack hardware support for interfaces such as SPI, i2c and/or UART (Serial), which are commonly available on ATmega devices. To minimize these differences, TinyCore provides modified versions of Wire.h and SPI.h that maintain the standard Arduino APIs while adapting their implementation to the available hardware on each chip.
As a result, code that includes Wire.h or SPI.h should generally work without modification. Because these interfaces are already handled internally, libraries such as USIWire, tinyWire, WireS, and similar alternatives are unnecessary and not supported.
For serial communication, devices without a hardware UART can use the SoftwareSerial library. However, SoftwareSerial relies on pin change interrupts (PCINTs), which prevents those interrupts from being used elsewhere. To avoid this limitation, TinyCore provides an alternative software serial implementation that uses the analog comparator interrupt instead. This allows PCINTs to remain available for other purposes. In this implementation, the RX pin is fixed, while the TX pin can be selected from a limited set of pins.
See the serial section below for additional details.  

<details>
<summary><b>Hardware communication interfaces table</b></summary>

| Part(s)               | SPI           | I2C Master  | I2C Slave | Serial (TX* , RX) |
|-----------------------|---------------|-------------|-----------|-------------------|
| ATtiny2313/4313       | USI           | USI         | USI       | 1x Hardware       |
| ATtiny43              | USI           | USI         | USI       | Software PA4, PA5 |
| ATtiny24/44/84        | USI           | USI         | USI       | Software PA1, PA2 |
| ATtiny25/45/85        | USI           | USI         | USI       | Software PB0, PA1 |
| ATtiny26              | USI           | USI         | USI       | Software PA6, PA7 |
| ATtiny261/461/861     | USI           | USI         | USI       | Software PA6, PA7 |
| ATtiny87/167          | Real SPI      | USI         | USI       | 1x Hardware (LIN) |
| ATtiny48/88           | Real SPI      | Real TWI    | Real TWI  | Software PD6, PD7 |
| ATtiny441/841         | Real SPI      | Software    | Slave TWI | 2x Hardware       |
| ATtiny1634            | USI           | USI         | Slave TWI | 2x Hardware **    |
| ATtiny828             | Real SPI      | Software    | Slave TWI | 1x Hardware       |

<b>*</b>  TX pin can be moved to any other pin on that port with Serial.setTxBit().  
<b>**</b> UART1 shares pins with the USI and slave TWI interface, which basically means you have to choose between USI (SPI or I2C master) or I2C slave, or a second serial port.

</details>

### SPI
On parts with hardware SPI, `SPI.h` behaves the same as on classic AVR devices. On USI-based parts, the interface differs slightly.

<details>
<summary><b>SPI support and differences on USI-based parts</b></summary>

USI uses **DI/DO** instead of **MISO/MOSI**. In master mode, **DI = MISO** and **DO = MOSI**. In slave mode, **DI = MOSI** and **DO = MISO**, though slave mode is not supported by `SPI.h`. The `MISO` and `MOSI` defines therefore assume master mode. For slave implementations with other libraries, `PIN_USI_DI`, `PIN_USI_DO`, and `PIN_USI_SCK` are provided. Do not confuse SPI pins used by sketches with the ISP programming pins, where the device operates as an SPI slave.

USI has no hardware clock generator, so clock dividers are implemented in software. Dividers **2, 4, 8, and ≥14** use separate routines. Passing a constant value to `SPISettings` or `setClockDivider` reduces code size; otherwise all routines and 32-bit math are included. Dividers ≥14 are approximate because the implementation is optimized for size.

Interrupts are not disabled during transfers. If an interrupt occurs during a byte, one clock bit may be stretched. This is usually harmless, but devices requiring consistent clock timing should wrap `transfer()` in `ATOMIC_BLOCK` or disable interrupts.

USI-based **SPI and i2c cannot be used simultaneously**, as they share the same hardware and pins.
</details>


### i2c
i2c support varies between devices. The ATtiny48 and ATtiny88 provide hardware i2c and behave like ATmega devices. As with SPI, the `Wire.h` library handles most differences, and code generally works without modification.

<details>
<summary><b>i2c support and limitations, USI, and slave-only parts</b></summary>

Most other devices implement i2c using USI. In these cases:

* **External pull-up resistors are required.** Unlike hardware TWI implementations, USI-based i2c cannot rely on internal pull-ups.
* The i2c master clock cannot be configured. The SCL frequency is fixed.

A few devices support hardware i2c slave mode only, with neither USI nor hardware TWI available for master operation. On these parts:

* i2c slave mode is supported through the included `Wire.h` library.
* Alternate or masked slave addresses can be configured via the `TWSAM` register. This register functions as on newer AVRs, but no wrapper API is provided.
* On the ATtiny828, the watchdog timer must be enabled for i2c operation due to a silicon erratum. Enabling the WDT in interrupt mode with an empty handler is sufficient.

Software i2c master implementations on these devices are unreliable. In particular, timeouts cannot be distinguished from slaves returning zero data, and clock configuration is not supported. Simultaneous master and slave operation is not supported on any of these devices.

##### Buffer size
Devices with more than 128 bytes of SRAM use a **32-byte buffer**. Smaller devices use **16 bytes**. However, most libraries assume a 32-byte buffer, so TinyCore uses a 32-byte buffer on larger devices even when this consumes a significant portion of available RAM.
</details>


### Serial/UART
All devices provide a `Serial` object. On parts with hardware UART, `Serial` behaves as a standard full-duplex AVR serial interface. Devices with two UARTs also provide `Serial1`. Most supported devices do not include hardware UART and instead use software serial.

<details>
<summary><b>Hardware and software serial</b></summary>

This core is compatible with the standard `SoftwareSerial` library, but that implementation uses all PCINT vectors. To avoid this, TinyCore provides a built-in software serial implementation that uses the analog comparator interrupt instead of PCINT. The RX pin is fixed to **AIN1**, while TX defaults to **AIN0** and can be moved to a limited set of pins.

Software serial can operate only one instance at a time. Transmission is always blocking: data is sent immediately rather than buffered as with hardware UART. Calls such as `Serial.print()` therefore return only after transmission completes.

```c
Serial.print("Hello World\n");
// On parts without hardware UART this is equivalent to:
Serial.print("Hello World\n");
Serial.flush();
```

##### Moving builtin soft-serial TX pin
On parts without hardware serial, the TX pin can be moved to another pin *on the same port* using `Serial.setTxBit(bit)`. The bit value must be between 0 and 7 and corresponds to the bit position within the port. This must be called before `Serial.begin()`.

##### TX only soft serial
The built-in software serial implementation makes it possible to only enable the TX only. This can be done in the Tools menu, or adding `-DSOFT_TX_ONLY` as a build flag in PlatformIO. TX only will exclude everything except the transmit functionality. read() and peek() will always return -1, and available() will always return 0.

##### Internal oscillator and Serial
Reliable UART communication requires accurate clock timing. The internal oscillator on many classic ATtiny devices is calibrated to approximately ±10%, which may be insufficient for serial communication. Some devices operate correctly without tuning, but others require calibration using OSCCAL.
The ATtiny x41 family, ATtiny1634R, and ATTiny828R include an oscillator calibrated to ±2%, but only below 4 V. At higher voltages the oscillator frequency increases, which can cause UART timing errors depending on baud configuration. Clock menu options are provided to compensate when operating above 4 V.
Because of these limitations, applications that rely on serial communication are generally best served by using an external crystal, except on devices with tighter oscillator calibration.
</details>


## ADC
Unlike official Arduino cores, TinyCore doesn't let you refer to analog input pins with just their channel number (0 for instance), you'll either have to use the Ax macros (A0 for instance) or use the equivalent digital pin number instead.  
This means that for an ATtiny841, you can either use `analogRead(A10)`, `analogRead(PIN_PB1)` or `analogRead(9)`.

Some parts have additional ADC functionality like differential inputs, programmable gain, bipolar mode and noise reduction. See the table below and read the appropriate [TinyCore target spesific documentation](#supported-microcontrollers) on how to utilize this functionality.

<details>
<summary><b>TinyCore ADC functionality overview</b></summary>

| **ATtiny**           | 25/45/85 | 24/44/84 | 441/841 | 261/461/861 | 87/167 | 48/88 | 2313/4313 | 1634 | 828 | 43 | 26 |
|----------------------|----------|----------|---------|-------------|--------|-------|-----------|------|-----|----|----|
| *Has ADC*            | ✅       | ✅        | ✅      | ✅          | ✅      | ✅    | ❌        | ✅    | ✅  | ✅ | ✅  |
| *ADC inputs*         | 4        | 12       | 12      | 11          | 11     | 6/8   |           | 12   | 28  | 4  | 11 |
| *Diff. support*      | ✅       | ✅       | ✅       | ✅          | ✅      | ❌    |           | ❌    | ❌  | ❌ | ✅  |
| *Bipolar mode*       | ✅       | ✅       | ❌       | ✅          | ✅      | ❌    |           | ❌    | ❌  | ❌ | ✅  |
| *Prog. gain*         | ❌       | ❌       | ✅       | ✅          | ❌      | ❌    |           | ❌    | ❌  | ❌ | ❌  |
| *Noise reduct. mode* | ✅       | ✅       | ✅       | ✅          | ✅      | ✅    |           | ✅    | ✅  | ✅ | ✅  |


Note that the number of analog input pins includes the input that may be multiplexed with the RESET pin. The RESET pin has to be disabled to utilize this input.
</details>

### Differential ADC support
Some parts will additionally have one or two additional configuration functions related to the differential mode.

<details>
<summary>Read more about this configuration here</summary>

#### analogGain()
analogGain() is unique to the ATtiny841/441, which has too many differential channels and gain setting combinations to fit into a single byte. It is documented in the [ATtiny841/441 page](/avr/extras/ATtiny_x41.md) as it only applies to those devices. All others with programmable gain pass the gain setting as part of the constant.

#### setADCBipolarMode(bool bipolar)
Enables or disables bipolar mode for the differential ADC on supported devices. Bipolar mode (true) returns a signed value representing the voltage difference between the positive and negative inputs. For example, with a 1.0 V reference, a +0.25 V difference returns +128, while reversing the inputs returns −128.
Unipolar mode (false) returns only positive values, and is the default behaviour. If the negative input exceeds the positive input, the result is 0. This mode provides one extra bit of resolution when the signal polarity is known.

#### ADC noise reduction mode
High ADC gain (up to 100× on ATtiny441/841 and 32× on ATtiny261/461/861) increases sensitivity to noise. For accurate measurements, proper hardware layout is recommended. In particular, the AVCC pin should be filtered as described in the datasheet (typically using an inductor), rather than tied directly to VCC. To reduce internal noise during conversions, use ADC noise reduction mode, which temporarily puts the CPU to sleep while the ADC performs a conversion.
Use `analogRead_NR()` instead of `analogRead()` to enable this behavior.

Notes:
* The I/O clock stops during conversion (~65–130 µs), so timers pause and PWM output stops during that time.
* Pin change interrupts and the watchdog interrupt can wake the CPU early, which may affect reading quality.
* Do not disable global interrupts, as the ADC interrupt is required to wake the CPU.
* `analogRead_NR()` uses the ADC interrupt vector.
</details>



<br/>
<br/>
<br/>
<br/>
<br/>
<br/>

# TODO - remove, rewrite, reword, format the rest

<br/>
<br/>

## ATTinyCore 2.0.0 - lots of changes, some of them big, a few of them may cause breakage
I cobbled ATTinyCore together with far less experience than I have now (indeed, I'd barely covered the basics when I started trying to get a working ATtiny841 core). I like to think I have a much better idea of how a core should be designed now. But this meant some terrible decisions were made in the past. Decisions that we have been paying the price for ever since. I decided that the core should be advanced to a state where the bad decisions have been fixed, and everything that needs to be exposed on the parts is exposed in a consistent manner (too much was done incrementally, and not enough planning was done, ever). This core should not expect any significant new feature enhancements from here on out. The new feature development will be for megaTinyCore and DxCore, as those represent the future of the AVR architecture. Bug fixes will still be made.
The most significant changes are:
1. **analogRead() and channel/pin numbers** ATTinyCore followed what the core it was based on did, which was to use analog channel numbers, not digital pin numbers for analogRead(). Originally, the An-constants were #defined as the number itself. Later, to make them work with the digital IO functions, I changed to  `#define An (n | 0x80)` - digital functions could check for the high bit, and if present, strip it off and use the analogInputToDigitalPin macro to find the digital pin number. I never did the inverse with analogRead() because it would have broken code which used the raw analog channel numbers. This is just absurd in this day and age, where every other core allows you to analogRead() digital pin numbers and it just works. As of 2.0.0, analogRead() takes either a digital pin number, a constant of the form An (one per analog pin, shown on pinout chart! corresponds directly to analog channels), one of the ADC_CHANNEL constants listed in the part-specific documentations pages (these are things like `ADC_TEMPERATURE` and `ADC_INTERNAL1V1`), or - assuming it has a differential ADC - one of the differential channels. If you were generating an analog channel at runtime, you can pass a number through the ADC_CH() macro to get a number that will be recognized as an analog **channel** number (this is preferred to directly setting the high bit, since it makes clear that you're doing it to get that analog channel). Several people to whom I have spoken about this to expressed disbelief that that was how it worked and unanimously favored this this change.
2. **Differential channels!** Yeah - about half of the parts we support have them, and they can be useful for accurately measuring small differences in voltage. These vary per part - some are rather basic, while the t861 and t841 are very fancy - and are listed in full in the part-specific documentation. These are now fully supported. For parts that support both modes, there is also a way to select bipolar (-512 to 511) vs unipolar (0 to 1023) mode. Be warned that some parts only support one mode, and others only support the other, and some let you select it. *read the part specific docs for further information*.
3. All the analog reference sources are named consistently, old (deprecated) names for references are still supported, but not recommended. **potential breakage** If you used to refer to a reference with a raw number, instead of the name (ie, if you did analogReference(0) instead of analogReference(DEFAULT), this will be totally broken for values other than zero (and 0 doesn't have a consistent meaning). The ADC_REF() macro can be used to convert from the REFS bits to a reference constant *if you must - but you shouldn't be using a raw number to select the reference if you want to be able to move it to different parts* - The 1.1v internal reference `INTERNAL1V1` is 2 on some parts, and 0 on others... almost everything has the 1.1v reference and everything can use Vcc, but that's really where the similarities end. If you are writing code that users might want to make work on any other part, you need to use the names, not numbers. With parts having at most 8 options for the analog reference, and most having fewer than that, I do not expect that this is a particularly burdensome requirement. .
4. **Legacy PIN_xn constants gone** `PIN_An` constants have the standard meaning, ie, PIN_An is the digital pin that analog channel `An` is on. Previously some (but to my surprise and horror, not even all) parts had a set of `PIN_xn` constants defined that worked like the new `PIN_Pxn` ones do. I was reading pins_arduino.h from an official core a few weeks after implementing my `PIN_xn` constants for a bunch of parts and discovered that `PIN_An` was already in use meaning something different. Use `PIN_Pxn` where x is the letter of a port, and n is the number of a pin within that port.
5. `PIN_Pxn` constants are in for all supported parts to refer to pins by port and bit. This is the recommended way to refer to pins, as it frees you from the need to consider which pin mapping is in use. If you soldered the LED to pin PB2, PIN_PB2 is going to control the LED no matter which pin mapping you have selected.
6. For various historical reasons, some parts have up to 3 pin mappings. These are now named consistently, and listed and described in the part specific documentation - All parts have a recommended pin mapping, some of them have a second one for a specific VUSB board (digispark pro, MH-ET) with the pins numbered differently, and some of them have a "legacy" pin mapping with the pins in an order that makes less sense, and which makes converting between analog and digital pins harder (as in, if there is stuff determined at runtime, it uses more flash and is slower), but which has been widely used in the past and is what existing code may have been written for. A number of inconsistencies between these pin mappings (where information was missing from one or the other) have been fixed, and they are now formatted and commented consistently.
7. The ATtiny1634 and ATtiny861 are now supported for Micronucleus. Test and demonstration boards will be available from my Tindie store. New versions of bootloader for all existing Micronucleus boards. Users should use the bootloader upgrade functionality to ensure that they have the latest version of the bootloader, and that it has their desired entry conditions (on reset pin, power on reset, power-on with pin held down, reset/power on w/reset held high (in case of disabled reset, holding the reset pin high during power on will make it enter bootloader - takes advantage of the fact that reset PIN bit always reads 0 when reset is not disabled... Actually, one wonders if it would work if you did PORTx |= 1 << RESETBIT; then test if that bit is set - does disabling reset actually make registers not store the value? If so, that would be even better - no dependence on hardware, clear the bit if could set it and run app, otherwise run bootloader. )

## Main Documentation
The documentation, broadly, falls into two categories. The General Documentation applies to all parts.

Additionally, there are parameters that are specific to one or several families of parts. These cover the specifics of the peripherals on the parts (how Serial, I2C, and SPI are implemented on that hardware), the PWM frequencies that will be used at specific system clock frequencies, any additional options relating to the ADC such as differential inputs, and particularly notable or problematic errata, if any, applicable to this part. These are linked below.

You should **always review that part-specific documentation** before making any choice of parts. The time it takes to read them is far less than the time it takes to redesign with a different part when you're nearly done and then trip over some unexpected limitation... I try to catch the differences that might pose this hazard and describe them in these documents.

### [General Documentation](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/README.md) - This applies to all supported parts
### [Wiring and required external components](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Ref_Wiring.md)

### [Programming Guide](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Ref_Programming.md)
### [Migration Guide - moving to ATTinyCore from a different ATtiny board package, or to ATTinyCore 2.0.0 from an earlier version](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Ref_Migration.md)
### Current **strongly** recommended IDE version: 1.8.13


### Timers and PWM
All of the supported parts have hardware PWM (timer with output compare functionality) on at least one pin. See the part-specific documentation pages for a chart showing which pins have PWM. In addition to PWM, the on-chip timers are also used for millis() (and other timekeeping functions) and tone() - as well as by many libraries to achieve other functionality. Typically, a timer can only be used for one purpose at a time.

On all supported parts, timekeeping functions are on timer0. This means that reconfiguring timer0 by manipulating it's registers will break `millis()` and `delay()`; this is not recommended unless millis is disabled entirely.

On all parts except the tiny841/441 `tone()` is on timer1; on 841/441. tone() is on Timer2 to improve compatibility; with Tone moved onto timer2, the many libraries that use timer1 (Servo, TimerOne, and many others) can be used alongside `tone()` on the 841/441. Using `tone()` will prevent PWM from working on PWM pins controlled by Timer1 (Timer2 for 841/441), and manipulating it's registers will break `tone()`.

Most of the ATtiny parts only have two timers. The attiny841 has a third timer, timer2, which is an exact copy of the lovely 16-bit timer1, and completely different from the timer2 that most atmega devices have. Libraries designed to work with "Timer2" will not work on any of these parts, even the 841/441.


### Built-in tinyNeoPixel library

The standard NeoPixel (WS2812/etc) libraries do not support all the clock speeds that this core supports, and some of them only support certain ports. This core includes two libraries for this, both of which are tightly based on the Adafruit_NeoPixel library, tinyNeoPixel and tinyNeoPixel_Static - the latter has a few differences from the standard library (beyond supporting more clocks speeds and ports), in order to save flash. Prior to 2.0.0, a tools submemu was needed to select the port. This is no longer required (the adafruit code was written with zero tolerance for any divergances from ideal timing; allowing for tiny divergences at points where it doesn't matter was all it took to reimplememt this without the need for that submenu. This code is not fully tested at "odd" clock speeds, but definitely works at 8/10/12/16/20 MHz, and will probably work at other speeds, as long as they are 7.3728 MHz or higher. See the [tinyNeoPixel documentation](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/tinyNeoPixel.md) and included examples for more information.

### Additional configuration options
These are available from tools submenus

#### Pin Remapping (x61, 441, 841 only, new in 2.0.0)
The x61-series can use either PORTA or PORTB pins for the USI. This must be chosen at compile time - implementing swap() like the megaAVR parts have would impose excess overhead.

The x41-series has two options for USART0 and two options for SPI. They are chosen independently, and the tools menu hence contains four options. (it's better than two menus, right?)

## Memory Lock Bits, disabling Reset
ATTinyCore will never set lock bits, nor will it set fuses to disable ISP programming (it is intentionally not made available as an option, since after doing that an HV programmer is needed to further reprogram the chip, and inexperienced users would be at risk of bricking their chips this way). The usual workflow when these bits are in use is Set other fuses -> Upload -> Test -> manually set the lockbits and/or fuses. This can be done from the command line using AVRdude. To expedite the process, you can enable "Verbose Upload" in preferences, do "burn bootloader" (the board and/or programmer does not need to be present), scroll to the top of the output window - the first line is the avrdude command used to burn the bootloader, including the paths to all the relevant files. It can be used as a template for the command you execute to set the fuse/lock bits.

Disabling of reset is only an option for boards definitions with a bootloader which uses a sound flash-erase implementation (Optiboot presently does not, while the VUSB bootloaders which disable reset are in widespread use, seemingly without issue). We recommend against it in all cases. The 8 and 14 pin parts can be unbricked with a comparatively simple HVSP programmer (only 4-7 pins - 4 pins + reset for 8-pin, plus 3 more tied low on 14-pin). Everything with more pins needs an HVPP programmer, involving a wire connected to every pin or almost every pin on the chip. The sheer number of connections makes it unlikely that it could ever be unbricked in-system if the "system" is much more than a breakout board. HVPP is extremely exotic within the hobby community, such that I've never heard anyone talk about unbricking with HVPP.

**USE EXTREME CAUTION WHEN USING THE USB UPDATE FOR MICRONUCLEUS** as you can update to a version of the bootloader that will not support your board.


## Pin Mappings


### ATtiny441/841
![x41 pin mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x41.jpg "Arduino Pin Mapping for ATtiny841/441")

### ATtiny1634
![1634 pin mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_1634.jpg "Arduino Pin Mapping for ATtiny1634")

### ATtiny828
![828 Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_828.jpg "Arduino Pin Mapping for ATtiny828")

### ATtiny25/45/85
![x5 pin mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x5.jpg "Arduino Pin Mapping for ATtiny85/45/25")

### ATtiny24/44/84
![x4 Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x4.jpg "Arduino Pin Mapping for ATtiny84/44/24")

### ATtiny261/461/861
![x61 Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x61.jpg "Arduino Pin Mapping for ATtiny861/461/261")

### ATtiny87/167
![x7 Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x7.jpg "Arduino Pin Mapping for ATtiny167/87")

### ATtiny48/88
![x8 SMD Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x8.jpg "Arduino Pin Mapping for ATtiny88/48 in TQFP")
![x8 DIP Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x8-PU.jpg "Arduino Pin Mapping for ATtiny88/48 in DIP")

### ATtiny2313/4313
![x313 Pin Mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_x313.jpg "Arduino Pin Mapping for ATtiny4313/2313")

### ATtiny43U
![ATtiny43U pin mapping](https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/Pinout_43.jpg "Arduino Pin Mapping for ATtiny43")


Note that two pin mappings are supported for some devices to retain backwards compatibility with other cores - the pin mapping may be chosen from a menu.

Note that analog pin numbers (ex A0 ) cannot be used with digitalWrite()/digitalRead()/analogWrite() - all pins have a digital pin number. Analog pin number should only be used for analogRead() - this represents a departure from the behavior used in the official AVR boards. This enables us to expose the advanced ADC functionality available on some of the ATtiny parts with minimal impact, as clearly written code is unlikely to fall afoul of this anyway.

## Caveats
* Some people have problems programming the 841 and 1634 with USBAsp and TinyISP - but this is not readily reproducible. ArduinoAsISP works reliably. In some cases, it has been found that connecting reset to ground while using the ISP programmer fixes things (particularly when using the USBAsp with eXtremeBurner AVR) - if doing this, you must release reset (at least momentarily) after each programming operation. This may be due to bugs in USBAsp firmware - See this thread on the Arduino forums for information on updated USBAsp firmware: [http://forum.arduino.cc/index.php?topic=363772](http://forum.arduino.cc/index.php?topic=363772) (Links to the new firmware are on pages 5-6 of that thread - the beginning is largely a discussion of the inadequacies of the existing firmware)
* At >4v, the speed of the internal oscillator on 828, 1634 and 841 parts increases significantly - enough that serial (and hence the bootloader) does not work. Significant enhancements have been made on this front in 1.4.0; reburning bootloader should sort it out. These are further improved in 2.0.0. Avoid using 115200 baud and 57600 baud if using the internal oscillator and running an 828, 1634, or x41 at 4V or higher - those speeds are over 2% off due to baud calculation error in the same direction that the clock speed is off.
* For that matter, don't use 115200 baud or 57600 baud on any classic AVR with a hardware serial port at 8/16 MHz, especially if they or the thing they are communicating with is using an internal oscillator. 115200 MHz is a failure prone baud rate at 16 MHz, as 57600 is at 8 MHz as well. Those speeds should be used only with a "USART crystal" as clock source, or a 12 MHz or 20 MHz clock source. For 8 and 16 MHz with hardware serial, 38400, 76800, and (at 16 MHz) 153600 get much better baud rate accuracy.
* There is a right and a wrong way to perform a software reset.
  * Unless you are using Optiboot and wish to reset *and* have the bootloader run, do not reset from software via `__asm__ __volatile__ (jmp 0)` - that performs a "dirty reset". Instead, enable the watchdog timer, set to reset the device on timeout, and then enter an infinite loop and wait for the reset 16ms later.
  * Do not attempt to generate a software reset by connecting an I/O pin to reset and driving it low; this is specifically warned about in the datasheet.
  * If using the WDT reset on an Optiboot board, no additional actions are necessary; The bootloader will see that the reset cause was the WDT, assume that it was the thing that generated the reset, turn off the WDT and start the application.
  * If using the WDT reset on a non-optiboot board definition, you must turn it off at the very start of setup() - the chip will reset with the watchdog still running at the minimum timeout.
  * Dirty resets are a Bad Thing. You want to do anything you can to avoid having to generate one, and to avoid accidentally generating one (a dirty reset is the normal result of smashing the stack or a number of other common programming errors - this is responsible most hangs caused by software bugs)
  * The only time that a dirty reset is acceptable (it is never recommended) is when a bootloader is in use and you need to make the bootloader run from within the app. **if you do this you MUST ensure the following**
    * (Optiboot) Timer1 and the UART (if any) used by the bootloader has been reset to the POR settings (all registers associated with peripheral set to 0).
    * Interrupts must be disabled.
    * If the chip is running at a prescaled or tuned clock speed, you must turn off the prescaling (unless chip was bootloaded with 1 MHz optiboot, which is used for 2 and 4 MHz internal on most parts), in which case it must be reset to divide by 8).
    * You must point the stack pointer to the end of the flash (`SP = RAMEND - 1`).
    * Failure to observe these precautions will likely result in the bootloader failing to function properly, and possibly leave you in a state where a physical reset button press is required!
    * Whatever you do, don't try to bring this trick with you onto a modern AVR - there's no reason to do that over there (they finally have a software reset function) and the unintended dirty resets can be far more dangerous than they are here (a necessary tradeoff to get some very nice interrupt-related features). On my modern AVR cores (megaTinyCore and DxCore) I actually detect that a dirty reset has happened at the very early stages of the boot process, and fire software reset automatically.
    * Seriously, don't do this unless you really have your back to the wall and there's just no other way.

