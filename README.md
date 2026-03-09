# TinyCore

An Arduino core for classic ATtiny chips, with the option of running the [Urboot](https://github.com/stefanrueger/urboot) bootloader.  
Arduino IDE v1.8 and v2.x supported!

TinyCore is a fork of the [ATTinyCore 2.0.0 branch](https://github.com/SpenceKonde/ATTinyCore). It focuses on improving the overall user experience and other quality-of-life improvements. Compared to ATtinyCore, TinyCore has:  
* Less clutter and confusing options in the Arduino IDE Tools menu
* Rock-solid bootloader support using [Urboot](https://github.com/stefanrueger/urboot) in favour of Optiboot
* Hardware debugging support using [PyAvrOCD](https://pyavrocd.io)
* No Digispark (Micronucleus) support due to [USB timing costraints and the poor user experience caused by this](https://github.com/SpenceKonde/ATTinyCore?tab=readme-ov-file#vusb-is-not-supported-for-usb-functionality-within-the-sketch)
* A simplified README

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


## Supported microcontrollers
Each chip family has different pinouts, peripherals, and capabilities. Some differences are obvious, while others are subtle and can affect how features behave. Please read the provided for the specific chip family you are using by clocking the links below. Here you'll find pin mappings, specs, and other important core-related details.

* [ATtiny25/45/85](/avr/extras/ATtiny_x5.md)
* [ATtiny24/44/84](/avr/extras/ATtiny_x4.md)
* [ATtiny441/841](/avr/extras/ATtiny_x41.md)
* [ATtiny261/461/861](/avr/extras/ATtiny_x61.md)
* [ATtiny87/167](/avr/extras/ATtiny_x7.md)
* [ATtiny48/88](/avr/extras/ATtiny_x8.md)
* [ATtiny1634](/avr/extras/ATtiny_1634.md)
* [ATtiny828](/avr/extras/ATtiny_828.md)
* [ATtiny43](/avr/extras/ATtiny_43.md)
* [ATtiny26](/avr/extras/ATtiny_26.md)

Can't decide what microcontroller to choose? Have a look at the specification table below:

| **ATtiny**        | 25/45/85     | 24/44/84     | 441/841  | 261/461/861  | 87/167  | 48/88    | 1634  | 828  | 43   | 26   |
|-------------------|--------------|--------------|----------|--------------|-------- |----------|-------|------|------|------|
| *Flash*           | 2/4/8kiB     | 2/4/8kiB     | 4/8kiB   | 2/4/8kiB     | 8/16kiB | 4/8kiB   | 16kiB | 8kiB | 4kiB | 2kiB |
| *RAM*             | 128/256/512B | 128/256/512B | 256/512B | 128/256/512B | 512B    | 256/512B | 1kiB  | 512B | 256B | 128B |
| *EEPROM*          | 128/256/512B | 128/256/512B | 256/512B | 128/256/512B | 512B    | 64B      | 512B  | 256B | 64B  | 128B |
| *PWM pins*        | 3            | 4            | 6        | 3            | 3       | 2        | 4     | 4    | 4    | 2    |
| *HW serial ports* | 0            | 0            | 2        | 0            | 1       | 0        | 2     | 1    | 0    | 0    |

## Supported clock frequencies
TinyCore supports a variety of different clock frequencies. Select the microcontroller in the boards menu, then select the clock frequency. *You will have to hit "Burn bootloader" in order to set the correct fuses and upload the correct bootloader. This also has to be done if you want to change any of the fuse settings (BOD and EEPROM settings) regardless if a bootloader is installed or not*.

Make sure you connect an ISP programmer, and select the correct one in the "Programmers" menu. For time-critical operations, an external crystal/oscillator is recommended. Urboot bootloader has automatic baud rate detection on devices that has a *hardware serial port*. This lets you change the default upload baud rate in the Tools menu. Note that not all baud rates will work with all clock frequency options, due to [UART baud rate error](https://wormfood.net/avrbaudcalc.php) being too high.

Note that the ATtiny48/88 and ATtiny828 requires an external clock signal and is not able to drive a resonator circuit itself. You may use a dedicated quartz crystal oscillator or a crystal driver.

| Frequency   | Oscillator type             | Default Urboot upload speed | Comment                                                         |
|-------------|-----------------------------|-----------------------------|-----------------------------------------------------------------|
| 16 MHz      | External crystal/oscillator | 115200                      |                                                                 |
| 20 MHz      | External crystal/oscillator | 115200                      |                                                                 |
| 18.4320 MHz | External crystal/oscillator | 115200                      |                                                                 |
| 14.7456 MHz | External crystal/oscillator | 115200                      |                                                                 |
| 12 MHz      | External crystal/oscillator | 115200                      |                                                                 |
| 11.0592 MHz | External crystal/oscillator | 115200                      |                                                                 |
| 9.216 MHz   | External crystal/oscillator | 115200                      |                                                                 |
| 8 MHz       | External crystal/oscillator | 38400                       |                                                                 |
| 7.3728 MHz  | External crystal/oscillator | 115200                      |                                                                 |
| 6 MHz       | External crystal/oscillator | 57600                       |                                                                 |
| 4 MHz       | External crystal/oscillator | 57600                       |                                                                 |
| 3.6864 MHz  | External crystal/oscillator | 115200                      |                                                                 |
| 2 MHz       | External crystal/oscillator | 9600                        |                                                                 |
| 1 MHz       | External crystal/oscillator | 9600                        |                                                                 |
| 16 MHz      | Internal oscillator (PLL)   | 115200                      | Only the ATtiny25/45/85, 24/44/84, 261/461/861 and 26 has a PLL |
| 8 MHz       | Internal oscillator         | 38400                       | Might cause UART upload issues. See comment above               |
| 4 MHz       | Internal oscillator         | 9600                        | Derived from the 8 MHz internal oscillator                      |
| 1 MHz       | Internal oscillator         | 9600                        | Derived from the 8 MHz internal oscillator                      |
| 128kHz      | Internal WDT oscillator     | 1200                        | 128kHz is not recommended for bootloader use                    |


## Bootloader
TinyCore supports the ultra-light-weight and efficient [Urboot](https://github.com/stefanrueger/urboot) bootloader, written by [Stefan Rueger](https://github.com/stefanrueger). The bootloader makes it trivial to upload sketches using a USB-to-serial adapter, just like with a traditional AVR-based Arduino board. But unlike other bootloaders, Urboot only occupies 256 bytes of flash and protects its patched reset vector, which means that, unlike Optiboot, it's practically impossible to mess up the reset vector and "brick" the chip. Give the bootloader option a try, and you'll be amazed at how well it works!

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
If you want the EEPROM to be erased every time you burn the bootloader or upload using a programmer, you can turn off this option. You'll have to connect an ISP programmer and hit "Burn bootloader" to enable or disable EEPROM retain. Note that when uploading using a bootloader, the EEPROM will always be retained.

Note that if you're using an ISP programmer, data specified in the user program using the `EEMEM` attribute will be uploaded to EEPROM when you upload your program in Arduino IDE. 

```cpp
#include <avr/eeprom.h>

volatile const char ee_data EEMEM = {"Data that's loaded straight into EEPROM\n"};

void setup() {
}

void loop() {
}
```


## Printf support
Unlike the official Arduino cores, TinyCore (And ATTinyCore for that matter) has printf support out of the box. If you're not familiar with printf you should probably [read this first](https://www.tutorialspoint.com/c_standard_library/c_function_printf.htm). It's added to the Print class and will work with all libraries that inherit Print. Printf is a standard C function that lets you format text much easier than using Arduino's built-in print and println. Note that this implementation of printf will NOT print floats or doubles. This is disabled by default to save space but can be enabled using a build flag if using PlatformIO.

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
[Flash_put_get](avr/libraries/Flash/examples/Flash_iterate/Flash_iterate.ino) + [Flash_iterate](/avr/libraries/Flash/examples/Flash_iterate/Flash_iterate.ino) for useful examples on how you can store strings, structs, and variables to flash and retrieve then afterward.


## Programmers
Select your microcontroller in the boards menu, then select the clock frequency. You'll have to hit "Burn bootloader" in order to set the correct fuses and upload the correct bootloader. <br/>
Make sure you connect an ISP programmer, and select the correct one in the "Programmers" menu. For time-critical operations, an external oscillator is recommended.


## How to install
#### Boards Manager Installation
This installation method requires Arduino IDE version 1.8.0 or greater.
* Open the Arduino IDE.
* Open the **File > Preferences** menu item.
* Enter the following URL in **Additional Boards Manager URLs**: `https://mcudude.github.io/TinyCore/package_MCUdude_TinyCore_index.json`
* Open the **Tools > Board > Boards Manager...** menu item.
* Wait for the platform indexes to finish downloading.
* Scroll down until you see the **MegaCore** entry and click on it.
* Click **Install**.
* After installation is complete close the **Boards Manager** window.

#### Manual Installation
Click on the "Download ZIP" button in the upper right corner. Extract the ZIP file, and move the extracted folder to the location "**~/Documents/Arduino/hardware**". Create the "hardware" folder if it doesn't exist.
Open Arduino IDE, and a new category in the boards menu called "MegaCore" will show up.

#### Arduino CLI Installation
Run the following command in a terminal:

```
arduino-cli core install TinyCore:avr --additional-urls https://mcudude.github.io/TinyCore/package_MCUdude_TinyCore_index.json
```

#### PlatformIO
[PlatformIO](http://platformio.org) is an open-source ecosystem for IoT and embedded systems. PlatformIO support is not ready just yet.


## Getting started with TinyCore
Ok, so you have downloaded and installed MicroCore, but how do you get the wheels spinning? Here's a quick start guide:
* Hook up your microcontroller as shown in the minimal setup schematic for the target you have selected.
* Open the **Tools > Board > TinyCore** menu item, and select the chip or chip family, e.g ATtiny25/45/85.
* Select your preferred BOD option. Read more about BOD [here](#bod-option).
* Select your preferred clock frequency. The **8 MHz internal oscillator** is the default setting.
* Select what kind of programmer you're using under the **Programmers** menu. Use one of the **slow** programmers if you're using the 128 kHz oscillator option, e.g., **USBasp slow**.
* Hit **Burn Bootloader** to set the fuses.
* Now that the correct fuse settings are set you can upload your code by using your programmer tool. Simply hit *Upload*, and the code will be uploaded to the microcontroller.

# TODO

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



### I2C, SPI and Serial
Most of these parts do not have hardware support for I2C, SPI, and/or UART (Serial) like an ATmega device would. **As much as possible we try to paper over the differences - you can include Wire.h or SPI.h and expect things to just work except as noted below** - this is achieved by a special version of Wire.h and SPI.h which presents the same API, but implements it very differebtly depending on the underlying hardeware. Hence **the use of libraries like USIWire, tinyWire, WireS, and so on is unnecessary** These libraries are also considered unsupported, as they should never be necessary. In the case of Serial/UART, where there is no hardware serial, the SoftwareSerial library can be used, but it is often undesirable because of how it takes over all the PCINTs. To address this, we provide a different software serial implementation which uses the analog comparator interrupt instead of a PCINT, allowing the PCINTs to be used freely. The RX pin is fixed, but the TX pin can be moved around to a limited subset of pins. See the serial section below for more information. The following table shows what hardware interface is available on each of these part.


| Part(s)               | SPI           | I2C Master  | I2C Slave | Serial (TX* , RX) |
|-----------------------|---------------|-------------|-----------|-------------------|
| ATtiny x313           | USI           | USI         | USI       | 1x Hardware       |
| ATtiny 43             | USI           | USI         | USI       | Software PA4, PA5 |
| ATtiny x4             | USI           | USI         | USI       | Software PA1, PA2 |
| ATtiny x5             | USI           | USI         | USI       | Software PB0, PA1 |
| ATtiny 26             | USI           | USI         | USI       | Software PA6, PA7 |
| ATtiny x61            | USI           | USI         | USI       | Software PA6, **  |
| ATtiny x7             | Real SPI      | USI         | USI       | 1x Hardware (LIN) |
| ATtiny x8             | Real SPI      | Real TWI    | Real TWI  | Software PD6, PD7 |
| ATtiny x41            | Real SPI      | Software    | Slave TWI | 2x Hardware       |
| ATtiny1634            | USI           | USI         | Slave TWI | 2x Hardware ***   |
| ATtiny828             | Real SPI      | Software    | Slave TWI | 1x Hardware       |

`*` - TX pin can be moved to any other pin on that port with Serial.setTxBit().
`**` - RX can be on PA5, PA6, or PA7 (default), controlled by the tools submenu.
`***` - Serial1 shares pins with the USI and slave TWI interface, which basically means you have to choose between USI (SPI or I2C master) or I2C slave, or a second serial port.

There are some specific considerations relevant to each of these interfaces, detailed below.

#### SPI
Where real hardware SPI is available, SPI.h will behave identically to that on any classic AVR.

On USI parts, there are a few minor concerns, though most things will work without issue, and it should all be handled transparently via the SPI library.
* **USI does not have MISO/MOSI, it has DI/DO**
  * when operating in master mode, **DI is MISO, and DO is MOSI**.
  * When operating in slave mode, **DI is MOSI and DO is MISO**. Note that like all other versions of SPI.h, slave mode is not supported. You must use a different library for that.
  * The #defines for MISO and MOSI assume master mode, slave mode being unsupported). PIN_USI_DI, PIN_USI_DO, and PIN_USI_SCK are defined and can be used for operation in slave mode with some other library. Be careful to distinguish the MISO/MOSI/SCK pins marked as "Programming Pins" from the pins used for SPI from within the sketch - when programming a part via ISP, it is, after all, acting as a slave.
* The clock dividers are implemented in software (a clock generator is one of the many things that USI lacks). Clock dividers of 2, 4, 8 and >=14 are implemented as separate routines; **call `SPISettings` or `setClockDivider` with a constant value to use less program space**, otherwise, all routines will be included along with 32-bit math. Clock dividers larger than 14 are only approximate because the routine is optimized for size, not exactness.
* Interrupts are not disabled during data transfer. That means that an interrupt could fire in the middle of a byte, and one of the bits in that byte would be very long. This is usually fine. If it isn't, because you're working with devices that require consistent clocking, wrap calls to `transfer` in `ATOMIC_BLOCK` or disable interrupts in the normal ways.
* Be aware that USI-based I2C is not available when USI-based SPI is in use (this should be obvious, as they used the same pins).


#### I2C
The situation regarding I2C is more complicated; The ATtiny48 and ATtiny88 have real hardware I2C, which works like it does on ATmega devices. Like SPI.h, the Wire.h library will handle most of these differences, and most things will work the same way.

Most other devices must use the USI for I2C. In these cases:
* **You must have external pullup resistors installed** - unlike devices with a real hardware TWI port, the internal pullups cannot be used with USI-based I2C to make simple cases (short wires, small number of tolerant slave devices) work.
* The option to set the clock as I2C master does not work. The SCL clock speed is fixed.

A small number of devices have support for hardware slave I2C **but neither a USI nor hardware TWI for master operation**. On THOSE parts, some additional considerations apply:
* I2C slave works great through the included Wire.h library.
  * You can even do an alternate address or masked address (where several of the bits of the incoming address are ignored) by setting the `TWSAM` register. That register works the same way as it does on modern (post-2016) AVRs - however, no wrapper is provided around setting it, unlike the modern AVRs (ex, megaTinyCore and DxCore)
  * On the ATtiny828, you must have the watchdog timer enabled in order for the USI to work in I2C mode; one of the pins in afflicted with one of the nastiest silicon bugs in a pre-2016 tinyAVR, and when the WDT is not enabled (interrupt mode with an empty interrupt is fine),
* Software I2C Master, on the other hand is... a little flaky on these parts, most notably, it's not possible to tell whether a transaction timed out, or if the slave just responded with a bunch of 0's. There is no clock configuration functionality here either. If planning a new project, consider using a different device (might I recommend a [modern tinyAVR](https://github.com/SpenceKonde/megaTinyCore/)?) if I2C master mode is a big part of your application.

Regardless of the implementation, simultaneously acting as both a master and a slave is never supported here. The hardware doesn't support it like it does on modern AVRs.

##### Buffer size
On all parts with more than 128b of SRAM, the buffer size in 32 bytes. On smaller parts, it is 16 bytes, but I'm not sure you could make those work with the Wire library anyway due to flash size constraints so this may not be relevant. All official cores use 32b buffers, and it is for this reason that a 32b buffer is used even on parts where the pair of buffers leads to using a painfully large fraction of the RAM  - libraries implicitly depend on the buffer being at least 32b, often without the author even being aware of that fact.

#### Serial Support
To most of us, the Serial interface is the most important of the big three serial protocols. All parts, whether or not they have hardware serial, will have an object named `Serial` that provides serial interface functionality. Where there is hardware serial, the Serial object is a normal fully featured, full duplex serial port that works just like any other AVR. The lucky chips that have two serial ports will also have Serial1 defined.

Unfortunately, that's only 8 of the 21 parts supported by this core. The rest must use some form of software serial. While this core is fully compatible with the usual SoftwareSerial library, it comes with the usual disadvantages, most notably the fact that it grabs all the PCINT vectors for itself. To address that on the parts not blessed with hardware serial, we include a software serial implementation with a fixed (on the x61, there are three options, selected from a tools menu.) RX pin, and a TX pin with a limited number of options - but which leaves the PCINT vectors available. It uises the analog comparator interrupt, and requires that the RX pin be the AIN1 pin. TX defaults to the AIN0 pin.

Regardless of how you achieve the software serial, however, you can still only transmit or receive on a single software serial instance at a time (SoftwareSerial or the builtin tinySoftSerial). Transmit is always blocking - a call that writes via software serial will not return until the data is sent unlike hardware serial which puts it into a buffer to send in the background.

```c
// In other words
Serial.println("Hello World");
// on a part without hardware serial, which is hence using our builtin software implementation, is equivalent to:
Serial.println("Hello World");
Serial.flush();
```

##### Moving builtin soft-serial TX pin
The builtin software serial port ("Serial" on parts without hardware serial) can be moved to any pin *as long as it is on the same port*. Use the `Serial.setTxBit(bit)` method. The 'bit' argument passed must be a number between 0 and 7, corresponding to the number within the port of the desired TX pin; **this should be called before** `Serial.begin()`. This option is only available on parts with the software serial.

##### TX-only soft serial
Many users have asked for a way to disable the receiving functionality of the builtin soft-serial entirely. New in 2.0.0, you can choose TX only the tools -> Software Serial menu. This will exclude everything except the transmit functionality. read() and peek() will always return -1, and available() will always return 0.

##### Warning: Internal oscillator and Serial
Note that when using the internal oscillator or pll clock, you may need to tune the chip (using one of many tiny tuning sketches) and set OSCCAL to the value the tuner gives you on startup in order to make serial (software or hardware) work at all - the internal clock is only calibrated to +/- 10% in most cases, while serial communication requires it to be within just a few percent. However, in practice, a larger portion of parts work without tuning than would be expected from the spec. That said, for the ATtiny x4, x5, x8, and x61-family I have yet to encounter a chip that was not close enough for serial using the internal oscillator at 3.3-5v at room temperature - This is consistent with the Typical Characteristics section of the datasheet, which indicates that the oscillator is fairly stable w/respect to voltage, but highly dependent on temperature.

The ATtiny x41-family, 1634R, and 828R have an internal oscillator factory calibrated to +/- 2% - but only at operating voltage below 4v. Above 4v, the oscillator gets significantly faster, and is no longer good enough for UART communication if the UART baud rate calculation error is in the same direction - See [the AVR baud rate chart](https://docs.google.com/spreadsheets/d/1uzU_HqWEpK-wQUo4Q7FBZGtHOo_eY4P_BAzmVbKCj6Y/edit?usp=sharing). These parts have a clock speed menu option for the internal oscillator when Vcc > 4v and when it is <= 4V; when the >4V option is selected, we'll takea guess at what the calibration should be lowered to, which should be enough to get serial working.. The 1634 and 828 (non-R) are not as tightly calibrated (so they may need tuning even at 3.3v) and are a few cents less expensive, but suffer from the same problem at higher voltages. Due to these complexities, **it is recommended that those planning to use serial (except on a x41, 1634R or 828R at 2.5~3.3v, or with a Micronucleus bootloader) use an external crystal** until a tuning solution is available.

### ADC Support
ATTinyCore 2.0.0 introduces a major enhancement to the handling of analog and digital pin numbers: Now, in all the #defined constants that refer to an analog channel, the high bit is set. (ie, ADC channel 4, A4, is defined by a line `#define A4 (0x80 | 4));` (actually, we also define ADC_CH() macro as shorthand for the bitwise or with 0x80. This advantage of this that it makes it more obvious why we're doing this to the number; if you see (0x80 | 4) you'd be like "wtf is this for? what does 0x80 have to do with anything?", whereas if you hadn't read this, and you saw ADC_CH(4) - you might not know exactly what's going on, but just from the name you'd know it was something to do with an analog reading, maybe of channel 4). Because all the analog channel number defines are all distinct from things that aren't analog channel numbers, the core's analogRead and digitalRead functions can tell the two apart; digitalRead(A3) will now look up what digital pin analog channel 3 is on, and use digitalRead on that, while analogRead(7) will now go look up what analog channel is on digital pin 7, and use analogRead on that.

#### Differential ADC support
ATTinyCore 2.0.0 includes proper support for using the differential ADC - which on many parts is as good as or better than the differential ADC found on ATmega parts of the same era (and in fact, in some use cases, is better than the one on the new AVR Dx-series parts!). Differential ADCs of varying sophistication are available on the following families of parts: x5, x7, x4, x61, x41. Refer to the part specific documentation for the details on how to use it - generally there will be constants you pass in place of the pin number (listed in a table on the part specific documentation page).

Some parts will additionally have one or two additional configuration functions related to the differential mode:
##### analogGain()
analogGain() is unique to the tiny841/441, which has too many differential channels and gain setting combinations to fit into a single byte. It is documented in the [ATtiny841/441 page](./https://github.com/SpenceKonde/ATTinyCore/blob/v2.0.0-devThis-is-the-head-submit-PRs-against-this/avr/extras/ATtiny_x41.md). as it only applies to those devices. All others with programmable gain pass the gain setting as part of the constant.

##### setADCBipolarMode(bool bipolar)
This function is available on four families of parts including a total of 11 devices. These parts included an option to configure the differential ADC in either Unipolar mode or Bipolar mode. In bipolar mode, the value returned is signed. If reference was 1.0 V and you were looking at a pair of pins with voltages of 2.75 V and 2.5 V on them, your positive pin is 0.25 times the reference voltage above the negative pin and hence, the value returned would be 128. If the pins were reversed, it would be -128. In Unipolar mode, you gain an additional bit of precision.... if the positive pin is indeed larger. If it's not, you'll just get 0's back This is why some of the part have two options for a pair of pins, differing only in which one is positive vs negative - it's so you can swap the pins and use unipolar mode for higher accuracy. The argument is a boolean value indicating whether bipolar mode should be enabled (true = bipolar mode, false = unipolar mode. )

Not all parts support the switch between unipolar and bipolar mode. The x4, x5, x61. and x7 do. The x41 supports only bipolar mode, while the 26 supports only unipolar mode. And obviously, devices without any differential ADC don't support this function either. Calling it for a part that doesn't support it will generate a compile error saying as much.

##### ADC noise reduction mode
Now that we've talked about ADC gain, which can be up to 100x on the 841 or 32x on the 861, we need to recognize that if you want to be measuring signal, rather than noise, you will need to take more care than usual. Some of this is in hardware - the AVCC pin (if present) should be connected with the inductor like the datasheet recommends rather than just being tied to Vcc like everyone in Arduinoland does normally. You should also probably be using the ADC noise reduction mode. This puts the processor to sleep during the conversion to reduce noise generated internally during that sensitive time.

To use ADC noise reduction mode, simply use `analogRead_NR()` instead of `analogRead()`. There are really only a few caveats here:
* The I/O clock will be stopped. This means timers will lose time (13 ADC clocks per reading; the ADC clock is 100-200 kHz on these so that means between 65 and 130 us of time is lost by millis/micros per ADC reading) and PWM will stop.
* If interrupts are enabled, pin change interrupts and the WDT interrupt will wake the part prematurely. This may impact the quality of the readings. You may wish to disable these interrupts if they are in use.
* Do not disable interrupts globally though, because then the ADC interrupt won't be able to wake the part.
* This uses the ADC interrupt vector. Hence if at other points in time, you wanted to, say, use the ADC in free running mode, and run your own interrupt when a new result is ready, this is not compatible with `analogRead_NR()`. However analogRead_NR is isolated in it's own file, along with the empty ISR, so unless you're using `analogRead_NR()`, you can freely define that vector yourself.

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

