# PlatformIO

[PlatformIO](https://platformio.org) is an open-source ecosystem for embedded development.
It has a built-in library manager and is Arduino-compatible. It supports most operating systems; Windows, MacOS, Linux 32 and 64-bit, ARM, and X86.  
And best of all, it works with TinyCore!

* [What is PlatformIO?](http://docs.platformio.org/en/latest/what-is-platformio.html)
* [PlatformIO IDE](http://platformio.org/#!/platformio-ide)
* Getting started with [PlatformIO IDE](http://docs.platformio.org/en/latest/ide/atom.html#quick-start) or [PlatformIO command line interface](http://docs.platformio.org/en/latest/quickstart.html)
* [Advanced functionality](http://docs.platformio.org/en/latest/platforms/atmelavr.html)
* [Project Examples](http://docs.platformio.org/en/latest/platforms/atmelavr.html#examples)


## TinyCore + PlatformIO
TinyCore and PlatformIO work great together. You can do serial uploads using the Urboot bootloader and upload using a dedicated programmer. You can also let PlatformIO calculate the fuses and load the correct bootloader file, just like Arduino IDE does!

PlatformIO uses the information provided in platformio.ini to calculate what fuse bits and what bootloader file to load.  
Provide enough information and run the following commands:  

```ini
; Only set fuses
pio run -t fuses -e fuses_bootloader
; Set fuses and burn bootloader
pio run -t bootloader -e fuses_bootloader
; (where "fuses_bootloader" can be replaced with a different environment to match your build configuration)
```

You can find a platformio.ini template you can use when creating a project for a MiniCore-compatible device below.  
The most common functionality is available in this template. As you can see, the template is divided into multiple environments.  

* The default build environment is defined under *[platformio]*.
* All parameters common for all environments are defined under *[env]*.
* Use *[env:Upload_UART]* or *[env:Upload_ISP]* to upload to your target.
* Use *[env:fuses_noboot]* to set the fuses if you don't need a bootloader.
* Use *[env:fuses_urboot]* to set the fuses and flash the Urboot bootloader.

More information on what each line means can be found further down on this page.


## platformio.ini template

These templates are very similar, but I've created a few ones to make it easier to get started

<details>
<summary><b>ATtiny85, with and without software based serial bootloader</b></summary>

``` ini
; PlatformIO Project Configuration File for TinyCore
; https://github.com/MCUdude/TinyCore/
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed, and extra flags
;   Library options: dependencies, extra library storage
;   Advanced options: extra scripting
;
; Please visit the documentation for the other options
; https://github.com/MCUdude/TinyCore/blob/main/PlatformIO.md
; https://docs.platformio.org/page/projectconf.html


[platformio]
default_envs = Upload_ISP ; Default build target


; Common settings for all environments
[env]
platform = https://github.com/MCUdude/platform-atmelavr-mcudude.git
framework = arduino

; TARGET SETTINGS
; Chip in use
board = ATtiny85
; Clock frequency in [Hz]
board_build.f_cpu = 8000000L

; BUILD OPTIONS
build_unflags =
; Extra build flags
build_flags = 

; SERIAL MONITOR OPTIONS
; Serial monitor port defined in the Upload_UART environment
monitor_port = ${env:Upload_UART.upload_port}
; Serial monitor baud rate
monitor_speed = 9600


; Run the following command to upload with this environment
; pio run -e Upload_UART -t upload
[env:Upload_UART]
; Serial bootloader protocol
upload_protocol = urclock
; Serial upload port
upload_port =
; Set upload baudrate. Can be changed on the fly if using Urboot
board_upload.speed = ${env:fuses_urboot.board_bootloader.speed}


; Run the following command to upload with this environment
; pio run -e Upload_ISP -t upload
[env:Upload_ISP]
; Custom upload procedure
upload_protocol = custom
; Avrdude upload flags
upload_flags =
  -C$PROJECT_PACKAGES_DIR/tool-avrdude/avrdude.conf
  -p$BOARD_MCU
  -cusbasp
; Avrdude upload command
upload_command = avrdude $UPLOAD_FLAGS -U flash:w:$SOURCE:i


; Run the following command to set fuses
; pio run -e fuses_urboot -t fuses
; Run the following command to set fuses + burn bootloader
; pio run -e fuses_urboot -t bootloader
[env:fuses_urboot]
board_hardware.oscillator = internal ; Oscillator type
board_bootloader.type = urboot       ; urboot, optiboot or no_bootloader
board_bootloader.speed = 38400       ; Bootloader baud rate
board_hardware.f_cpu_error = 1.25    ; The internal oscillator is 1.25% too fast
board_hardware.bod = 2.7v            ; Set brown-out detection
board_hardware.eesave = yes          ; Preserve EEPROM when uploading using programmer
upload_protocol = usbasp             ; Use the USBasp as programmer
upload_flags =                       ; Divide the SPI clock by 8
  -B8

; Run the following command to set fuses
; pio run -e fuses_noboot -t fuses
[env:fuses_noboot]
board_hardware.oscillator = internal
board_hardware.bod = 2.7v
board_hardware.eesave = yes
upload_protocol = usbasp
upload_flags =
  -B8
```

</details>
  
</br>
<details>
<summary><b>ATtiny841, with and without hardware based serial bootloader</b></summary>

``` ini
; PlatformIO Project Configuration File for TinyCore
; https://github.com/MCUdude/TinyCore/
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed, and extra flags
;   Library options: dependencies, extra library storage
;   Advanced options: extra scripting
;
; Please visit the documentation for the other options
; https://github.com/MCUdude/TinyCore/blob/main/PlatformIO.md
; https://docs.platformio.org/page/projectconf.html


[platformio]
default_envs = Upload_ISP ; Default build target


; Common settings for all environments
[env]
platform = https://github.com/MCUdude/platform-atmelavr-mcudude.git
framework = arduino

; TARGET SETTINGS
; Chip in use
board = ATtiny841
; Clock frequency in [Hz]
board_build.f_cpu = 8000000L

; BUILD OPTIONS
build_unflags =
; Extra build flags
build_flags = 

; SERIAL MONITOR OPTIONS
; Serial monitor port defined in the Upload_UART environment
monitor_port = ${env:Upload_UART.upload_port}
; Serial monitor baud rate
monitor_speed = 9600


; Run the following command to upload with this environment
; pio run -e Upload_UART -t upload
[env:Upload_UART]
; Serial bootloader protocol
upload_protocol = urclock
; Serial upload port
upload_port =
; Set upload baudrate. Can be changed on the fly if using Urboot
board_upload.speed = 38400


; Run the following command to upload with this environment
; pio run -e Upload_ISP -t upload
[env:Upload_ISP]
; Custom upload procedure
upload_protocol = custom
; Avrdude upload flags
upload_flags =
  -C$PROJECT_PACKAGES_DIR/tool-avrdude/avrdude.conf
  -p$BOARD_MCU
  -cusbasp
; Avrdude upload command
upload_command = avrdude $UPLOAD_FLAGS -U flash:w:$SOURCE:i


; Run the following command to set fuses
; pio run -e fuses_urboot -t fuses
; Run the following command to set fuses + burn bootloader
; pio run -e fuses_urboot -t bootloader
[env:fuses_urboot]
board_hardware.oscillator = internal ; Oscillator type
board_bootloader.type = urboot       ; urboot, optiboot or no_bootloader
board_hardware.bod = 2.7v            ; Set brown-out detection
board_hardware.eesave = yes          ; Preserve EEPROM when uploading using programmer
upload_protocol = usbasp             ; Use the USBasp as programmer
upload_flags =                       ; Divide the SPI clock by 8
  -B8

; Run the following command to set fuses
; pio run -e fuses_noboot -t fuses
[env:fuses_noboot]
board_hardware.oscillator = internal
board_hardware.bod = 2.7v
board_hardware.eesave = yes
upload_protocol = usbasp
upload_flags =
  -B8
```

</details>


### `board`
PlatformIO requires the `board` parameter to be present. Simply use the chip name.


### `board_build.f_cpu`
Specifies the clock frequency in [Hz].
Used to determine what oscillator option to choose. A capital `L` has to be added to the end of the frequency number.
Below is a table with supported clocks. Defaults to 8 MHz if not specified.
**Note that some parts cannot drive an external crystal, and requires an externa clock instead

| Clock speed | Oscillator | board_build.f_cpu                 |
|-------------|------------|-----------------------------------|
| 16 MHz      | Internal   | `16000000L` (parts with int. PLL) |
| 8 MHz       | Internal   | `8000000L` (default)              |
| 4 MHz       | Internal   | `4000000L`                        |
| 1 MHz       | Internal   | `1000000L`                        |
| 20 MHz      | External   | `20000000L`                       |
| 18.432 MHz  | External   | `18432000L`                       |
| 16 MHz      | External   | `16000000L`                       |
| 14.7456 MHz | External   | `14745600L`                       |
| 12 MHz      | External   | `12000000L`                       |
| 11.0592 MHz | External   | `11059200L`                       |
| 9.216 MHz   | External   | `9216000L`                        |
| 8 MHz       | External   | `8000000L`                        |
| 7.3728  MHz | External   | `7372800L`                        |
| 6 MHz       | External   | `6000000L`                        |
| 4 MHz       | External   | `4000000L`                        |
| 3.6864  MHz | External   | `3686400L`                        |
| 1 MHz       | External   | `1000000L`                        |


### `board_hardware.oscillator`
Specifies to use the internal or an external oscillator or external clock.  
Internal oscillator only works with `board_build.f_cpu` values `16000000L` (parts with PLL only), `8000000L`, `4000000L` and `1000000L`.

| Oscillator option    |
|----------------------|
| `internal` (default) |
| `external`           |
| `external_clock`     |


### `board_bootloader.type`
Specifies the bootloader type to burn.

| Bootloader type           |
|---------------------------|
| `no_bootloader` (default) |
| `urboot`                  |


### `board_hardware.uart`
Specifies the hardware UART port used for serial upload. This only applies with parts that has a hardware serial port and wants to flash a bootloader. Use `no_bootloader` if you’re using a dedicated programmer, i.e. not using a bootloader for serial upload.

| Upload serial port option       |
|---------------------------------|
| `no_bootloader` (default)       |
| `uart0`                         |
| `uart1` (841/441 and 1634 only) |

### `board_hardware.f_cpu_error`

Compensate for the internal oscillator error. Valid values are -10 to +10 in 1.25 steps. Defaults to 0 (%) if not present, and only applies when board_hardware.oscillator = internal and for parts that does not have a hardware serial port, such as the ATtiny25/45/85.

### `board_hardware.bod`
Specifies the hardware brown-out detection. Use `disabled` to disable.

| All other parts  | ATtiny26         | ATtiny43U        |
|------------------|------------------|------------------|
| `4.3v`           | `4.0v`           | `4.3v`           |
| `2.7v` (default) | `2.7v` (default) | `2.7v` (default) |
| `1.8v`           |                  | `2.3v`           |
| `disabled`       | `disabled`       | `2.2v`           |
|                  |                  | `2.0v`           |
|                  |                  | `1.9v`           |
|                  |                  | `1.8v`           |
|                  |                  | `disabled`       |


### `board_hardware.eesave`
Specifies if the EEPROM memory should be retained when uploading using a programmer. Use `no` to disable.

| EEPROM retain   |
|-----------------|
| `yes` (default) |
| `no`            |


### `board_hardware.ckout`
Enable system clock output on the CKOUT pin (varies from chip family to chip family, see datasheet for more info).

| Clock output enable |
|---------------------|
| `yes`               |
| `no` (default)      |


### `board_upload.speed` / `board_bootloader.speed`
Specifies the upload baud rate. Valid baud rates for devices with **hardware serial ports** are shown in the table below. The Urboot bootloader has auto baud support on devices that utilize their hardware serial port, so the upload baud rate can be changed without re-flashing the bootloader.
  
**Note that if you're using a programmer that communicates with Avrdude with a serial port (Arduino as ISP, STK500, etc.) the `board_upload.speed` field will interfere with the programmer's baud rate.  
In this case, use `board_bootloader.speed` to set the bootloader baud rate, and `board_upload.speed` to set the baud rate for the programmer.**  

#### Baud rates for hardware serial bootloaders

|             | 1000000 | 500000 | 460800 | 250000 | 230400 | 115200 | 57600  | 38400  | 19200 | 9600   |
|-------------|---------|--------|--------|--------|--------|--------|--------|--------|-------|--------|
| `20000000L` |         |  X     |        |  X     |        |  **X** |        |        |  X    |        |
| `18432000L` |         |        |  X     |        |  X     |  **X** |  X     |  X     |  X    |  X     |
| `16000000L` |  X      |  X     |        |  X     |        |  **X** |        |  X     |  X    |  X     |
| `14745600L` |         |        |  X     |        |  X     |  **X** |  X     |  X     |  X    |  X     |
| `12000000L` |         |  X     |        |  X     |        |        |  **X** |        |  X    |  X     |
| `11059200L` |         |        |  X     |        |  X     |  X     |  **X** |  X     |  X    |  X     |
| `9216000L`  |         |        |        |        |  X     |  **X** |  X     |  X     |  X    |  X     |
| `8000000L`  |  X      |  X     |        |  X     |        |  X     |  X     |  **X** |  X    |  X     |
| `7372800L`  |         |        |  X     |        |  X     |  **X** |  X     |  X     |  X    |  X     |
| `6000000L`  |         |        |        |  X     |        |        |  **X** |  X     |  X    |  X     |
| `4000000L`  |         |  X     |        |        |        |        |        |        |  X    |  **X** |
| `3686400L`  |         |        |  X     |        |  X     |  **X** |  X     |  X     |  X    |  X     |
| `1000000L`  |         |        |        |        |        |        |        |        |       |  **X** |

Suggested baud rates for a particular clock speed are in **bold text**.

#### Recommended baud rates for software serial bootloaders

|             | 1000000 | 500000 | 460800 | 250000 | 230400 | 115200 | 57600  | 38400  | 19200 | 9600   |
|-------------|---------|--------|--------|--------|--------|--------|--------|--------|-------|--------|
| `20000000L` |         | X      |        | X      | X      | **X**  | X      | X      | X     |        |
| `18432000L` |         | X      | X      | X      | X      | **X**  | X      | X      | X     |        |
| `16000000L` |         | X      |        | X      | X      | **X**  | X      | X      | X     |        |
| `14745600L` |         |        | X      | X      | X      | **X**  | X      | X      | X     | X      |
| `12000000L` |         |        |        | X      | X      | **X**  | X      | X      | X     | X      |
| `11059200L` |         |        |        | X      | X      | **X**  | X      | X      | X     | X      |
| `9216000L`  |         |        |        | X      | X      | **X**  | X      | X      | X     | X      |
| `8000000L`  |         |        |        | X      |        | X      | X      | **X**  | X     | X      |
| `7372800L`  |         |        |        |        | X      | **X**  | X      | X      | X     | X      |
| `6000000L`  |         |        |        |        |        | X      | **X**  | X      | X     | X      |
| `4000000L`  |         |        |        |        |        |        | X      | X      | X     | **X**  |
| `3686400L`  |         |        |        |        |        | **X**  | X      | X      | X     | X      |
| `1000000L`  |         |        |        |        |        |        |        |        | X     | **X**  |


### `build_unflags`
This parameter is used to unflag.


### `build_flags`
This parameter is used to set compiler flags. This is useful if you want to, for instance, change the serial RX or TX buffer. Here's a list of the currently available core files flags:

| Flag                        | Description                                                        |
|-----------------------------|--------------------------------------------------------------------|
| -lprintf_flt                | Lets you print floats with printf (occupies ~1.5 kB)               |
| -Wall -Wextra               | Show all compiler warnings                                         |
| -DTX_ONLY                   | Only enable Serial TX (saves flash and RAM)                        |
| -DTXBUFFER                  | Enable 16-byte TX buffer (chips with hardware serial only)         |
| -DLOWER_CAL=6               | Used to slow down the internal oscillator on when running at 5V.<br/>Increase number to slow down more (ATtiny841/441/828/1634 only) |
| -DWIRE_MASTER_ONLY          | Enable i2c master only functionality (ATtiny841/441/828 only)      |
| -DWIRE_SLAVE_ONLY           | Enable i2c slave only functionality (ATtiny841/441/828 only)       |
| -DWIRE_BOTH                 | Enable i2c master and slave functionality (ATtiny841/441/828 only) |

**Example:**
`build_flags = -Wall -Wextra -DTX_ONLY -DLOWER_CAL=6 -DWIRE_MASTER_ONLY`


### `upload_port`
Specifies the serial port used for uploading. PlatformIO automatically detects the serial port. However, if you want to override this you can uncomment `upload_port`. Use `/dev/[port]` on Unix-compatible systems, and use `COMx` on Windows.


### `upload_protocol`
Used when using a programmer rather than using a USB to serial adapter.  
Supports all Avrdude compatible programmers such as `usbasp`, `usbtiny` and `stk500v1`.


### `upload_flags`
Used to pass extra flags to Avrdude when uploading using a programmer.  
Typical parameters are `-B[clock divider]` and `-b[baudrate]`.  
**Note that every flag has to be on its own line, and they have to be indented with two spaces:**
```ini
upload_flags =
  -B32
  -v
```


### `monitor_port`
PlatformIO detects serial ports automatically. However, In the template above it uses the upload port defined in the `env:Upload_UART` environment.
if you want to override this you can insert your upload port here. Use `/dev/[port]` on Unix-compatible systems, and use `COMx` on Windows.


### `monitor_speed`
Sets the serial monitor baud rate. Defaults to 9600 if not defined.


## User-defined fuses
Even though PlatformIO can calculate fuse values depending on the user-specified data in platformio.ini, there may be applications where the fuses have to be set to known values. This can be done like so:
```ini
; run the following command to set fuses
; pio run -e custom_fuses -t fuses
[env:custom_fuses]
; Inherit upload settings from the Upload_ISP environment
extends = env:Upload_ISP

; Fuse settings
board_fuses.lfuse = 0xe2
board_fuses.hfuse = 0xd5
board_fuses.efuse = 0xff
```