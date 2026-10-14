# Using Avrdude with TinyCore

[Avrdude](https://github.com/avrdudes/avrdude) is the command-line program that does all the actual programming work behind the Arduino IDE. Every time you hit *Upload*, *Upload Using Programmer* or *Burn Bootloader*, the IDE builds an Avrdude command and runs it. Avrdude talks to ISP programmers (USBasp, USBtinyISP, Arduino as ISP, Atmel-ICE, ...) as well as to the Urboot bootloader, and it can read and write flash, EEPROM, fuses and lock bits.

Avrdude can also **generate Urboot bootloaders on the fly**. This is why TinyCore ships no Urboot hex files at all: the bootloader is built by Avrdude at the moment you burn it.

Running Avrdude yourself is useful when you want to set fuses that aren't exposed in the Tools menu, read back what is actually on the chip, or build a bootloader with a different pin, baud rate or feature set.

This document is only a brief overview of some of Avrdude's functionality, focused on what's most relevant for TinyCore users. Avrdude can do a lot more than what's covered here. For complete information see the official [Avrdude documentation](https://avrdudes.github.io/avrdude/).

> **Tip:** Enable *File > Preferences > Show verbose output during: Upload*. The IDE will then print the exact Avrdude command it runs.


## Where is Avrdude located?
When TinyCore is installed through the Boards Manager, Avrdude is downloaded alongside it:

| OS      | Avrdude executable                                                                   |
|---------|--------------------------------------------------------------------------------------|
| Windows | `%LOCALAPPDATA%\Arduino15\packages\TinyCore\tools\avrdude\[version]\bin\avrdude.exe` |
| macOS   | `~/Library/Arduino15/packages/TinyCore/tools/avrdude/[version]/bin/avrdude`          |
| Linux   | `~/.arduino15/packages/TinyCore/tools/avrdude/[version]/bin/avrdude`                 |

The configuration file is bundled with Avrdude, and is located next to it, in `../etc/avrdude.conf`.

The examples below assume you are in the `bin` directory, or have added it to your `PATH`. On Windows, replace `avrdude` with `avrdude.exe`, and replace USB serial ports like `/dev/ttyUSB0` with `COM3` or similar.

The most important options are:

| Option | Meaning                                                                                |
|--------|----------------------------------------------------------------------------------------|
| `-c`   | Programmer type, e.g. `usbasp`, `usbtiny`, `arduino_as_isp`, `atmelice_isp`, `urclock` |
| `-p`   | Target part, e.g. `attiny85`, `attiny841`, `attiny167`                                 |
| `-P`   | Port, only needed for serial programmers and bootloaders                               |
| `-b`   | Baud rate for serial programmers and bootloaders                                       |
| `-U`   | Memory operation: `memory:r/w/v:value_or_file[:format]`                                |
| `-t`   | Start the interactive terminal                                                         |
| `-T`   | Run a single terminal command non-interactively                                        |

All examples below use an ATtiny85 and a USBasp programmer. Replace `-c` and `-p` to match your own setup. Add `-P` and `-b` if you're using a programmer that communicates with Avrdude over UART (arduino_as_isp for instance).


## Writing fuses
Fuses can be written directly as hex (or binary) values. The `:m` suffix means "immediate value".
This example sets an ATtiny85 to 8 MHz internal oscillator, BOD at 2.7 V and EEPROM retained:

```sh
avrdude -c usbasp -p attiny85 -U lfuse:w:0xe2:m -U hfuse:w:0xd5:m -U efuse:w:0xff:m
```

The fuse values used by TinyCore can all be found in [boards.txt](avr/boards.txt). For instance, the ATtiny85 high fuse is defined as `0b1101{eesave_bit}{bod_bits}`, where "EEPROM retained" gives `0` and "BOD 2.7V" gives `101`, resulting in `0b11010101` = `0xd5`.

Instead of calculating fuse bytes by hand, you can set them by name using the terminal `config` command, passed with `-T`. This gives the same result as the example above:

```sh
avrdude -c usbasp -p attiny85 -T "config ckdiv8=by_1; config bodlevel=bod_2v7; config eesave=ee_preserved"
```

See [Fuses and configuration](#fuses-and-configuration) below for how to find the available names and values.

> **Warning:** Be careful with fuses that disable ISP programming. Setting `rstdisbl` (reset pin disabled) or `spien` (SPI programming disabled), or selecting an external clock source without a clock connected, will lock you out of ISP programming. Only high-voltage programming can recover the chip after this.


## The Avrdude terminal
Avrdude has a built-in terminal that lets you explore the chip interactively, one command at a time. Start it with `-t`:

```sh
avrdude -c usbasp -p attiny85 -t
```

```text
avrdude> help
```

`help` lists all commands, and `<command> -?` shows the syntax of a single command, e.g. `dump -?`. Type `quit` (or `q`) to exit.

> **Note:** Flash and EEPROM writes in the terminal are cached, and are only written to the chip when you type `flush` or `quit`. Type `abort` to throw away pending writes. Fuses and lock bits are written immediately.

> **Tip:** Use `-c dryrun` instead of a real programmer to try out commands without any hardware connected. Avrdude then simulates a factory-new chip.


### Running terminal commands with -T
Every terminal command can also be passed on the command line with `-T`. The command runs, Avrdude exits, and any cached writes are flushed. This is handy for scripts and one-off operations:

```sh
# Show the device signature
avrdude -c usbasp -p attiny85 -T sig

# Show the first 64 bytes of flash
avrdude -c usbasp -p attiny85 -T "dump flash 0 64"

# Write a string to the beginning of EEPROM
avrdude -c usbasp -p attiny85 -T 'write eeprom 0 "Hello"'

# Multiple commands, either as separate -T options or separated by semicolons
avrdude -c usbasp -p attiny85 -T "dump eeprom 0 16" -T "config bodlevel"
```


### Reading flash and EEPROM
`dump` (or its alias `read`) shows a memory section as a hex dump. Addresses and lengths can be decimal or hex, and **negative values count from the end of the memory**, which is very convenient for looking at the bootloader:

```text
avrdude> dump flash 0 32         # Dump 32 bytes, starting from address 0
0000  7f cf ff ff ff ff ff ff  ff ff ff ff ff ff ff ff  |................|
0010  ff ff ff ff ff ff ff ff  ff ff ff ff ff ff ff ff  |................|

avrdude> dump flash -256 256     # Dump the last 256 bytes, where Urboot lives
avrdude> dump flash -16 16       # Dump the last 16 bytes, which hold the Urboot version and feature info
1ff0  11 24 08 95 ff ff ff ff  ff ff 04 06 db cf 06 40  |.$.............@|

avrdude> dump flash 0x100 ...    # From 0x100 to the end of flash
avrdude> dump                    # Continue where the previous dump left off
avrdude> dump eeprom 0 64        # First 64 bytes of EEPROM
```

`disasm` turns flash contents back into AVR assembly, which is useful for checking what a sketch or bootloader actually does:

```text
avrdude> disasm flash -256 24
L1f00: 11 24        clr     r1
L1f02: 24 b6        in      r2, io.mcusr
L1f04: 14 be        out     io.mcusr, r1
...
```

To keep a copy of what's on the chip, use `save` for a single region or `backup` for complete memories:

```text
avrdude> save flash -256 256 bootloader.hex:i   # Only the bootloader, as Intel Hex
avrdude> backup all chip_backup.hex              # Flash, EEPROM, fuses and lock bits
```

A backup can later be written back with `restore all chip_backup.hex`.


### Fuses and configuration
`config` shows all fuse settings by name, which is far easier to read than raw fuse bytes. `config -f` additionally groups them by fuse byte and shows the raw values:

```text
avrdude> config
config sut_cksel=intrcosc_8mhz_6ck_14ck_64ms # 34
config ckout=co_disabled # 1
config ckdiv8=by_8 # 0
config bodlevel=bod_disabled # 7
config eesave=ee_erased # 1
config wdton=wdt_programmable # 1
config spien=isp_enabled # 0
config dwen=dw_off # 1
config rstdisbl=external_reset # 1
config selfprgen=spm_disabled # 1
config lb=no_lock # 3

avrdude> config bodlevel=       # List the valid values for a setting
# conf bodlevel=bod_4v3      # 4
# conf bodlevel=bod_2v7      # 5
# conf bodlevel=bod_1v8      # 6
config bodlevel=bod_disabled # 7 (factory)

avrdude> config bodlevel=bod_2v7  # Change a setting
avrdude> read hfuse               # Check the resulting fuse byte
```


## Generating the Urboot bootloader on the fly
Avrdude treats anything starting with `urboot:` as a virtual file. The text after the colon is an underscore-separated list of features, written in any order. When used with `-U`, Avrdude generates the bootloader, writes it to flash, *and* sets the fuses the _bootloader_ needs to work (such as `SELFPRGEN`).

This is what TinyCore runs when you click *Burn Bootloader* for an ATtiny85, 8 MHz internal oscillator, BOD 2.7 V, EEPROM retained, with the Urboot bootloader selected:

```sh
avrdude -c usbasp -p attiny85 -e -U lock:w:0xff:m -U efuse:w:0xfe:m -U hfuse:w:0xd5:m -U lfuse:w:0xe2:m \
  -U urboot:1s_pr_noled_swio_rxb1_txb0_38400baud_i8MHz -U lock:w:0xff:m
```

The bootloader string breaks down like this:

| Feature     | Meaning                                                                                    |
|-------------|--------------------------------------------------------------------------------------------|
| `1s`        | 1 second watchdog timeout before the sketch is started                                     |
| `pr`        | Reset vector protection, which makes it practically impossible to "brick" the bootloader   |
| `noled`     | No activity LED                                                                            |
| `swio`      | Software serial, since the ATtiny85 has no hardware UART                                   |
| `rxb1_txb0` | RX on PB1, TX on PB0                                                                       |
| `38400baud` | Upload baud rate                                                                           |
| `i8MHz`     | 8 MHz, `i` = internal oscillator. Use `x` for an external crystal/oscillator               |

On parts with a hardware UART, TinyCore uses `uart0` and `autobaud` instead of `swio`, pins and baud rate, for example `urboot:1s_pr_autobaud_noled_uart0` on the ATtiny167.

Instead of `i`, you can use a letter that tells Avrdude how much the internal oscillator deviates from its nominal frequency: `a` to `h` means 10 % to 1.25 % slow, and `j` to `q` means 1.25 % to 10 % fast, in 1.25 % steps. This is what the *Urboot (F_CPU -1.25%)*, *Urboot (F_CPU +1.25%)* etc. options in the Tools menu do. For example, `g8MHz` is for an oscillator running about 2.5 % slow.


### Browsing Urboot configurations in the terminal
The terminal is the best place to explore which bootloaders are possible before you commit to one. A few special features make `write flash urboot:...` *describe* a bootloader instead of writing it. Start a dry run so no chip is needed:

```sh
avrdude -c dryrun -p attiny85 -t
```

`help` lists every available feature:

```text
avrdude> write flash urboot:help
```

`list` shows all bootloader variants that match the features you've given so far, together with their size and the flash space they occupy. Starred entries are the most feature-rich ones for the space they use:

```text
avrdude> write flash urboot:swio_rxb1_txb0_38400baud_i8MHz_list
   Selection Size Use Vers Features  Type         Canonical file name
         _pr *250 256 u8.0 w---jPr-- vector/eerdy urboot_t85_1s_i8m0_38k4_swio_rxb1_txb0_no-led_pr.hex
      _ee_pr *320 320 u8.0 we--jPr-- vector/eerdy urboot_t85_1s_i8m0_38k4_swio_rxb1_txb0_no-led_ee_pr.hex
      _ce_pr  ...
```

Add features one at a time to narrow the list down. Here `ee` adds EEPROM read/write support to the bootloader:

```text
avrdude> write flash urboot:swio_rxb1_txb0_38400baud_i8MHz_list_ee
Selection Size Use Vers Features  Type         Canonical file name
      _pr *320 320 u8.0 we--jPr-- vector/eerdy urboot_t85_1s_i8m0_38k4_swio_rxb1_txb0_no-led_ee_pr.hex
   _ce_pr *378 384 u8.0 weU-jPr-c vector/eerdy urboot_t85_1s_i8m0_38k4_swio_rxb1_txb0_no-led_ee_ce_u1_pr.hex
   ...
```

Once you've found one you like, `show` displays a single bootloader, and `configs` shows which fuse settings it needs:

```text
avrdude> write flash urboot:swio_rxb1_txb0_38400baud_i8MHz_ee_pr_show
320 320 u8.0 we--jPr-- vector/eerdy urboot_t85_1s_i8m0_38k4_swio_rxb1_txb0_no-led_ee_pr.hex

avrdude> write flash urboot:swio_rxb1_txb0_38400baud_i8MHz_ee_pr_configs
config lb=no_lock
config rstdisbl=external_reset
config selfprgen=spm_enabled
```

Some other useful features:

| Feature              | Meaning                                                                      |
|----------------------|------------------------------------------------------------------------------|
| `best`               | Pick the smallest feature-rich variant (the first starred entry in the list) |
| `led+b2`             | Activity LED on PB2, active high (`led-b2` for active low)                   |
| `2s`                 | Other watchdog timeouts: `250ms`, `500ms`, `1s`, `2s`, `4s`, `8s`            |
| `save` / `save=x.hex`| Also save the bootloader as a hex file                                       |

Remove `show`, `list` or `configs` and the bootloader is actually written. Note that, unlike `-U`, writing a bootloader from the terminal does **not** set the fuses for you, so apply the settings that `configs` reported, e.g. `config selfprgen=spm_enabled`. On a real chip, it's usually simpler to burn the final choice with `-U`:

```sh
avrdude -c usbasp -p attiny85 -U urboot:1s_pr_ee_led+b2_swio_rxb1_txb0_38400baud_i8MHz
```

Keep in mind that extra features cost flash. The example above produces a 320-byte bootloader instead of the usual 256, while the Arduino IDE still assumes 256 bytes when checking sketch size. The IDE also always uploads at the bootloader baud rate for the selected clock frequency. If you change the baud rate, upload from the command line instead (see below).


## Uploading a sketch through Urboot
With the bootloader in place, sketches are uploaded over a USB to serial adapter using the `urclock` programmer. Use *Sketch > Export Compiled Binary* in the Arduino IDE to get the hex file:

```sh
avrdude -c urclock -p attiny85 -P /dev/ttyUSB0 -b 38400 -D -U flash:w:my_sketch.hex:i
```

`-D` skips the chip erase, since the bootloader erases pages as they are written.


## More Urboot options
The `urclock` programmer has a set of extended options, passed with `-x`, that go beyond what the Arduino IDE uses. List them with:

```sh
avrdude -c urclock -p attiny85 -x help
```

These let you query a chip that already runs Urboot, for instance its bootloader version and capabilities (`-x showversion`), bootloader size (`-x showboot`) or a summary of everything (`-x showall`). There are also settings that change how uploading works, such as `-x noautoreset` and `-x delay=<n>` for boards without an auto-reset circuit, or `-x eepromrw` for bootloaders built with EEPROM support:

```sh
avrdude -c urclock -p attiny85 -P /dev/ttyUSB0 -b 38400 -x showversion
```

Note that Urboot can store metadata such as the sketch file name and upload date in unused flash, which some of the `show` options rely on. TinyCore disables this with `-x nometadata` to keep all flash available for your sketch, so those particular options won't return anything useful unless you upload without it.

The full list of terminal commands, Urboot features and `urclock` options is documented in the [Avrdude manual](https://avrdudes.github.io/avrdude/8.3/avrdude.html).
