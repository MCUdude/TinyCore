# Minimum functionality tests

The tests in this folder each check one particular basic functionality. They are organized in folders, where each folder contains at least a target sketch, which needs to be flashed to the target chip, and perhaps a host sketch for an ATmega328P (Uno, Mega, or Nano). Further, there will be a generic description of how to wire up the two boards and how to run the test protocol. 

- `digitalrw`: Tests `digitalRead` and `digitalWrite` on all pins
- `analogw`: Tests `analogWrite` on all supported pins, one by one, trying out different values.
- `analogr`: Tests `analogRead` on all supported pins, one by one, trying 0V and Vcc.
- `analogv`: Tests `analogRead` on all supported pins in parallel, printing the measured voltage. 
- `serial`: Tests serial I/O.
- `wire`: Tests to check I2C master and slave capabilities.
- `spi`: Tests to check SPI master and slave capabilities.
- `neo`: Tests to check the neopixel functionality.
- `servo`: Tests to check servo functionality.



| Tests                                       | ATtinyX5 | ATtinyX4 | ATtinyX41 | ATtinyX61 | ATtinyX7 | ATtinyX8 | ATtinyX313 | ATtiny1634 | ATtiny828 | ATtiny43 | ATtiny26 |
| ------------------------------------------- | -------- | -------- | --------- | --------- | -------- | -------- | ---------- | ---------- | --------- | -------- | -------- |
| `digitalRead()`/`digitalWrite()`on all pins | 🟢        | 🟢        | 🟢         | 🟢         | 🟢        | 🟢        | 🟢          | 🟢          | 🟢         | 🟢        | 🟢        |
| `analogWrite()`on all supported pins        | 🟢        | 🟢        | 🟢         | 🟢         | 🟢        | 🟢        | 🟢          | 🟢          | ⚪️         | 🟢        | ⚪️        |
| `Serial.print()` and `Serial.read()`        | 🟢        | 🟢        | 🟢         | 🟢         | 🟢        | 🟢        | 🟢          | 🟢          | 🟢         | 🟢        | 🟢        |
| `analogRead()`on all supported pins         | 🟢        | 🟢        | 🟢         | 🟢         | 🟢        | 🟢        | ⚫️          | 🟢          | 🟢         | 🟢        | 🟢        |
| SPI master                                  | 🟢        | 🟢        | 🟢         | 🟢         | 🔴        | 🟢        | 🟢          | 🟢          | 🟢         | 🟢        | 🟢        |
| SPI slave                                   | ⚫️        | ⚫️        | ⚪️         | ⚫️         | 🟢        | 🟢        | ⚫️          | ⚫️          | 🟢         | ⚫️        | ⚫️        |
| Wire master                                 | 🟢        | 🟢        | 🟢         | 🟢         | 🟢        | 🟢        | 🟢          | 🟢          | 🔴         | 🟢        | 🔴        |
| Wire slave                                  | 🟢        | ⚪️        | ⚪️         | 🟢         | 🔴        | 🟢        | ⚪️          | ⚪️          | 🔴         | 🔴        | 🔴        |
| Neopixel library/libraries                  | 🟢        | 🟢        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🟢         | 🟢        | 🔴        |
| Servo library/libraries                     | ⚪️        | ⚪️        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🟢         | ⚫️        | 🔴        |
| `tone()` (using timer1)                     | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚫️        |

🟢 = Works
🔴 = Does not work
🟡 = Partially works
⚫️ = Not present / Not implemented
⚪️ = Untested

ATtiny26:

- analogWrite: PWM does not work on either PWM pin (9 and 11)
- Wire Master & Slave, Neopixel: Not enough memory for test sketch
- Servo: Compilation error

ATtiny167:

- TWI slave: Stops after a few interactions and is stuck in waiting for the serial output buffer to become empty (same behavior as in the 1.5.2 core)

- SPI master: Stops in the middle of the test. (may be related to a bug previously fixed. Needs re-testing)

ATtiny43U:

- Wire slave: Does not start
- SPI master: No transfer (in 1.5.2 core: compilation error)

ATtiny828:

- **Note**: When PD3 (D27) is used as an input (also for TWI/SPI slave clock), WDT has to be activated!

- Wire Master & Slave fail with a compilation error (pins need to be defined in pins_arduino.h)
- analogWrite: Does not work at all. This still holds for the new hardware setup. Under the 1.5.2 core it works for pins 20-22 (but not for 16). 