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
| `digitalRead()`/`digitalWrite()`on all pins | 🟢        | 🟢        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | ⚪️         | 🟢        | 🟢        |
| `analogWrite()`on all supported pins        | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🔴         | 🟢        | 🔴        |
| `Serial.print()` and `Serial.read()`        | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🟢         | 🟢        | 🟢        |
| `analogRead()`on all supported pins         | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚫️          | ⚪️          | 🟡         | 🟢        | 🟢        |
| SPI master                                  | 🔴        | ⚪️        | ⚪️         | 🔴         | 🔴        | 🟢        | ⚪️          | ⚪️          | 🟢         | 🔴        | 🔴        |
| SPI slave                                   | ⚫️        | ⚫️        | ⚪️         | ⚫️         | 🟢        | 🟢        | ⚫️          | ⚫️          | 🔴         | ⚫️        | ⚫️        |
| Wire master                                 | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🔴         | 🟢        | 🔴        |
| Wire slave                                  | 🟢        | ⚪️        | ⚪️         | 🟢         | 🔴        | 🟢        | ⚪️          | ⚪️          | 🔴         | 🔴        | 🔴        |
| Neopixel library/libraries                  | 🟢        | 🟢        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🟡         | 🟢        | 🔴        |
| Servo library/libraries                     | ⚪️        | ⚪️        | ⚪️         | 🟢         | 🟢        | 🟢        | ⚪️          | ⚪️          | 🟢         | 🔴        | 🔴        |

🟢 = Works
🔴 = Does not work
🟡 = Partially works
⚫️ = Not present / Not implemented
⚪️ = Untested

ATtiny26:

- analogWrite: PWM does not work on either PWM pin (9 and 11)
- Wire Master & Slave, SPI master, Neopixel: Not enough memory for test sketch
- Servo: Compilation error

ATtiny85:

- SPI master does not work

ATtiny167:

- TWI slave: Stops after a few interactions and is stuck in waiting for the serial output buffer to become empty (same behavior as in the 1.5.2 core)

- SPI master: Stops in the middle of the test. Perhaps a similar problem as above.

ATtiny861:

- SPI master: Message is not received by slave 

ATtiny43U:

- Servo: Compiler message: Not supported
- Wire slave: Does not start
- SPI master: No transfer (in 1.5.2 core: compilation error)

ATtiny828:

- Neopixel does not work on: 6,7,8,9,11, ...? Does not work on 1.5.2 either on pin 11, even if I set Neopixel port to PORTB.
- Wire Master & Salve use apparently my bitbang I2C lib SoftI2CMaster and fail with a compilation error
- Reading analog values: Wildly wrong and huge differences. Even when measuring individually, the values are way off (with the 1.5.2 core, it is the same; maybe it is a problem with the sample?)
- Does not work at all. Under the 1.5.2 core it works for pins 20-22 (but not for 16).