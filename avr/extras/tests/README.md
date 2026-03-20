# Minimum functionality tests

The tests in this folder each check one particular basic functionality. They are organized in folders, where each folder contains at least a target sketch, which needs to be flashed to the target chip, and perhaps a host sketch for an ATmega328P (Uno, Mega, or Nano). Further, there will be a generic description of how to wire up the two sketches and how to run the test protocol. 

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
| `digitalRead()`/`digitalWrite()`on all pins | 🟢        | 🟢        | ⚪️         | 🟢         | 🟢        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | 🟢        | 🟢        |
| `analogWrite()`on all supported pins        | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟢        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | 🟢        | 🔴        |
| `Serial.print()` and `Serial.read()`        | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟡        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | 🟢        | 🟢        |
| `analogRead()`on all supported pins         | 🟢        | ⚪️        | ⚪️         | 🟢         | 🟢        | ⚪️        | ⚫️          | ⚪️          | ⚪️         | 🟢        | 🟢        |
| SPI master                                  | 🔴        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | 🟢        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| SPI slave                                   | 🔴        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | 🟢        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Wire master                                 | 🟢        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Wire slave                                  | 🟢        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Neopixel library/libraries                  | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Servo library/libraries                     | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |

🟢 = Works
🔴 = Does not work
🟡 = Partially works
⚫️ = Not present
⚪️ = Untested

ATtiny26:

- analogWrite: PWM does not work on either PWM pin (9 and 11)

ATtinyX7:

- Serial: TX works, RX does not

ATtiny85:

- SPI master does not work
- SPI slave does not compile, because some SPI registers are missing. I guess, one needs a different script 