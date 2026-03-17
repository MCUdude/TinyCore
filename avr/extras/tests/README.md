# Minimum functionality tests

The tests in this folder each check one particular basic functionality. They are organized in folders, where each folder contains at least a target sketch, which needs to be flashed to the target chip, and perhaps a host sketch for an ATmega328P (Uno or Nano). Further, there will be a generic description of how to wire up the two sketches and how to run the test protocol. In order to simplify things, the test sketches can be uploaded and started using a Python script.



| Tests                                       | ATtinyX5 | ATtinyX4 | ATtinyX41 | ATtinyX61 | ATtinyX7 | ATtinyX8 | ATtinyX313 | ATtiny1634 | ATtiny828 | ATtiny43 | ATtiny26 |
| ------------------------------------------- | -------- | -------- | --------- | --------- | -------- | -------- | ---------- | ---------- | --------- | -------- | -------- |
| `digitalRead()`/`digitalWrite()`on all pins | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| `analogWrite()`on all supported pins        | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| `Serial.print()` and `Serial.read()`        | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| `analogRead()`on all supported pins         | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| SPI master                                  | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| SPI slave                                   | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Wire master                                 | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Wire slave                                  | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Neopixel library/libraries                  | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |
| Servo library/libraries                     | ⚪️        | ⚪️        | ⚪️         | ⚪️         | ⚪️        | ⚪️        | ⚪️          | ⚪️          | ⚪️         | ⚪️        | ⚪️        |

🟢 = Works
🔴 = Does not work
🟡 = Partially works
⚫️ = Not implemented
⚪️ = Untested