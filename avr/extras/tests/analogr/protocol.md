# Test Protocol: Analog read

This test only checks whether the basic functionality of `analogRead` is there. It will check whether LOW input leads to something close to zero and HIGH leads to something close to 1023.

1. Set up the host MCU board (Uno, Nano, or Mega) and target chip/board on a breadboard.
2. Erase the target board, so that all I/O pins are high impedance.
3. Connect all host and target GPIOs by using a number 2 higher on the host: 0 -> 2, 1 -> 3, ... and from target pin 11 (host pin 13) onwards, add 3 (in order to leave out LED_BUILTIN). Actually, one only needs to connect the pins with ADC input.
4. Provide power to the boards.
5. Flash target sketch.
6. Upload the host sketch and open terminal at 115200 baud.
7. Disconnect the programmer/debugger from the target!
8. Reset the host, then the target.
9. After a few seconds, the test run should start and end with "Analog write test successfully completed."

The host board will report success over the serial line and blink 1 sec on/1 sec off, or it will report failure and blink 0.1 sec on / 0.1 sec off.

The protocol is as follows:

1. Host pin 2 (connected to target pin 0) is set to INPUT_PULLUP. Now, the host waits for maximal 30 seconds for the target to start. 30 seconds is also the time-out later on, after which it will report failure.
2. The target will report the number of digital pins (without the RESET pin) to the target. It will send as many negative pulses as there are pins.
3. Then the target board sends a digital pin number on which it will expect an analog signal.
4. The host switches this pin to LOW, waits 50 msec, then it switches it to HIGH for 50 msec.
5. If this were the last ADC pin, then the target would send the number of I/O pins + 1; if there are more ADC pins, it continues with step 3.
6. If this were the last pin, the target would report success by pulling the host pin 2 to LOW. If there was an error, it will not pull the pin down.

