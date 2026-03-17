# Test Protocol

1. Set up the host MCU board (Uno, Nano, or Mega) and target chip/board on a breadboard. 
2. Erase the target board, so that all I/O pins are high impedance.
3. Connect all host and target GPIOs by using a number 2 higher on the host: 0 -> 2, 1 -> 3, ... and from target pin 11 (host pin 13) onwards, add 3 (in order to leave out LED_BUILTIN)
4. Provide power to the boards.
5. Flash target sketch.
6. Disconnect the programmer/debugger from the target!
7. Upload the host sketch and open terminal at 115200 baud.
8. Reset the host, then the target.
9. After a few seconds, the test run should start and end with "Digital read/write test successfully completed."

The host board will report success over the serial line and blink 1 sec on/1 sec off, or it will report failure and blink 0.1 sec on / 0.1 sec off.

The protocol is as follows:

1. Host pin 2 (connected to target pin 0) is set to INPUT_PULLUP. Now, the host waits for maximal 30 seconds, waiting for the target to start. 30 seconds is also the time-out later on, after which the host will reset the target (if possible) and report failure.
2. The target will report the number of digital pins (without the RESET pin) to the target. It will send as many negative pulses as there are pins. 
3. The target sketch puts all GPIOs into INPUT_PULLUP mode, which means all inputs of the host go HIGH.
4. The target checks whether some pins are LOW, meaning that there is a shortage. If so, it will terminate by switching all pins to INPUT, at which point the host will terminate with failure.
5. The target switches all lines to HIGH.
6. Then it will switch one pin after the other to LOW, starting at pin 0.
7. After all pins are LOW (for 100 ms), the target switches all pins to INPUT.
8. Now the host switches all pins to INPUT_PULLUP 
9. And waits for them to become HIGH.
10. Then the host switches pin after pin to LOW.
11. After 100 ms the host switches all pins again to input. 
12. If the target switches the pin with the highest number to HIGH, everything is good; otherwise, something failed.