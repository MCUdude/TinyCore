# Test Protocol: Analog write

# Test Protocol

1. Set up the host MCU board (Uno, Nano, or Mega) and target chip/board on a breadboard. 
2. Erase the target board, so that all I/O pins are high impedance.
3. Connect all host and target GPIOs by using a number 2 higher on the host: 0 -> 2, 1 -> 3, ... and from target pin 11 (host pin 13) onwards, add 3 (in order to leave out LED_BUILTIN). Actually, one only needs to connect the pins with PWM.
4. Provide power to the boards.
5. Flash target sketch.
6. Upload the host sketch and open terminal at 115200 baud.
7. Disconnect the programmer/debugger from the target!
8. Reset the host, then the target.

1. After a few seconds, the test run should start and end with "Analog write test successfully completed."

The host board will report success over the serial line and blink 1 sec on/1 sec off, or it will report failure and blink 0.1 sec on / 0.1 sec off.

The protocol is as follows:

1. Host pin 2 (connected to target pin 0) is set to INPUT_PULLUP. Now, the host waits for maximal 30 seconds for the target to start. 30 seconds is also the time-out later on, after which it will report failure.
2. The target will report the number of digital pins (without the RESET pin) to the target. It will send as many negative pulses as there are pins. 
3. Then the target board sends a pin number on which it will output a PWM signal.
4. The target sends 20 msec a 10% duty cycle signal, then 20 msec a 50% duty cycle signal, and finally 20 msec a 90% duty cycle signal.
5. If this were the last PWM pin, then it would send the number of I/O pins + 1; if there are more PWM pins, it continues with step 3.
6. If this were the last pin, the target would report success.

