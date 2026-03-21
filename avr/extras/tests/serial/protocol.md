# Test protocol: Serial I/O

This test checks whether the basic serial I/O functionality is there. It outputs some text, and you can type in characters that are echoed.

1. Set up the target chip/board on a breadboard. 
2. Connect the serial lines to a USB/UART converter.
3. Adjust the compile constants in the sketch. Perhaps you have to tune the OSCCAL value (using the tune_osccal sketch).
4. Flash target sketch
5. Open a terminal application on your host and connect it with the serial I/O of the target. 