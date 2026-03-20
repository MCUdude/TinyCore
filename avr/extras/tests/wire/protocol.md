# Test protocol: I2C/TWI

This test checks whether the basic I2C functionality is there. It consists of a master sketch (`mwire.ino`) and a slave sketch (`swire.ino`).

1. Set up the target and the host (e.g., Uno or Nano) chips/boards on a breadboard. 
2. Connect the SDA and SCL lines. Insert also pull-up resistors.
3. Connect the serial line of one or both of the boards to the PC.
4. Flash the slave sketch to the target board and the master sketch to the host board.
5. Start the test by pressing reset on the board with the master sketch.
6. Success should be reported on both ends.
7. Repeat the test with master and slave roles exchanged.