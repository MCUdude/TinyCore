# Test Protocol: Analog value test

This test checks how well the ADC measures an externally provided voltage by repeatedly measuring and displaying the voltage provided to each ADC input in parallel.

1. Set up the target chip/board on a breadboard.
2. Connect all ADC inputs and provide a connection to a lab power supply.
3. Adjust the constant for the supply voltage, the serial connection,
   and perhaps the OSCCAL correction in the `anav` sketch.
4. Flash target sketch.
5. The voltage of all ADC inputs will be displayed over the serial line.
