# Micronucleus bootloader size

| File name                       | Boot address | App size | Boot size |
| ------------------------------- | ------------ | -------- | --------- |
| attiny25_micronucleus_16MHz.hex | 0x0280       | 640-4    | 1408+4    |
| attiny45_micronucleus_16MHz.hex | 0x0A00       | 2560-4   | 1536+4    |
| attiny85_micronucleus_16MHz.hex | 0x1A80       | 6784-4   | 1408+4    |
| attiny48_micronucleus_16MHz.hex | 0x0A80       | 2688-4   | 1408+4    |
| attiny88_micronucleus_16MHz.hex | 0x1A80       | 6784-4   | 1408+4    |

The original micronucleus bootloaders, both for Digispark and for MH-Tiny, have apparently a size of 2176 bytes. This is at least the impression one gets when looking into Arduino board package files for the original boards.

Further, for some obscure reasons, the application size is reduced by another 4 bytes. So the effective bootloader size is 2180. Similarly, one needs to add 4 more bytes to the bootloader sizes reported in the table.
