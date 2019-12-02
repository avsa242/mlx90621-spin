# mlx90621-spin
---------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for Melexis's MLX90621 16x4 IR sensing array

## Salient Features

* I2C connection at up to 1MHz (P2/SPIN2: TBD)
* Single Pixel, Line, Column or whole frame retrieval to buffer
* Continuous or step measure mode
* Set sensor ADC resolution (15..18 bits)
* Set sensor framerate (0.5..512Hz)

## Requirements

* P1/SPIN1: 1 extra core/cog for the PASM I2C driver
* P2/SPIN2: N/A

## Compiler Compatibility

* P1/SPIN1: OpenSpin (tested with 1.00.81)
* P2/SPIN2: FastSpin (tested with 4.0.3)

## Limitations

* Very early in development - may malfunction, or outright fail to build
* The sensor has a built-in EEPROM, which unfortunately has the same (fixed) slave address as the EEPROM on most any Propeller 1-based board, so the sensor must be connected to pins other than the "standard" Propeller I2C pins 28 & 29, or the Propeller must be booted by some method other than EEPROM (doesn't apply to P2)
* No correction is performed on the sensor data (i.e., Ta and To; ambient temperature and object temperature). Only the raw image data is collected and returned

## TODO

- [ ] Implement Ta/ambient temperature measurement
- [ ] Implement To/object temperature measurement
