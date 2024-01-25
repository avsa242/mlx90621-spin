# mlx90621-spin
---------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for Melexis's MLX90621 16x4 IR sensing array

**IMPORTANT**: This software is meant to be used with the [spin-standard-library](https://github.com/avsa242/spin-standard-library) (P8X32A) or [p2-spin-standard-library](https://github.com/avsa242/p2-spin-standard-library) (P2X8C4M64P). Please install the applicable library first before attempting to use this code, otherwise you will be missing several files required to build the project.


## Videos

* [P2 VGA-based demo](https://youtu.be/9pcsTAazFpA)


## Salient Features

* I2C connection at up to 1MHz (P2/SPIN2: TBD)
* Single Pixel, Line, Column or whole frame retrieval to buffer
* Continuous or step measure mode
* Set sensor ADC resolution (15..18 bits)
* Set sensor framerate (0.5..512Hz)


## Requirements

P1/SPIN1:
* spin-standard-library
* P1/SPIN1: 1 extra core/cog for the PASM I2C engine

P2/SPIN2:
* p2-spin-standard-library


## Compiler Compatibility

| Processor | Language | Compiler               | Backend      | Status                |
|-----------|----------|------------------------|--------------|-----------------------|
| P1        | SPIN1    | FlexSpin (6.8.0)       | Bytecode     | OK                    |
| P1        | SPIN1    | FlexSpin (6.8.0)       | Native/PASM  | OK                    |
| P2        | SPIN2    | FlexSpin (6.8.0)       | NuCode       | Build OK              |
| P2        | SPIN2    | FlexSpin (6.8.0)       | Native/PASM2 | OK                    |


## Limitations

* Very early in development - may malfunction, or outright fail to build
* The sensor has a built-in EEPROM, which unfortunately has the same (fixed) slave address as the EEPROM on most any Propeller 1-based board, so the sensor must be connected to pins other than the "standard" Propeller I2C pins 28 & 29, or the Propeller must be booted by some method other than EEPROM (doesn't apply to P2)
* No correction is performed on the sensor data (i.e., Ta and To; ambient temperature and object temperature). Only the raw image data is collected and returned

