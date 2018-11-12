# mlx90621-spin
---------------

This is a P8X32A/Propeller driver object for Melexis's mlx90621 16x4 IR sensing array.

## Salient Features

* I2C connection (tested up to 1MHz)
* Single Pixel, Line, Column or whole frame retrieval to buffer

## Limitations

* The sensor has a built-in EEPROM, which unfortunately has the same (fixed) slave address as the EEPROM on most any Propeller-based board, so the sensor must be connected to pins other than the "standard" Propeller I2C pins 28 & 29, or the Propeller must be booted by some method other than EEPROM.

## TODO

* Implement absolute temperature measurement (_requires implementing Ta, To calculations)
