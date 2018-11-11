{
    --------------------------------------------
    Filename: core.con.mlx90621
    Author: Jesse Burt
    Copyright (c) 2018
    Started: Nov 11, 2018
    Updated: Nov 11, 2018
    See end of file for terms of use.
    --------------------------------------------
}

CON

    I2C_MAX_FREQ            = 1_000_000
    SLAVE_ADDR              = $60 << 1

    EE_MAX_FREQ             = 400_000
    EE_SLAVE_ADDR           = $50 << 1

'' Register definitions
    CMD_READREG             = $02
    CMD_WRITEREG_CFG        = $03
    CMD_WRITEREG_OSCTRIM    = $04


PUB Null
'' This is not a top-level object
