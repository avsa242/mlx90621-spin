{
    --------------------------------------------
    Filename: core.con.mlx90621
    Author: Jesse Burt
    Copyright (c) 2018
    Started: Nov 11, 2018
    Updated: Mar 13, 2019
    See end of file for terms of use.
    --------------------------------------------
}

CON

    I2C_MAX_FREQ            = 1_000_000
    SLAVE_ADDR              = $60 << 1

    EE_MAX_FREQ             = 400_000
    EE_SLAVE_ADDR           = $50 << 1

' Register definitions
    CMD_READREG             = $02
    CMD_WRITEREG_CFG        = $03
    CMD_WRITEREG_OSCTRIM    = $04
    CMD_STEP_MEASURE        = $0801

    PTAT                    = $40
    COMP_PIXEL              = $41

    CONFIG                  = $92
    CONFIG_MASK             = $5EFF
        FLD_REFRATE         = 0
        FLD_ADCRES          = 4
        FLD_MEASMODE        = 6
        FLD_OPMODE          = 7
        FLD_MEASURING       = 9
        FLD_RESET           = 10
        FLD_I2CFMP          = 11
        FLD_EEPROMENA       = 12
        FLD_ADCHIGHREF      = 14
        BITS_REFRATE        = %1111
        BITS_ADCRES         = %11
        MASK_REFRATE        = CONFIG_MASK ^ (BITS_REFRATE << FLD_REFRATE)
        MASK_ADCRES         = CONFIG_MASK ^ (BITS_ADCRES << FLD_ADCRES)
        MASK_MEASMODE       = CONFIG_MASK ^ (1 << FLD_MEASMODE)
        MASK_OPMODE         = CONFIG_MASK ^ (1 << FLD_OPMODE)
        MASK_MEASURING      = CONFIG_MASK ^ (1 << FLD_MEASURING)
        MASK_RESET          = CONFIG_MASK ^ (1 << FLD_RESET)
        MASK_I2CFMP         = CONFIG_MASK ^ (1 << FLD_I2CFMP)
        MASK_EEPROMENA      = CONFIG_MASK ^ (1 << FLD_EEPROMENA)
        MASK_ADCHIGHREF     = CONFIG_MASK ^ (1 << FLD_ADCHIGHREF)

    OSC_TRIM                = $93
    OSC_TRIM_MASK           = $007F

PUB Null
'' This is not a top-level object
