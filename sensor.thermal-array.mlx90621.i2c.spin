{
    --------------------------------------------
    Filename: sensor.thermal-array.mlx90621.i2c.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90621
        16x4 IR array
    Copyright (c) 2020
    started: Dec 2, 2019
    Updated: Nov 22, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR        = core#SLAVE_ADDR
    SLAVE_RD        = core#SLAVE_ADDR|1

    DEF_HZ          = 100_000
    I2C_MAX_FREQ    = core#I2C_MAX_FREQ

    EE_SIZE         = 256

    WIDTH           = 16
    HEIGHT          = 4
    XMAX            = WIDTH-1
    YMAX            = HEIGHT-1

' I2C Fast Mode
    I2CFMODE_ENA    = 0
    I2CFMODE_DIS    = 1

' Operation modes
    CONTINUOUS      = 0
    SINGLE          = 1

' Sensor power states
    OFF             = 0
    ON              = 1

' ADC reference settings
    ADCREF_HI       = 0
    ADCREF_LO       = 1

' On-sensor EEPROM
    EE_ENA          = 0
    EE_DIS          = 1

' Offsets to sensor calibration data
    EE_OFFS_VTH     = $DA
    EE_OFFS_KT1     = $DC
    EE_OFFS_KT2     = $DE
    EE_OFFS_KT1SCL  = $D2
    EE_OFFS_KT2SCL  = $D2
    EE_OFFS_OSCTRIM = $F7
    EE_OFFS_CFGH    = $F6
    EE_OFFS_CFGL    = $F5

    RAM_OFFS_PTAT   = $40
    RAM_OFFS_CPIX   = $41

    CFG_CKBYTE      = $55
    OSC_CKBYTE      = $AA

    W               = 0
    R               = 1

OBJ

    core    : "core.con.mlx90621"
    i2c     : "com.i2c"
    time    : "time"
    io      : "io"

VAR

    word _ptat

    byte _ee_data[EE_SIZE]

PUB Null{}
' This is not a top-level object

PUB Start(SCL_PIN, SDA_PIN, I2C_HZ): okay
' Start using "standard" Propeller I2C pins, and 100kHz
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31)
        if I2C_HZ =< core#I2C_MAX_FREQ
            if okay := i2c.setupx(SCL_PIN, SDA_PIN, I2C_HZ)' start the I2C driver
                time.msleep(3)
                if i2c.present(core#EE_SLAVE_ADDR)
                    readeeprom{}                        '   ...to read the EEPROM.
                    time.msleep(5)
                    i2c.setupx(SCL_PIN, SDA_PIN, I2C_HZ)' This time re-setup for the sensor
                    time.msleep(5)
                    if i2c.present (SLAVE_WR)           ' check device bus presence
                        return okay

    return FALSE                                ' something above failed

PUB Stop{}

    i2c.terminate{}

PUB Defaults{}
' Write osc trimming val extracted from EEPROM address $F7
    osctrim(_ee_data[EE_OFFS_OSCTRIM])
    refreshrate(1)
    adcres(18)
    opmode(CONTINUOUS)
    powered(TRUE)
    i2cfm(TRUE)
    eeprom(TRUE)
    adcreference(ADCREF_LO)
    reset(TRUE)
    time.msleep(5)

PUB ADCReference(mode): curr_mode
' Set ADC reference high, low
'   Valid values:
'      ADCREF_HI (0) - ADC High reference enabled
'     *ADCREF_LO (1) - ADC Low reference enabled
'   Any other value polls the chip and returns the current setting
' NOTE: Re-calibration must be done after this method is called
    readreg(core#CONFIG, 2, 0, @curr_mode)
    case mode
        ADCREF_HI, ADCREF_LO:
            mode := mode << core#FLD_ADCHIGHREF
        other:
            return (curr_mode >> core#FLD_ADCHIGHREF) & %1

    mode := ((curr_mode & core#MASK_ADCHIGHREF) | mode) & core#CONFIG_MASK
    writereg(core#CONFIG, mode)

'TODO: Call Re-cal method here

PUB ADCRes(bits): curr_res
' Set ADC resolution, in bits
'   Valid values: 15..18 (default: 18)
'   Any other value polls the chip and returns the current setting
    readreg(core#CONFIG, 2, 0, @curr_res)
    case bits
        15..18:
            bits := lookdownz(bits: 15, 16, 17, 18) << core#FLD_ADCRES
        other:
            curr_res := (curr_res >> core#FLD_ADCRES) & core#BITS_ADCRES
            return lookupz(result: 15, 16, 17, 18)

    bits := ((curr_res & core#MASK_ADCRES) | bits) & core#CONFIG_MASK
    writereg(core#CONFIG, bits)

'    _adc_res := 3-((_cfg_reg >> 4) & %11)   'Update the VAR used in calculations

PUB AmbientTemp{}: ptat | Kt1, Kt2, Vth, Ta
' Read Proportional To Ambient Temperature sensor   'XXX needs calc
    readreg(core#PTAT, 1, 0, @ptat)
{    Kt1 := (_ee_data[$DD] << 8) | _ee_data[$DC]
    Kt2 := (_ee_data[$DF] << 8) | _ee_data[$DE]
    Vth := (_ee_data[$DB] << 8) | _ee_data[$DA]
    Ta := Kt1 * Kt1
    Ta := Ta - (4 * Kt2)
    Ta := Ta * (Vth - PTAT_data)
    Ta := ^^Ta
    Ta := Ta + (-Kt1)
    Ta := Ta / (Kt2 * 2)}
    return

PUB Dump_EE(ptr_buff)
' Copy downloaded EEPROM image to ptr_buff
' NOTE: This buffer must be at least 256 bytes
    bytemove(ptr_buff, @_ee_data, EE_SIZE)

PUB EEPROM(state): curr_state
' Enable/disable the sensor's built-in EEPROM
'   Valid values:
'      TRUE (-1 or 1): Sensor's built-in EEPROM state (default)
'      FALSE: Sensor's built-in EEPROM disabled
'   Any other value polls the chip and returns the current setting
'   NOTE: Use with care! Driver will fail to restart if EEPROM is disabled.
'       Cycle power in this case.
    readreg(core#CONFIG, 2, 0, @curr_state)
    case ||(state)
        0, 1:
            state := (1-(||(state))) << core#FLD_EEPROMENA
        other:
            return (1-((curr_state >> core#FLD_EEPROMENA) & %1)) * TRUE

    state := ((curr_state & core#MASK_EEPROMENA) | state) & core#CONFIG_MASK
    writereg(core#CONFIG, state)

PUB GetColumn(ptr_buff, col) | tmpframe[2], tmp, offs, line
' Read a single column of pixels from the sensor into ptr_buff
'   NOTE This buffer must be at least 4 words
    if not lookdown(col: 0..15)
        return

    readreg(col * 4, 4, 1, @tmpframe)
    repeat line from 0 to YMAX
        offs := (col * 4) + line
        long[ptr_buff][tmp] := ~~tmpframe.word[offs]

PUB GetFrame(ptr_buff) | tmpframe[64], offs
' Read entire frame from sensor and store it in buffer at ptr_buff
'   NOTE: This buffer must be at least 64 words
    readreg(0, 64, 1, @tmpframe)
    repeat offs from 0 to 63
        long[ptr_buff][offs] := ~~tmpframe.word[offs]

PUB GetFrameExt(ptr_buff) | tmpframe[66], offs, line, col
' Read entire frame, as well as PTAT and compensation pixel data from sensor and stores it in buffer at ptr_buff
'   NOTE: This buffer must be at least 66 words
    readreg(0, 66, 1, @tmpframe)
    repeat offs from 0 to 65
        long[ptr_buff][offs] := ~~tmpframe.word[offs]
    _PTAT := tmpframe[RAM_OFFS_PTAT]                        ' Get PTAT data

PUB GetLine(ptr_buff, line) | tmpframe[8], offs, col
' Read a single line of pixels from the sensor into ptr_buff
'   NOTE: This buffer must be at least 16 words
    if not lookdown(line: 0..3)
        return
    readreg(line, 16, 4, @tmpframe)
    repeat col from 0 to XMAX
        offs := (col * 4) + line
        long[ptr_buff][offs] := ~~tmpframe.word[offs]

PUB GetPixel(ptr_buff, col, line): pix_word | tmpframe, offs
' Read a single pixel from the sensor into ptr_buff
'   Returns: pixel value
'   NOTE: This buffer must be at least 1 word
    case col
        0..XMAX:
        other:
            return

    case line
        0..YMAX:
        other:
            return

    offs := (col * 4) + line                                ' Calc offset within array image

    readreg(offs, 1, 0, @tmpframe)
    long[ptr_buff][offs] := pix_word := ~~tmpframe.word[0]

PUB I2CFM(mode): curr_mode
' Enable I2C Fast Mode+
'   Valid values:
'     *TRUE (-1 or 1): Max I2C bus speed 1000kbit/sec
'      FALSE (0): Max I2C bus speed 400kbit/sec
'   NOTE: This is independent of, and has no effect on what speed the driver was started with.
'       e.g., you may have started the driver at 400kHz, but left this option at the default 1000kHz.
'       Thus, the sensor will _allow_ traffic at up to 1000kHz, but the driver will only actually be operating at 400kHz.
'   Any other value polls the chip and returns the current setting
    readreg(core#CONFIG, 2, 0, @curr_mode)
    case ||(mode)
        0, 1:
            mode := (1-(||(mode))) << core#FLD_I2CFMP
        other:
            return (1-((curr_mode >> core#FLD_I2CFMP) & %1)) == 1

    mode := ((curr_mode & core#MASK_I2CFMP) | mode) & core#CONFIG_MASK
    writereg(core#CONFIG, curr_mode)

PUB Measure{}
' Perform measurement, when OpMode is set to SINGLE
'   NOTE: This method waits/blocks while a measurement is ongoing
    writereg(core#CMD_STEP_MEASURE, 0)
    repeat while measuring{}

PUB Measuring{}: flag
' Flag indicating a measurement is running
'   Returns:
'       FALSE (0): No IR measurement running
'       TRUE (-1): IR measurement running
'   NOTE: This method is intended for use when MeasureMode is set to SINGLE
    readreg(core#CONFIG, 2, 0, @flag)
    return ((flag >> core#FLD_MEASURING) & %1) == 1

PUB OpMode(mode): curr_mode
' Set measurement mode
'   Valid values:
'     *CONTINUOUS (0) - Continuous measurement mode
'      SINGLE (1) - Single-measurement mode only
'   Any other value polls the chip and returns the current setting
'   NOTE: In SINGLE mode, measurements must be triggered manually using the Measure method
    readreg(core#CONFIG, 2, 0, @curr_mode)
    case mode
        CONTINUOUS, SINGLE:
            mode := (mode << core#FLD_MEASMODE)
        other:
            return (curr_mode >> core#FLD_MEASMODE) & %1

    mode := ((curr_mode & core#MASK_MEASMODE) | mode) & core#CONFIG_MASK
    writereg(core#CONFIG, curr_mode)

PUB OSCTrim(val): curr_val
' Set Oscillator Trim value
'   Valid values: 0..127 (default: 0)
'   Any other value polls the chip and returns the current setting
'   NOTE: It is recommended to use the factory set value contained in the device's EEPROM.
    readreg(core#OSC_TRIM, 1, 0, @curr_val)
    case val
        0..127:
        other:
            return curr_val & core#OSC_TRIM_MASK

    val := val & core#OSC_TRIM_MASK
    writereg(core#OSC_TRIM, curr_val)

PUB Powered(state): curr_state
' Power on sensor
'   Valid values:
'      *FALSE (0) - Sleep
'       TRUE (-1 or 1) - Powered on/normal operation
'   Any other value polls the chip and returns the current setting
    readreg(core#CONFIG, 2, 0, @curr_state)
    case ||(state)
        0, 1:
            state := (not(||(state))) << core#FLD_OPMODE        ' Logic on the chip is reversed
        other:
            return not( (curr_state >> core#FLD_OPMODE) & %1 )

    state := ((curr_state & core#MASK_OPMODE) | state) & core#CONFIG_MASK
    writereg(core#CONFIG, state)

PUB ReadEEPROM{} | tmp
' Read EEPROM contents into RAM
    bytefill (@_ee_data, $00, EE_SIZE)  'Make sure data in RAM copy of EEPROM image is clear

    i2c.start{}                                 'start reading at addr $00
    i2c.write(core#EE_SLAVE_ADDR)
    i2c.write($00)

    i2c.start{}                                 'Read in the EEPROM
    i2c.write(core#EE_SLAVE_ADDR|1)
    repeat tmp from 0 to EE_SIZE-1
        _ee_data[tmp] := i2c.read (tmp == EE_SIZE-1)
    i2c.stop{}

PUB RefreshRate(rate): curr_rate
' Set sensor refresh rate
'   Valid values are 0, for 0.5Hz, or 1 to 512 in powers of 2 (default: 1)
'   Any other value polls the chip and returns the current setting
'   NOTE: Higher rates will yield noisier images
    readreg(core#CONFIG, 2, 0, @curr_rate)
    case rate
        512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0:
            rate := lookdownz(rate: 512, 512, 512, 512, 512, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0)
        other:
            curr_rate := curr_rate & core#BITS_REFRATE
            return lookupz(curr_rate: 512, 512, 512, 512, 512, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0)

    rate := ((curr_rate & core#MASK_REFRATE) | rate) & core#CONFIG_MASK
    writereg(core#CONFIG, rate)

PUB Reset(set): flag
' Set sensor reset flag
'   Valid values: TRUE (-1 or 1)
'   Any other value polls the chip and returns the current setting
'   NOTE: This must be done any time the sensor is initialized, _after_ the configuration register has been updated
'       If FALSE is returned, POR or brown-out has occurred and the process must be repeated
    readreg(core#CONFIG, 2, 0, @flag)
    case ||(set)
        1:
            set := 1 << core#FLD_RESET
        other:
            return ((flag >> core#FLD_RESET) & %1) == 1

    set := ((flag & core#MASK_RESET) | set) & core#CONFIG_MASK
    writereg(core#CONFIG, set)

PRI readReg(reg_nr, nr_reads, rd_step, ptr_buff): result | cmd_packet[2], tmp

    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_READREG
    case reg_nr
        $00..$41:                           'RAM
            cmd_packet.byte[2] := reg_nr
            cmd_packet.byte[3] := rd_step
            cmd_packet.byte[4] := nr_reads
            nr_reads <<= 1
        $92..$93:                           'Configuration regs
            cmd_packet.byte[2] := reg_nr
            cmd_packet.byte[3] := 0         'Address step
            cmd_packet.byte[4] := 1         'Number of reads sent as command parameter to device
            nr_reads := 2
        other:
            return

    i2c.start
    i2c.wr_block (@cmd_packet, 5)

    i2c.start
    i2c.write (SLAVE_RD)
    i2c.rd_block (ptr_buff, nr_reads, TRUE)
    i2c.stop

PRI writeReg(reg_nr, val): result | cmd_packet[2], nr_bytes, tmp

    cmd_packet.byte[0] := SLAVE_WR
    case reg_nr
        core#CONFIG:
            cmd_packet.byte[1] := core#CMD_WRITEREG_CFG
            cmd_packet.byte[2] := val.byte[0] - CFG_CKBYTE
            cmd_packet.byte[3] := val.byte[0]
            cmd_packet.byte[4] := val.byte[1] - CFG_CKBYTE
            cmd_packet.byte[5] := val.byte[1]
            nr_bytes := 6

        core#OSC_TRIM:
            cmd_packet.byte[1] := core#CMD_WRITEREG_OSCTRIM
            cmd_packet.byte[2] := val.byte[0] - OSC_CKBYTE
            cmd_packet.byte[3] := val.byte[0]
            cmd_packet.byte[4] := val.byte[1] - OSC_CKBYTE
            cmd_packet.byte[5] := val.byte[1]
            nr_bytes := 6

        core#CMD_STEP_MEASURE:
            cmd_packet.byte[1] := core#CMD_STEP_MEASURE & $FF
            cmd_packet.byte[2] := (core#CMD_STEP_MEASURE >> 8) & $FF
            nr_bytes := 3

        other:
            return

    i2c.start
    i2c.wr_block (@cmd_packet, nr_bytes)
    i2c.stop

DAT
{
    --------------------------------------------------------------------------------------------------------
    TERMS OF USE: MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
    associated documentation files (the "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial
    portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
    LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    --------------------------------------------------------------------------------------------------------
}
