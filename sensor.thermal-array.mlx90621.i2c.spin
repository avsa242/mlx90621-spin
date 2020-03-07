{
    --------------------------------------------
    Filename: sensor.thermal-array.mlx90621.i2c.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90621
     16x4 IR array
    Copyright (c) 2020
    Started: Jan 14, 2018
    Updated: Mar 7, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR        = core#SLAVE_ADDR
    SLAVE_RD        = core#SLAVE_ADDR|1

    DEF_SCL         = 28
    DEF_SDA         = 29
    DEF_HZ          = 100_000
    I2C_MAX_FREQ    = core#I2C_MAX_FREQ

    EE_SIZE         = 256

' I2C Fast Mode
    I2CFMODE_ENA    = 0
    I2CFMODE_DIS    = 1

' Operation modes
    CONT            = 0
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
    type    : "system.types"
    u64     : "math.unsigned64"

VAR

    word _raw_frame[64]
    word _PTAT

    byte _ee_data[EE_SIZE]

PUB Null
'This is not a top-level object

PUB Start: okay                                                 'Default to "standard" Propeller I2C pins and 100kHz

    okay := Startx (DEF_SCL, DEF_SDA, DEF_HZ)

PUB Startx(SCL_PIN, SDA_PIN, I2C_HZ): okay

    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31)
        if I2C_HZ =< core#I2C_MAX_FREQ
            if okay := i2c.Setupx (SCL_PIN, SDA_PIN, I2C_HZ)            ' Start the I2C driver
                time.MSleep (3)
                if i2c.Present (core#EE_SLAVE_ADDR)
                    ReadEEPROM                                          '   ...to read the EEPROM.
                    i2c.terminate
                    if okay := i2c.setupx (SCL_PIN, SDA_PIN, I2C_HZ)    ' This time re-setup for the sensor
                        time.MSleep (5)
                        if i2c.Present (SLAVE_WR)                       ' Check for response
                            return okay

    return FALSE                                                        'If we got here, something went wrong

PUB Stop

    i2c.terminate

PUB Defaults

    OSCTrim (_ee_data[EE_OFFS_OSCTRIM]) ' Write osc trimming val extracted from EEPROM address $F7
    RefreshRate (1)
    ADCRes (18)
    OpMode (CONT)
    Powered (TRUE)
    I2CFM (TRUE)
    EEPROM (TRUE)
    ADCReference (ADCREF_LO)
    Reset (TRUE)
    time.MSleep (5)

PUB ADCReference(mode) | tmp
' Set ADC reference high, low
'   Valid values:
'      ADCREF_HI (0) - ADC High reference enabled
'     *ADCREF_LO (1) - ADC Low reference enabled
'   Any other value polls the chip and returns the current setting
' NOTE: Re-calibration must be done after this method is called
    readReg (core#CONFIG, 2, 0, @tmp)
    case mode
        ADCREF_HI, ADCREF_LO:
            mode := mode << core#FLD_ADCHIGHREF
        OTHER:
            return (tmp >> core#FLD_ADCHIGHREF) & %1
    tmp &= core#MASK_ADCHIGHREF
    tmp := (tmp | mode) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

'TODO: Call Re-cal method here

PUB ADCRes(bits) | tmp
' Set ADC resolution, in bits
'   Valid values: 15..18 (default: 18)
'   Any other value polls the chip and returns the current setting
    readReg (core#CONFIG, 2, 0, @tmp)
    case bits
        15..18:
            bits := lookdownz(bits: 15, 16, 17, 18) << core#FLD_ADCRES
        OTHER:
            result := (tmp >> core#FLD_ADCRES) & core#BITS_ADCRES
            return result := lookupz(result: 15, 16, 17, 18)
    tmp &= core#MASK_ADCRES
    tmp := (tmp | bits) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

'    _adc_res := 3-((_cfg_reg >> 4) & %11)   'Update the VAR used in calculations

PUB Dump_EE(buff_addr)
' Copy downloaded EEPROM image to buff_addr
' NOTE: This buffer must be at least 256 bytes
    bytemove(buff_addr, @_ee_data, EE_SIZE)

PUB EEPROM(enabled) | tmp
' Enable/disable the sensor's built-in EEPROM
'   Valid values:
'      TRUE (-1 or 1): Sensor's built-in EEPROM enabled (default)
'      FALSE: Sensor's built-in EEPROM disabled
'   Any other value polls the chip and returns the current setting
'   NOTE: Use with care! Driver will fail to restart if EEPROM is disabled.
'       Cycle power in this case.
    readReg (core#CONFIG, 2, 0, @tmp)
    case ||enabled
        0, 1:
            enabled := (1-(||enabled)) << core#FLD_EEPROMENA
        OTHER:
            return (1-((tmp >> core#FLD_EEPROMENA) & %1)) * TRUE
    tmp &= core#MASK_EEPROMENA
    tmp := (tmp | enabled) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

PUB GetColumn(buff_addr, col) | rawpix[2], line, offset
' Read a single column of pixels from the sensor into buff_addr
'   NOTE This buffer must be at least 4 words
    if not lookdown(col: 0..15)
        return

    readReg (col * 4, 4, 1, @rawpix)
    repeat line from 0 to 3
        offset := (col * 4) + line
        word[buff_addr][offset] := type.u16_s16 (rawpix.word[line])

PUB GetFrame(buff_addr)
' Read entire frame from sensor and stores it in buffer at buff_addr
'   NOTE: This buffer must be at least 64 words
    readReg (0, 64, 1, buff_addr)

PUB GetFrameExt(buff_addr) | line, col, rawpix[33], offset
' Read entire frame, as well as PTAT and compensation pixel data from sensor and stores it in buffer at buff_addr
'   NOTE: This buffer must be at least 66 words
    readReg (0, 66, 1, @rawpix)
    repeat line from 0 to 3
        repeat col from 0 to 15
            offset := (col * 4) + line       'Compute offset location in array of current pixel
            word[buff_addr][offset] := type.u16_s16 (rawpix.word[offset])

    _PTAT := (word[buff_addr][RAM_OFFS_PTAT] := rawpix.word[RAM_OFFS_PTAT])  ' Get PTAT data
    word[buff_addr][RAM_OFFS_CPIX] := rawpix.word[RAM_OFFS_CPIX]             ' and Compensation Pixel, too

PUB GetLine(buff_addr, line) | rawpix[8], col, offset
' Read a single line of pixels from the sensor into buff_addr
'   NOTE: This buffer must be at least 16 words
    if not lookdown(line: 0..3)
        return
    readReg (line, 16, 4, @rawpix)
    repeat col from 0 to 15
        offset := (col * 4) + line
        word[buff_addr][offset] := type.u16_s16 (rawpix.word[col])

PUB GetPixel(buff_addr, col, line) | rawpix, offset
' Read a single pixel from the sensor into buff_addr
'   Returns: pixel value
'   NOTE: This buffer must be at least 1 word
    case lookdown(col: 0..15)
        1..16:
        OTHER:
            return

    case lookdown(line: 0..3)
        1..4:
        OTHER:
            return

    offset := (col * 4) + line 'Compute offset location in array of current pixel

    readReg (offset, 1, 0, @rawpix)
    return word[buff_addr][offset] := type.u16_s16 (rawpix & $FFFF)

PUB I2CFM(enabled) | tmp
' Enable I2C Fast Mode+
'   Valid values:
'     *TRUE (-1 or 1): Max I2C bus speed 1000kbit/sec
'      FALSE (0): Max I2C bus speed 400kbit/sec
'   NOTE: This is independent of, and has no effect on what speed the driver was started with.
'       e.g., you may have started the driver at 400kHz, but left this option at the default 1000kHz.
'       Thus, the sensor will _allow_ traffic at up to 1000kHz, but the driver will only actually be operating at 400kHz.
'   Any other value polls the chip and returns the current setting
    readReg (core#CONFIG, 2, 0, @tmp)
    case ||enabled
        0, 1:
            enabled := (1-(||enabled)) << core#FLD_I2CFMP
        OTHER:
            return (1-((tmp >> core#FLD_I2CFMP) & %1)) * TRUE
    tmp &= core#MASK_I2CFMP
    tmp := (tmp | enabled) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

PUB Measure
' Perform measurement, when OpMode is set to SINGLE
'   NOTE: This method waits/blocks while a measurement is ongoing
    writeReg(core#CMD_STEP_MEASURE, 0)
    repeat while Measuring

PUB Measuring
' Flag indicating a measurement is running
'   Returns:
'       FALSE (0): No IR measurement running
'       TRUE (-1): IR measurement running
'   NOTE: This method is intended for use when MeasureMode is set to SINGLE
    readReg (core#CONFIG, 2, 0, @result)
    result := ((result >> core#FLD_MEASURING) & %1) * TRUE

PUB OpMode(mode) | tmp
' Set measurement mode
'   Valid values:
'     *CONT (0) - Continuous measurement mode
'      SINGLE (1) - Single-measurement mode only
'   Any other value polls the chip and returns the current setting
'   NOTE: In SINGLE mode, measurements must be triggered manually using the Measure method
    readReg (core#CONFIG, 2, 0, @tmp)
    case mode
        CONT, SINGLE:
            mode := (mode << core#FLD_MEASMODE)
        OTHER:
            return (tmp >> core#FLD_MEASMODE) & %1
    tmp &= core#MASK_MEASMODE
    tmp := (tmp | mode) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

PUB OSCTrim(val) | tmp
' Set Oscillator Trim value
'   Valid values: 0..127 (default: 0)
'   Any other value polls the chip and returns the current setting
'   NOTE: It is recommended to use the factory set value contained in the device's EEPROM.
    readReg (core#OSC_TRIM, 1, 0, @tmp)
    case val
        0..127:
        OTHER:
            return tmp & core#OSC_TRIM_MASK

    tmp := val & core#OSC_TRIM_MASK
    writeReg (core#OSC_TRIM, tmp)

PUB Powered(enabled) | tmp
' Power on sensor
'   Valid values:
'      *FALSE (0) - Sleep
'       TRUE (-1 or 1) - Powered on/normal operation
'   Any other value polls the chip and returns the current setting
    readReg (core#CONFIG, 2, 0, @tmp)
    case ||enabled
        0, 1:
            enabled := ( not(||enabled) ) << core#FLD_OPMODE        ' Logic on the chip is reversed
        OTHER:
            return not( (tmp >> core#FLD_OPMODE) & %1 )

    tmp &= core#MASK_OPMODE
    tmp := (tmp | enabled) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

PUB PTAT | PTAT_data, Kt1, Kt2, Vth, Ta
' Read Proportional To Ambient Temperature sensor
    readReg(core#PTAT, 1, 0, @PTAT_data)
{    Kt1 := (_ee_data[$DD] << 8) | _ee_data[$DC]
    Kt2 := (_ee_data[$DF] << 8) | _ee_data[$DE]
    Vth := (_ee_data[$DB] << 8) | _ee_data[$DA]
    Ta := Kt1 * Kt1
    Ta := Ta - (4 * Kt2)
    Ta := Ta * (Vth - PTAT_data)
    Ta := ^^Ta
    Ta := Ta + (-Kt1)
    Ta := Ta / (Kt2 * 2)}
    return PTAT_data

PUB ReadEEPROM
' Read EEPROM contents into RAM
    bytefill (@_ee_data, $00, EE_SIZE)  'Make sure data in RAM copy of EEPROM image is clear

    i2c.Start                           'Start reading at addr $00
    i2c.Write (core#EE_SLAVE_ADDR)
    i2c.Write ($00)

    i2c.Start                           'Read in the EEPROM
    i2c.Write (core#EE_SLAVE_ADDR|1)
    i2c.Rd_Block (@_ee_data, EE_SIZE, TRUE)
    i2c.Stop

PUB RefreshRate(Hz) | tmp
' Set sensor refresh rate
'   Valid values are 0, for 0.5Hz, or 1 to 512 in powers of 2 (default: 1)
'   Any other value polls the chip and returns the current setting
'   NOTE: Higher rates will yield noisier images
    readReg (core#CONFIG, 2, 0, @tmp)
    case Hz
        512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0:
            Hz := lookdownz(Hz: 512, 512, 512, 512, 512, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0)
        OTHER:
            result := tmp & core#BITS_REFRATE
            return lookupz(result: 512, 512, 512, 512, 512, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0)

    tmp &= core#MASK_REFRATE
    tmp := (tmp | Hz) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

PUB Reset(set) | tmp
' Set sensor reset flag
'   Valid values: TRUE (-1 or 1)
'   Any other value polls the chip and returns the current setting
'   NOTE: This must be done any time the sensor is initialized, _after_ the configuration register has been updated
'       If FALSE is returned, POR or brown-out has occurred and the process must be repeated
    readReg (core#CONFIG, 2, 0, @tmp)
    case ||set
        1:
            set := 1 << core#FLD_RESET
        OTHER:
            return ((tmp >> core#FLD_RESET) & %1) * TRUE
    tmp &= core#MASK_RESET
    tmp := (tmp | set) & core#CONFIG_MASK
    writeReg (core#CONFIG, tmp)

PRI readReg(reg, nr_reads, rd_step, rd_buf) | cmd_packet[2]

    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_READREG
    case reg
        $00..$41:                           'RAM
            cmd_packet.byte[2] := reg
            cmd_packet.byte[3] := rd_step
            cmd_packet.byte[4] := nr_reads
            nr_reads <<= 1
        $92..$93:                           'Configuration regs
            cmd_packet.byte[2] := reg
            cmd_packet.byte[3] := 0         'Address step
            cmd_packet.byte[4] := 1         'Number of reads sent as command parameter to device
            nr_reads := 2
        OTHER:
            return

    i2c.start
    i2c.wr_block (@cmd_packet, 5)
    i2c.start
    i2c.write (SLAVE_RD)
    i2c.rd_block (rd_buf, nr_reads, TRUE)
    i2c.stop

PRI writeReg(reg, val) | cmd_packet[2], nr_bytes

    cmd_packet.byte[0] := SLAVE_WR
    case reg
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
            cmd_packet.byte[1] := core#CMD_STEP_MEASURE
            nr_bytes := 2

        OTHER:
            return

    i2c.start
    i2c.wr_block (@cmd_packet, nr_bytes)
    i2c.stop

PRI readData(buff_ptr, addr_start, addr_step, word_count) | cmd_packet[2]

    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_READREG
    cmd_packet.byte[2] := addr_start
    cmd_packet.byte[3] := addr_step
    cmd_packet.byte[4] := word_count
    
    i2c.start
    i2c.wr_block (@cmd_packet, 5)
    i2c.start
    i2c.write (SLAVE_RD)
    i2c.rd_block (buff_ptr, word_count << 1, TRUE) '*2 = 81.6uS, << 1 = 71.8uS
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
