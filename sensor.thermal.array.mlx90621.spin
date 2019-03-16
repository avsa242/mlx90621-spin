{
    --------------------------------------------
    Filename: sensor.thermal.array.mlx90621.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90621
     16x4 IR array (I2C).
    Copyright (c) 2018
    Started: Jan 14, 2018
    Updated: Mar 13, 2019
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR        = core#SLAVE_ADDR
    SLAVE_RD        = core#SLAVE_ADDR|1

    DEF_SCL         = 28
    DEF_SDA         = 29
    DEF_HZ          = 400_000
    I2C_MAX_FREQ    = core#I2C_MAX_FREQ

    EE_SIZE         = 256

    I2CFMODE_ENA    = 0
    I2CFMODE_DIS    = 1

    MMODE_CONT      = 0
    MMODE_STEP      = 1

    OPMODE_NORM     = 0
    OPMODE_SLEEP    = 1

    ADCREF_HI       = 0
    ADCREF_LO       = 1

    EE_ENA          = 0
    EE_DIS          = 1

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
    POR_BIT         = %0000_0100_0000_0000

    W               = 0
    R               = 1

OBJ

    core    : "core.con.mlx90621"
    i2c     : "com.i2c"
    time    : "time"
    type    : "system.types"

VAR

    byte _ee_data[EE_SIZE]
    byte _osc_trim
    word _cfg_reg
    word _comp_pix
    byte _adc_res
    word _Vir_comp
    
    long _Vth_h, _Vth_l, _Vth25, _Kt1 , _Kt1_h, _Kt1_l, _Kt1_Scl, _Kt2, _Kt2_h, _Kt2_l, _Kt2_Scl, KtScl
    word _PTAT
    
    word _cfgset_adcref, _cfgset_ee_ena, _cfgset_i2cfm_ena, _cfgset_opmode, _cfgset_mmode, _cfgset_adcres, _cfgset_refr_rate
    word _cfgflag_por_bit, _cfgflag_measuring

PUB Null
''This is not a top-level object

PUB Start: okay                                                 'Default to "standard" Propeller I2C pins and 400kHz

    okay := Startx (DEF_SCL, DEF_SDA, DEF_HZ)

PUB Startx(SCL_PIN, SDA_PIN, I2C_HZ): okay

    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31)
        if I2C_HZ =< core#I2C_MAX_FREQ
            if okay := i2c.setupx (SCL_PIN, SDA_PIN, core#EE_MAX_FREQ)  'First, start the I2C object...
                time.MSleep (3)
                if i2c.present (core#EE_SLAVE_ADDR)
                    Read_EE                                             '...to read the EEPROM.
                    i2c.terminate                                       'Shut it down.
                    if okay := i2c.setupx (SCL_PIN, SDA_PIN, I2C_HZ)    'Then start it back up,
                        time.MSleep (5)                                 ' but this time setup for the sensor
                        if i2c.present (SLAVE_WR)                       'Check for response from device
                            return okay

    return FALSE                                                        'If we got here, something went wrong

PUB Stop

    i2c.terminate

PUB Defaults

    OSCTrim (peek_ee (EE_OFFS_OSCTRIM)) ' Write osc trimming val extracted from EEPROM address $F7
    RefreshRate (1)
    ADCRes (18)
    MeasureMode (MMODE_CONT)
    OperationMode (OPMODE_NORM)
    I2CFM (TRUE)
    EEPROM (TRUE)
    ADCReference (ADCREF_LO)

    time.MSleep (5)

PUB EEPROM(enabled) | tmp
' Enable/disable the sensor's built-in EEPROM
'   Valid values:
'      TRUE (-1 or 1): Sensor's built-in EEPROM enabled (default)
'      FALSE: Sensor's built-in EEPROM disabled
'   NOTE: Use with care! Driver will fail to restart if EEPROM is disabled.
'       Cycle power in this case.
'   Any other value polls the chip and returns the current setting
    readRegX (core#CONFIG, 1, 0, @tmp)
    case ||enabled
        0, 1:
            enabled := (1-(||enabled)) << core#FLD_EEPROMENA
        OTHER:
            return (1-((tmp >> core#FLD_EEPROMENA) & %1)) * TRUE
    tmp &= core#MASK_EEPROMENA
    tmp := (tmp | enabled) & core#CONFIG_MASK
    writeRegX (core#CONFIG, tmp)

PUB I2CFM(enabled) | tmp
' Enable I2C Fast Mode+
'   Valid values:
'      TRUE (-1 or 1): Max I2C bus speed 1000kbit/sec (default)
'      FALSE: Max I2C bus speed 400kbit/sec
'   NOTE: This is independent of, and has no effect on what speed the driver was started with.
'       e.g., you may have started the driver at 400kHz, but left this option at the default 1000kHz.
'       Thus, the sensor will _allow_ traffic at up to 1000kHz, but the driver will only actually be operating at 400kHz.
'   Any other value polls the chip and returns the current setting
    readRegX (core#CONFIG, 1, 0, @tmp)
    case ||enabled
        0, 1:
            enabled := (1-(||enabled)) << core#FLD_I2CFMP
        OTHER:
            return (1-((tmp >> core#FLD_I2CFMP) & %1)) * TRUE
    tmp &= core#MASK_I2CFMP
    tmp := (tmp | enabled) & core#CONFIG_MASK
    writeRegX (core#CONFIG, tmp)

PUB GetColumn(ptr_col, col) | rawpix[2], line, pixel

    if not lookdown(col: 0..15)
        return

    readData (@rawpix, col * 4, 1, 4)

    repeat line from 0 to 3
        pixel := (col * 4) + line
        word[ptr_col][pixel] := type.u16_s16 (rawpix.word[line])

PUB GetFrame(ptr_frame) | line, col, rawpix[32], pixel
'' Gets frame from sensor and stores it in buffer at ptr_frame
'' This buffer must be 32 longs/64 words
    readData (@rawpix, $00, 1, 64)
    repeat line from 0 to 3
        repeat col from 0 to 15
            pixel := (col * 4) + line       'Compute offset location in array of current pixel
            word[ptr_frame][pixel] := type.u16_s16 (rawpix.word[pixel])

PUB GetFrameExt(ptr_frame) | line, col, rawpix[33], pixel
'' Gets frame, as well as PTAT and compensation pixel data from sensor and stores it in buffer at ptr_frame
'' This buffer must be 33 longs/66 words
    readData (@rawpix, $00, 1, 66)
    repeat line from 0 to 3
        repeat col from 0 to 15
            pixel := (col * 4) + line       'Compute offset location in array of current pixel
            word[ptr_frame][pixel] := type.u16_s16 (rawpix.word[pixel])

    _PTAT := (word[ptr_frame][RAM_OFFS_PTAT] := rawpix.word[RAM_OFFS_PTAT]) * 100 ' Also get PTAT data
    word[ptr_frame][RAM_OFFS_CPIX] := rawpix.word[RAM_OFFS_CPIX]          ' and Compensation Pixel, too

PUB GetLine(ptr_line, line) | rawpix[8], col, pixel

    if not lookdown(line: 0..3)
        return

    readData (@rawpix, line, 4, 16)

    repeat col from 0 to 15
        pixel := (col * 4) + line
        word[ptr_line][pixel] := type.u16_s16 (rawpix.word[col])

PUB GetPixel(ptr_frame, col, line) | rawpix, pixel

    if not lookdown(col: 0..15) or not lookdown(line: 0..3)
        return

    pixel := (col * 4) + line 'Compute offset location in array of current pixel

    readData (@rawpix, pixel, 0, 1)

    word[ptr_frame][pixel] := type.u16_s16 (rawpix & $FFFF)

    return ptr_frame.word[pixel] := rawpix

PUB ADCReference(mode) | tmp
' Set ADC reference high, low
'   Valid values:
'      ADCREF_HI (0) - ADC High reference enabled
'      ADCREF_LO (1) - ADC Low reference enabled (default)
'   Any other value polls the chip and returns the current setting
' NOTE: Re-calibration must be done after this method is called
    readRegX (core#CONFIG, 1, 0, @tmp)
    case mode
        ADCREF_HI, ADCREF_LO:
            mode := mode << core#FLD_ADCHIGHREF
        OTHER:
            return (tmp >> core#FLD_ADCHIGHREF) & %1
    tmp &= core#MASK_ADCHIGHREF
    tmp := (tmp | mode) & core#CONFIG_MASK
    writeRegX (core#CONFIG, tmp)

'TODO: Call Re-cal method here

PUB ADCRes(bits) | tmp
' Set ADC resolution, in bits
'   Valid values: 15..18
'   Any other value polls the chip and returns the current setting
    readRegX (core#CONFIG, 1, 0, @tmp)
    case bits
        15..18:
            bits := lookdownz(bits: 15, 16, 17, 18) << core#FLD_ADCRES
        OTHER:
            result := (tmp >> core#FLD_ADCRES) & core#BITS_ADCRES
            return result := lookupz(result: 15, 16, 17, 18)
    tmp &= core#MASK_ADCRES
    tmp := (tmp | bits) & core#CONFIG_MASK
    writeRegX (core#CONFIG, tmp)

'    _adc_res := 3-((_cfg_reg >> 4) & %11)   'Update the VAR used in calculations

PUB MeasureMode(mode) | tmp
' Set measurement mode to continuous or one-shot/step
'   Valid values:
'      MMODE_CONT (0) - Continuous (default)
'      MMODE_STEP (1) - Step
'   Any other value polls the chip and returns the current setting
    readRegX (core#CONFIG, 1, 0, @tmp)

    case mode
        MMODE_CONT, MMODE_STEP:
            mode := (mode << core#FLD_MEASMODE)
        OTHER:
            return (tmp >> core#FLD_MEASMODE) & %1
    tmp &= core#MASK_MEASMODE
    tmp := (tmp | mode) & core#CONFIG_MASK
    writeRegX ( core#CONFIG, tmp)

PUB OperationMode(mode) | tmp
' Set Operation mode
'   Valid values:
'       OPMODE_NORM (0) - Normal (default)
'       OPMODE_SLEEP (1) - Sleep mode
'   Any other value polls the chip and returns the current setting
    readRegX (core#CONFIG, 1, 0, @tmp)
    case mode
        OPMODE_NORM, OPMODE_SLEEP:
            mode := (mode << core#FLD_OPMODE)
        OTHER:
            return (tmp >> core#FLD_OPMODE) & %1
    tmp &= core#MASK_OPMODE
    tmp := (tmp | mode) & core#CONFIG_MASK
    writeRegX (core#CONFIG, tmp)

PUB RefreshRate(Hz) | tmp
' Set sensor refresh rate
'   Valid values are 0, for 0.5Hz, or 1 to 512 in powers of 2
'   Any other value polls the chip and returns the current setting
' NOTE: Higher rates will yield noisier images
    readRegX (core#CONFIG, 1, 0, @tmp)
    case Hz := lookdownz(Hz: 512, 512, 512, 512, 512, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0)
        0..15:
        OTHER:
            result := tmp & core#BITS_REFRATE
            return lookupz(result: 512, 512, 512, 512, 512, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0)

    tmp &= core#MASK_REFRATE
    tmp := (tmp | Hz) & core#CONFIG_MASK
    writeRegX (core#CONFIG, tmp)

PUB Dump_EE(ee_buf)
'' Copy downloaded EEPROM image to ee_buf
'' NOTE: Make sure ee_buf is 256 bytes in size!
    bytemove(ee_buf, @_ee_data, EE_SIZE-1)

PUB Peek_EE(location)
'' Return byte at 'location' in EEPROM memory
    return _ee_data.byte[location]

PUB Read_EE
'' Read EEPROM contents into RAM
    bytefill (@_ee_data, $00, EE_SIZE)  'Make sure data in RAM copy of EEPROM image is clear

    i2c.start                           'Start reading at addr $00
    i2c.write (core#EE_SLAVE_ADDR)
    i2c.write ($00)

    i2c.start                           'Read in the EEPROM
    i2c.write (core#EE_SLAVE_ADDR|1)
    i2c.rd_block (@_ee_data, EE_SIZE-1, TRUE)
    i2c.stop

PUB OSCTrim(val) | tmp
' Set Oscillator Trim value
'   Valid values: 0..127
'   Any other value polls the chip and returns the current setting
'   NOTE: It is recommended to use the factory set value contained in the device's EEPROM.
    readRegX (core#OSC_TRIM, 1, 0, @tmp)
    case val
        0..127:
        OTHER:
            return val & core#OSC_TRIM_MASK

    tmp := val & core#OSC_TRIM_MASK
    writeRegX (core#OSC_TRIM, tmp)

PUB readRegX(reg, nr_reads, rd_step, rd_buf) | cmd_packet[2]

    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_READREG
    case reg
        $00..$41:   'RAM
            cmd_packet.byte[2] := reg
            cmd_packet.byte[3] := rd_step
            cmd_packet.byte[4] := nr_reads

        $92..$93:   'Configuration regs
            cmd_packet.byte[2] := reg
            cmd_packet.byte[3] := 0     'Address step
            cmd_packet.byte[4] := 1     'Number of reads

        OTHER:
            return

    i2c.start
    i2c.wr_block (@cmd_packet, 5)
    i2c.start
    i2c.write (SLAVE_RD)
    i2c.rd_block (rd_buf, nr_reads, TRUE)
    i2c.stop

PUB writeRegX(reg, val) | cmd_packet[2], nr_bytes

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

PUB readData(buff_ptr, addr_start, addr_step, word_count) | cmd_packet[2]
'XXXPRI
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
