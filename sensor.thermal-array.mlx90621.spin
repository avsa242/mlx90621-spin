{
    --------------------------------------------
    Filename: sensor.thermal-array.mlx90621.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90621 16x4 IR array
    Copyright (c) 2022
    Started: Jan 4, 2018
    Updated: Sep 22, 2022
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR        = core#SLAVE_ADDR
    SLAVE_RD        = core#SLAVE_ADDR|1

    DEF_HZ          = 100_000
    I2C_MAX_FREQ    = core#I2C_MAX_FREQ

    { image dimensions }
    WIDTH           = 16
    HEIGHT          = 4
    XMAX            = WIDTH-1
    YMAX            = HEIGHT-1

    { I2C Fast Mode+ }
    I2CFMODE_ENA    = 0
    I2CFMODE_DIS    = 1

    { Operation modes }
    CONT            = 0
    SINGLE          = 1

    { Sensor power states }
    OFF             = 0
    ON              = 1

    { ADC reference settings }
    ADCREF_HI       = 0
    ADCREF_LO       = 1

    { On-sensor EEPROM }
    EE_SIZE         = 256
    EE_ENA          = 0
    EE_DIS          = 1

    { Offsets to sensor calibration data }
    EE_KS4          = $9E                       ' s8
    EE_DELALPH      = $A2                       ' 
    EE_KS_SCL       = $C0                       ' u4
    EE_KS4EE        = $C4                       ' s8
    EE_ACOM         = $D0                       ' s16
    EE_ACP          = $D3                       ' s16
    EE_BCP          = $D5                       ' s8
    EE_ALPHCP       = $D6                       ' u16
    EE_TGC          = $D8                       ' s8
    EE_AISCL        = $D9                       ' [7:4], u4
    EE_BISCL        = $D9                       ' [3:0], u4
    EE_VTH25        = $DA                       ' s16
    EE_KT1          = $DC                       ' s16
    EE_KT2          = $DE                       ' s16
    EE_KT1SCL       = $D2                       ' [7:4], u4
    EE_KT2SCL       = $D2                       ' [3:0], u4
    EE_ALPH0        = $E0                       ' u16
    EE_ALPH0SCL     = $E2                       ' u8
    EE_DELALSCL     = $E3                       ' u8
    EE_EMIS         = $E4                       ' u16
    EE_KSTA         = $E6                       ' s16
    EE_OSCTRIM      = $F7                       ' u7
    EE_CFG          = $F5                       ' u16

    TA0             = 25_00
    TWO20           = 1 << 20                   ' 2^20

    RAM_PTAT        = $40
    RAM_COMPPIX     = $41

    CFG_CKBYTE      = $55
    OSC_CKBYTE      = $AA

    W               = 0
    R               = 1

    { u64 math }
    H               = 0
    L               = 1

OBJ

    core: "core.con.mlx90621"                   ' HW-specific constants
    i2c : "com.i2c"                             ' PASM I2C engine
    time: "time"                                ' timekeeping methods
    u64 : "math.unsigned64"                     ' unsigned 64-bit math routines

VAR

    word _ptat
    byte _ee_data[EE_SIZE]

    long _res, _kt1scl, _kt2scl, _vth25, _kt1, _kt2, _adcres_bits

PUB null{}
' This is not a top-level object

PUB startx(SCL_PIN, SDA_PIN, I2C_HZ): status
' Start using custom I/O settings
    if lookdown(SCL_PIN: 0..63) and lookdown(SDA_PIN: 0..63) and {
}   I2C_HZ =< core#I2C_MAX_FREQ
        if (status := i2c.init(SCL_PIN, SDA_PIN, core#EE_MAX_FREQ))
            time.usleep(core#T_POR)
            if i2c.present(core#EE_SLAVE_ADDR)  ' first start I2C engine
                if readeeprom{}                 '   to read the EEPROM
                    time.msleep(5)
                    i2c.deinit{}
                                                ' now re-setup for the sensor
                    i2c.init(SCL_PIN, SDA_PIN, I2C_HZ)
                    time.msleep(5)
                    if i2c.present(SLAVE_WR)    ' check device bus presence
                        return
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

PUB stop{}
' Stop the driver
    i2c.deinit{}
    longfill(@_res, 0, 7)
    bytefill(@_ee_data, 0, EE_SIZE)
    _ptat := 0

PUB defaults{}
' Write osc trimming val extracted from EEPROM address $F7
    osctrim(_ee_data[EE_OSCTRIM])
    refreshrate(1)
    adcres(18)
    opmode(CONT)
    powered(TRUE)
    i2cfm(TRUE)
    eeprom(TRUE)
    adcreference(ADCREF_LO)
    reset(TRUE)
    time.msleep(5)

PUB adcreference(mode): curr_mode
' Set ADC reference high, low
'   Valid values:
'      ADCREF_HI (0) - ADC High reference enabled
'     *ADCREF_LO (1) - ADC Low reference enabled
'   Any other value polls the chip and returns the current setting
' NOTE: Re-calibration must be done after this method is called
    readreg(core#CONFIG, 2, 0, @curr_mode)
    case mode
        ADCREF_HI, ADCREF_LO:
            mode := mode << core#ADCHIGHREF
        other:
            return (curr_mode >> core#ADCHIGHREF) & 1

    mode := ((curr_mode & core#ADCHIGHREF_MASK) | mode)
    writereg(core#CONFIG, mode)

'TODO: Call Re-cal method here

PUB adcres(bits): curr_res
' Set ADC resolution, in bits
'   Valid values: 15..18 (default: 18)
'   Any other value polls the chip and returns the current setting
    readreg(core#CONFIG, 2, 0, @curr_res)
    case bits
        15..18:
            _adcres_bits := (1 << (3-lookdownz(bits: 15, 16, 17, 18)) )
            bits := lookdownz(bits: 15, 16, 17, 18) << core#ADCRES
        other:
            curr_res := (curr_res >> core#ADCRES) & core#ADCRES_BITS
            return lookupz(curr_res: 15, 16, 17, 18)

    bits := ((curr_res & core#ADCRES_MASK) | bits)
    writereg(core#CONFIG, bits)

PUB ambienttemp{}: ta | ptat, kt1, kt2, kt1scl, kt2scl, vth25, t1_64[2], t1_32, t2_64[2], t2_32, t3, t3sign
' Read Proportional To Ambient Temperature sensor
'   Returns: temperature, in hundredths of a degree Celsius
    ptat := 0
    readreg(core#PTAT, 1, 0, @ptat)

    { gather coefficients from EEPROM image }
    vth25 := _vth25
    kt1 := _kt1
    kt2 := _kt2
    kt1scl := _kt1scl
    kt2scl := _kt2scl

    { scale Vth(25) down according to the current ADC resolution }
    vth25 /= _adcres_bits

    { scale down Kt1 and Kt2 using the EEPROM coefficients }
    kt1 := (kt1 * 1_000) / (kt1scl * _adcres_bits)
    kt2 := u64.multdiv(kt2, 1000000, (kt2scl * _adcres_bits))

    { Ta = ( (-Kt1 + sqrt(Kt1^2 - 4Kt2 * (Vth(25) - PTAT)) ) / 2Kt2 ) + 25 }

    u64.mult(@t1_64, kt1, kt1)              ' Kt1 ^ 2

    t2_32 := (4 * kt2)                      ' 4KT2

    t3 := (vth25 - ptat)                    ' (Vth(25) - PTAT)
    if (t3 < 0)                             ' preserve sign for u64 math below
        t3sign := -1
    else
        t3sign := 1

    u64.mult(@t2_64, t2_32, ||(t3))         ' u64: 4Kt2 * abs(Vth(25) - PTAT)

    if (t3sign == -1)
        u64.dadd(@t1_64, t2_64[H], t2_64[L])' Kt1^2 - (Vth(25) - PTAT)
    else
        u64.dsub(@t1_64, t2_64[H], t2_64[L])
    t1_32 := u64.div(t1_64[H], t1_64[L], 1_00)

    t2_32 := ^^(t1_32) * 10                 ' sqrt(Kt1^2 - 4Kt2 * (Vth(25) - PTAT))

    t3 := (-kt1 + t2_32)                    ' -Kt1 + sqrt(Kt1^2 - 4Kt2 * (Vth(25) - PTAT))

    t2_32 := (kt2 * 2)                      ' 2Kt2
    return u64.multdiv(t3, 100000, t2_32) + 25_00

PUB dump_ee(ptr_buff)
' Copy downloaded EEPROM image to ptr_buff
' NOTE: This buffer must be at least 256 bytes
    bytemove(ptr_buff, @_ee_data, EE_SIZE)

PUB eeprom(state): curr_state
' Enable/disable the sensor's built-in EEPROM
'   Valid values:
'      TRUE (-1 or 1): Sensor's built-in EEPROM enabled (default)
'      FALSE: Sensor's built-in EEPROM disabled
'   Any other value polls the chip and returns the current setting
'   NOTE: Use with care! Driver will fail to restart if EEPROM is disabled.
'       Cycle power in this case.
    readreg(core#CONFIG, 2, 0, @curr_state)
    case ||(state)
        0, 1:
            state := (1-(||(state))) << core#EEPROMENA
        other:
            return (1-((curr_state >> core#EEPROMENA) & 1)) == 1

    state := ((curr_state & core#EEPROMENA_MASK) | state)
    writereg(core#CONFIG, state)

PUB getcolumn(ptr_buff, col) | tmpframe[2], tmp, offs, line
' Read a single column of pixels from the sensor into ptr_buff
'   NOTE This buffer must be at least 4 longs
    if not lookdown(col: 0..15)
        return

    readreg(col * 4, 4, 1, @tmpframe)
    repeat line from 0 to YMAX
        offs := (col * 4) + line
        long[ptr_buff][tmp] := ~~tmpframe.word[offs]

PUB getframe(ptr_buff) | tmpframe[32], offs
' Read entire frame from sensor and store it in buffer at ptr_buff
'   NOTE: This buffer must be at least 64 longs
    readreg(0, 64, 1, @tmpframe)
    repeat offs from 0 to 63
        long[ptr_buff][offs] := ~~tmpframe.word[offs]

PUB getframeext(ptr_buff) | tmpframe[33], offs, line, col
' Read entire frame, as well as PTAT and compensation pixel data from sensor and stores it in buffer at ptr_buff
'   NOTE: This buffer must be at least 66 longs
    readreg(0, 66, 1, @tmpframe)
    repeat offs from 0 to 65
        long[ptr_buff][offs] := ~~tmpframe.word[offs]
    _PTAT := tmpframe[RAM_OFFS_PTAT]            ' Get PTAT data

PUB getline(ptr_buff, line) | tmpframe[8], offs, col
' Read a single line of pixels from the sensor into ptr_buff
'   NOTE: This buffer must be at least 16 longs
    if not lookdown(line: 0..3)
        return
    readreg(line, 16, 4, @tmpframe)
    repeat col from 0 to XMAX
        offs := (col * 4) + line
        long[ptr_buff][offs] := ~~tmpframe.word[offs]

PUB getpixel(ptr_buff, col, line): pix_word | tmpframe, offs
' Read a single pixel from the sensor into ptr_buff
'   Returns: pixel value
'   NOTE: This buffer must be at least 1 long
    case col
        0..XMAX:
        other:
            return

    case line
        0..YMAX:
        other:
            return

    offs := (col * 4) + line                    ' offset within array image

    readreg(offs, 1, 0, @tmpframe)
    long[ptr_buff][offs] := pix_word := ~~tmpframe.word[0]

PUB measure{}
' Perform measurement, when OpMode is set to SINGLE
'   NOTE: This method waits/blocks while a measurement is ongoing
    writereg(core#CMD_STEP_MEASURE, 0)
    repeat while measuring{}

PUB measuring{}: flag
' Flag indicating a measurement is running
'   Returns:
'       FALSE (0): No IR measurement running
'       TRUE (-1): IR measurement running
'   NOTE: This method is intended for use when MeasureMode is set to SINGLE
    readreg(core#CONFIG, 2, 0, @flag)
    return ((flag >> core#MEASURING) & 1) == 1

PUB opmode(mode): curr_mode
' Set measurement mode
'   Valid values:
'     *CONT (0) - Continuous measurement mode
'      SINGLE (1) - Single-measurement mode only
'   Any other value polls the chip and returns the current setting
'   NOTE: In SINGLE mode, measurements must be triggered manually using the Measure method
    readreg(core#CONFIG, 2, 0, @curr_mode)
    case mode
        CONT, SINGLE:
            mode := (mode << core#MEASMODE)
        other:
            return (curr_mode >> core#MEASMODE) & 1

    mode := ((curr_mode & core#MEASMODE_MASK) | mode)
    writereg(core#CONFIG, mode)

PUB osctrim(val): curr_val
' Set Oscillator Trim value
'   Valid values: 0..127 (default: 0)
'   Any other value polls the chip and returns the current setting
'   NOTE: It is recommended to use the factory set value contained in the device's EEPROM.
    case val
        0..127:
            val &= core#OSC_TRIM_MASK
            writereg(core#OSC_TRIM, curr_val)
        other:
            readreg(core#OSC_TRIM, 1, 0, @curr_val)
            return curr_val & core#OSC_TRIM_MASK

PUB powered(state): curr_state
' Power on sensor
'   Valid values:
'      *FALSE (0) - Sleep
'       TRUE (-1 or 1) - Powered on/normal operation
'   Any other value polls the chip and returns the current setting
    readreg(core#CONFIG, 2, 0, @curr_state)
    case ||(state)
        0, 1:
            ' chip logic is reversed, so flip the bit
            state := (||(state) ^ 1) << core#OPMODE
        other:
            return (((curr_state >> core#OPMODE) & 1) ^ 1)

    state := ((curr_state & core#OPMODE_MASK) | state)
    writereg(core#CONFIG, state)

PUB readeeprom{}: status | ackbit, tries
' Read sensor EEPROM contents into RAM
'   Returns:
'       TRUE (-1): success
'       FALSE (0): failure
    bytefill(@_ee_data, 0, EE_SIZE)             ' clear RAM copy of EEPROM
    tries := 0

    i2c.start{}                                 ' try to talk to the EEPROM
    repeat
        ackbit := i2c.write(core#EE_SLAVE_ADDR)
        if (++tries > 3)                        ' give up after 3 tries
            i2c.stop{}
            return FALSE
    until (ackbit == i2c#ACK)
    i2c.write($00)

    i2c.start{}                                 ' Read in the EEPROM
    repeat
        ackbit := i2c.write(core#EE_SLAVE_ADDR|1)
        if (++tries > 3)
            i2c.stop{}
            return FALSE
    until (ackbit == i2c#ACK)
    i2c.rdblock_lsbf(@_ee_data, EE_SIZE, i2c#NAK)
    i2c.stop{}

    _adcres_bits := (1 << (3-lookdownz(adcres(-2): 15, 16, 17, 18)) )

    bytemove(@_vth25, @_ee_data+EE_VTH25, 2)
    bytemove(@_kt1, @_ee_data+EE_KT1, 2)
    bytemove(@_kt1scl, @_ee_data+EE_KT1SCL, 1)
    bytemove(@_kt2, @_ee_data+EE_KT2, 2)
    bytemove(@_kt2scl, @_ee_data+EE_KT2SCL, 1)

    _kt1scl := 1 << ((_kt1scl & $f0) >> 4)
    _kt2scl := ( 1 << ((_kt2scl & $0f) + 10) )

    ~~_kt1
    ~~_kt2

    return TRUE

PUB refreshrate(rate): curr_rate
' Set sensor refresh rate
'   Valid values are 0, for 0.5Hz, or 1 to 512 in powers of 2 (default: 1)
'   Any other value polls the chip and returns the current setting
'   NOTE: Higher rates will yield noisier images
    readreg(core#CONFIG, 2, 0, @curr_rate)
    case rate
        512, 256, 128, 64, 32, 16, 8, 4, 2, 1, 0:
            rate := lookdownz(rate: 512, 512, 512, 512, 512, 512, 256, 128,{
}           64, 32, 16, 8, 4, 2, 1, 0)
        other:
            curr_rate &= core#REFRATE_BITS
            return lookupz(curr_rate: 512, 512, 512, 512, 512, 512, 256, 128,{
}           64, 32, 16, 8, 4, 2, 1, 0)

    rate := ((curr_rate & core#REFRATE_MASK) | rate) & core#CONFIG_MASK
    writereg(core#CONFIG, rate)

PUB reset(set): flag    ' XXX should be renamed - doesn't do what 'Reset()' normally does
' Set sensor reset flag
'   Valid values: TRUE (-1 or 1)
'   Any other value polls the chip and returns the current setting
'   NOTE: This must be done any time the sensor is initialized, _after_ the configuration register has been updated
'       If FALSE is returned, POR or brown-out has occurred and the process must be repeated
    readreg(core#CONFIG, 2, 0, @flag)
    case ||(set)
        1:
            set := 1 << core#RESET
        other:
            return (((flag >> core#RESET) & 1) == 1)

    set := ((flag & core#RESET_MASK) | set)
    writereg(core#CONFIG, set)

PRI i2cfm(mode): curr_mode
' Enable I2C Fast Mode+
'   Valid values:
'     *TRUE (-1 or 1): Max I2C bus speed 1000kbit/sec
'      FALSE (0): Max I2C bus speed 400kbit/sec
'   NOTE: This is independent of, and has no effect on what speed the driver
'   was started with.
'   Any other value polls the chip and returns the current setting
    readreg(core#CONFIG, 2, 0, @curr_mode)
    case ||(mode)
        0, 1:
            mode := (1-(||(mode))) << core#I2CFMP
        other:
            return (1-((curr_mode >> core#I2CFMP) & 1)) == 1

    mode := ((curr_mode & core#I2CFMP_MASK) | mode)
    writereg(core#CONFIG, curr_mode)

PRI readreg(reg_nr, nr_reads, rd_step, ptr_buff) | cmd_pkt[2]
' Read nr_reads from device into ptr_buff
    cmd_pkt.byte[0] := SLAVE_WR
    cmd_pkt.byte[1] := core#CMD_READREG
    case reg_nr
        $00..$41:                               ' RAM
            cmd_pkt.byte[2] := reg_nr
            cmd_pkt.byte[3] := rd_step
            cmd_pkt.byte[4] := nr_reads
            nr_reads <<= 1
        $92..$93:                               ' Configuration regs
            cmd_pkt.byte[2] := reg_nr
            cmd_pkt.byte[3] := 0                ' Address step
            cmd_pkt.byte[4] := 1                ' Number of reads
            nr_reads := 2
        other:
            return

    i2c.start{}
    i2c.wrblock_lsbf(@cmd_pkt, 5)

    i2c.start{}
    i2c.write(SLAVE_RD)
    i2c.rdblock_lsbf(ptr_buff, nr_reads, i2c#NAK)
    i2c.stop{}

PRI writereg(reg_nr, val) | cmd_pkt[2], nr_bytes
' Write val to device
    cmd_pkt.byte[0] := SLAVE_WR
    case reg_nr
        core#CONFIG:
            cmd_pkt.byte[1] := core#CMD_WRITEREG_CFG
            cmd_pkt.byte[2] := val.byte[0] - CFG_CKBYTE
            cmd_pkt.byte[3] := val.byte[0]
            cmd_pkt.byte[4] := val.byte[1] - CFG_CKBYTE
            cmd_pkt.byte[5] := val.byte[1]
            nr_bytes := 6
        core#OSC_TRIM:
            cmd_pkt.byte[1] := core#CMD_WRITEREG_OSCTRIM
            cmd_pkt.byte[2] := val.byte[0] - OSC_CKBYTE
            cmd_pkt.byte[3] := val.byte[0]
            cmd_pkt.byte[4] := val.byte[1] - OSC_CKBYTE
            cmd_pkt.byte[5] := val.byte[1]
            nr_bytes := 6
        core#CMD_STEP_MEASURE:
            cmd_pkt.byte[1] := core#CMD_STEP_MEASURE & $FF
            cmd_pkt.byte[2] := (core#CMD_STEP_MEASURE >> 8) & $FF
            nr_bytes := 3

        other:
            return

    i2c.start{}
    i2c.wrblock_lsbf(@cmd_pkt, nr_bytes)
    i2c.stop{}

DAT
{
Copyright 2022 Jesse Burt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
}

