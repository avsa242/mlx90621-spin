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

OBJ

    core    : "core.con.mlx90621"
    i2c     : "jm_i2c_fast"
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

    Write_OSCTrim (peek_ee (EE_OFFS_OSCTRIM)) ' Write osc trimming val extracted from EEPROM address $F7
'    _cfg_reg := (peek_ee(EE_OFFS_CFGH) << 8) | peek_ee(EE_OFFS_CFGL) & $FFFF
    RefreshRate (1)
    ADCRes (18)
    MeasureMode (MMODE_CONT)
    OperationMode (OPMODE_NORM) '0019 XXX OPMODE_SLEEP unverified (returns 463E)
    I2CFM (TRUE)
    EEPROM (TRUE)
    ADCReference (ADCREF_LO)
    Write_Cfg

    time.MSleep (5)

    Read_Cfg
    Read_OSCTrim

PUB EEPROM(enabled)
'' Enable/disable the sensor's built-in EEPROM
''   TRUE, 1 - Sensor's built-in EEPROM enabled (default)
''   FALSE, 0- Sensor's built-in EEPROM disabled (use with care -
''               object will fail to restart if EEPROM is disabled.
''               Cycle power in this case.)
    Read_Cfg
    case ||enabled
        0, 1:
            _cfgset_ee_ena := (1-||enabled) << 12
        OTHER:
            return

    Write_Cfg

PUB I2CFM(enabled)
'' Enable/disable I2C Fast Mode mode
''   TRUE, 1  - Max I2C bus speed/bit transfer rate up to 1000kbit/sec (default)
''   FALSE, 0 - Max I2C bus speed/bit transfer rate up to 400kbit/sec
    Read_Cfg
    case ||enabled
        0, 1:
            _cfgset_i2cfm_ena := (1-||enabled) << 11
        OTHER:
            return

    Write_Cfg

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

PUB Read_Cfg | tmp 'XXX Remove? Just use readData instead?

    readData (@tmp, core#REG_CFG, 0, 1)
    
    _cfg_reg := (tmp.byte[1] << 8) | tmp.byte[0]

    _cfgset_adcref := _cfg_reg &       %0_1_0_0_0_0_0_0_0_0_00_0000
    _cfgset_ee_ena := _cfg_reg &       %0_0_0_1_0_0_0_0_0_0_00_0000
    _cfgset_i2cfm_ena := _cfg_reg &    %0_0_0_0_1_0_0_0_0_0_00_0000
    _cfgflag_por_bit := _cfg_reg &     %0_0_0_0_0_1_0_0_0_0_00_0000
    _cfgflag_measuring := _cfg_reg &   %0_0_0_0_0_0_1_0_0_0_00_0000
    _cfgset_opmode := _cfg_reg &       %0_0_0_0_0_0_0_0_1_0_00_0000
    _cfgset_mmode := _cfg_reg &        %0_0_0_0_0_0_0_0_0_1_00_0000
    _cfgset_adcres := _cfg_reg &       %0_0_0_0_0_0_0_0_0_0_11_0000
    _cfgset_refr_rate := _cfg_reg &    %0_0_0_0_0_0_0_0_0_0_00_1111

    return _cfg_reg

PUB Read_OSCTrim | read_data, osctrim_data

    readData (@read_data, core#REG_OSC, 0, 1)
    osctrim_data := (read_data.byte[1] << 8) | read_data.byte[0]

PUB ADCReference(mode)
'' Set ADC reference high, low
''   ADCREF_HI (0) - ADC High reference enabled
''   ADCREF_LO (1) - ADC Low reference enabled (default)
'' NOTE: Re-cal must be done after this method is called
    Read_Cfg
    case mode
        ADCREF_HI, ADCREF_LO:
            _cfgset_adcref := (mode << 14)
        OTHER:
            return

    Write_Cfg
  'TODO: Call Re-cal method here

PUB ADCRes(bits)
'' Set ADC resolution, in bits
''  Valid values are 15 to 18
'' NOTE: Updates the VAR _adc_res, as this is used in the various thermal correction calculations
'' TODO:
''   Create a wrapper re-cal method
    Read_Cfg
    case lookdown(bits: 15..18)
        1..4:
            _cfgset_adcres := (bits-15) << 4
        OTHER:
            return

    Write_Cfg
    _adc_res := 3-((_cfg_reg >> 4) & %11)   'Update the VAR used in calculations

PUB MeasureMode(mode)
'' Set measurement mode
''   MMODE_CONT (0) - Continuous (default)
''   MMODE_STEP (1) - Step
    Read_Cfg
    case mode
        MMODE_CONT, MMODE_STEP:
            _cfgset_mmode := (mode << 6)
        OTHER:
            return

    Write_Cfg

PUB OperationMode(mode)
'' Set Operation mode
''   OPMODE_NORM (0) - Normal (default)
''   OPMODE_SLEEP (1) - Sleep mode
    Read_Cfg
    case mode
        OPMODE_NORM, OPMODE_SLEEP:
            _cfgset_opmode := (mode << 7)
        OTHER:
            return

    Write_Cfg

PUB RefreshRate(Hz)
'' Set sensor refresh rate
'' Valid values are 0, 0.5 or 5 for 0.5Hz, or 1 to 512 in powers of 2
'' NOTE: Higher rates will yield noisier images
    Read_Cfg    'XXX Convert to lookup?
    case Hz
        0, 0.5, 5:
            _cfgset_refr_rate := %1111
        1:
            _cfgset_refr_rate := %1110
        2:
            _cfgset_refr_rate := %1101
        4:
            _cfgset_refr_rate := %1100
        8:
            _cfgset_refr_rate := %1011
        16:
            _cfgset_refr_rate := %1010
        32:
            _cfgset_refr_rate := %1001
        64:
            _cfgset_refr_rate := %1000
        128:
            _cfgset_refr_rate := %0111
        256:
            _cfgset_refr_rate := %0110
        512:
            _cfgset_refr_rate := %0101
        OTHER:
            return

    Write_Cfg

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
    i2c.pread (@_ee_data, EE_SIZE-1, TRUE)
    i2c.stop

PUB Write_Cfg | lsbyte, lsbyte_ck, msbyte, msbyte_ck, cmd_packet[2]
'' Write config register (automatically called from methods that modify fields within the register)
    _cfg_reg := _cfgset_adcref | _cfgset_ee_ena | _cfgset_i2cfm_ena | _cfgset_opmode | _cfgset_mmode | _cfgset_adcres | _cfgset_refr_rate | POR_BIT

    lsbyte := _cfg_reg.byte[0]
    msbyte := _cfg_reg.byte[1]

    lsbyte_ck := (lsbyte - CFG_CKBYTE)          'Generate simple checksum values
    msbyte_ck := (msbyte - CFG_CKBYTE)          'from least and most significant bytes

    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_WRITEREG_CFG
    cmd_packet.byte[2] := lsbyte_ck
    cmd_packet.byte[3] := lsbyte
    cmd_packet.byte[4] := msbyte_ck
    cmd_packet.byte[5] := msbyte

    i2c.start
    i2c.pwrite (@cmd_packet, 6)
    i2c.stop

    return (msbyte<<8)|lsbyte

PUB Write_OSCTrim(val_word) | lsbyte, lsbyte_ck, msbyte, msbyte_ck, cmd_packet[2]
'' Write oscillator trimming register
'' NOTE: Value typically used is stored in the EEPROM image at offset EE_OFFS_OSCTRIM
''  and varies per device.
    lsbyte := val_word.byte[0]
    msbyte := val_word.byte[1]
    lsbyte_ck := (lsbyte - OSC_CKBYTE)           'Generate simple checksum values
    msbyte_ck := (msbyte - OSC_CKBYTE)           'from least and most significant bytes

    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_WRITEREG_OSCTRIM
    cmd_packet.byte[2] := lsbyte_ck
    cmd_packet.byte[3] := lsbyte
    cmd_packet.byte[4] := msbyte_ck
    cmd_packet.byte[5] := msbyte

    i2c.start
    i2c.pwrite (@cmd_packet, 6)
    i2c.stop

PUB readData(buff_ptr, addr_start, addr_step, word_count) | cmd_packet[2]
'XXXPRI
    cmd_packet.byte[0] := SLAVE_WR
    cmd_packet.byte[1] := core#CMD_READREG
    cmd_packet.byte[2] := addr_start
    cmd_packet.byte[3] := addr_step
    cmd_packet.byte[4] := word_count
    
    i2c.start
    i2c.pwrite (@cmd_packet, 5)
    i2c.start
    i2c.write (SLAVE_RD)
    i2c.pread (buff_ptr, word_count << 1, TRUE) '*2 = 81.6uS, << 1 = 71.8uS
    i2c.stop

{PUB command (cmd, addr_start, addr_step, num_reads) | cmd_packet[2], ackbit

  cmd_packet.byte[0] := SLAVE_WR
  cmd_packet.byte[1] := cmd
  cmd_packet.byte[2] := addr_start
  cmd_packet.byte[3] := addr_step
  cmd_packet.byte[4] := num_reads

  i2c.start
  ackbit := i2c.pwrite(@cmd_packet, 5)
  if ackbit == i2c#NAK
    _nak_count++
}
{PRI readword: data_word | read_data

  i2c.start
  _ackbit := i2c.write (SLAVE_RD)
  i2c.pread (@read_data, 2, TRUE)
  i2c.stop

  data_word := (read_data.byte[1] << 8) | read_data.byte[0]
}
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
