{
    --------------------------------------------
    Filename: sensor.thermal.array.mlx90621.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90621
     16x4 IR array (I2C).
    Copyright (c) 2018
    Started: Jan 14, 2018
    Updated: Nov 11, 2018
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR          = core#SLAVE_ADDR
    SLAVE_RD          = core#SLAVE_ADDR|1

    DEF_SCL           = 28
    DEF_SDA           = 29
    DEF_HZ            = 400_000
    I2C_MAX_FREQ      = core#I2C_MAX_FREQ

    EE_SIZE           = 256

    I2CFMODE_ENA      = 0
    I2CFMODE_DIS      = 1

    MMODE_CONT        = 0
    MMODE_STEP        = 1

    OPMODE_NORM       = 0
    OPMODE_SLEEP      = 1

    ADCREF_HI         = 0
    ADCREF_LO         = 1

    EE_ENA            = 0
    EE_DIS            = 1

    EE_OFFS_VTH       = $DA
    EE_OFFS_KT1       = $DC
    EE_OFFS_KT2       = $DE
    EE_OFFS_KT1SCL    = $D2
    EE_OFFS_KT2SCL    = $D2

    RAM_OFFS_PTAT     = $40
    RAM_OFFS_CPIX     = $41

OBJ

    core    : "core.con.mlx90621"
    i2c     : "jm_i2c_fast"
    time    : "time"
    type    : "system.types"

VAR

  byte _ee_data[EE_SIZE]
  byte _osc_trim
  byte _ackbit
  long _nak
  word _ir_frame[64]
  word _cfg_reg
  word _comp_pix
  byte _adc_res
  word _Vir_comp

  long _Vth_h, _Vth_l, _Vth25, _Kt1 , _Kt1_h, _Kt1_l, _Kt1_Scl, _Kt2, _Kt2_h, _Kt2_l, _Kt2_Scl, KtScl
  word _PTAT

  long _nak_count 'temporary

PUB Null
''This is not a top-level object

PUB Start: okay                                                 'Default to "standard" Propeller I2C pins and 400kHz

    okay := Startx (DEF_SCL, DEF_SDA, DEF_HZ)

PUB Startx(SCL_PIN, SDA_PIN, I2C_HZ): okay

    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31)
        if I2C_HZ =< core#I2C_MAX_FREQ
            if okay := i2c.setupx (SCL_PIN, SDA_PIN, core#EE_MAX_FREQ)  'First, start the I2C object...
                time.MSleep (3)
                i2c.start
                okay := i2c.write(core#EE_SLAVE_ADDR)
                i2c.stop
                if (okay == i2c#ACK)
                    read_ee(@_ee_data)                                  '...to read the EEPROM.
                else
                    return FALSE
                
                i2c.terminate                                           'Shut it down.
                
                ifnot okay := i2c.setupx (SCL_PIN, SDA_PIN, I2C_HZ)     'Then start it back up,
                      return FALSE                                      ' but this time setup for the sensor
                time.MSleep (5)

                if Ping                                                 'Check for response from device
                    return okay

    return FALSE                                                        'If we got here, something went wrong

''  TASKS:
''C  1 POR (see 8.3)
''C  2 wait 5ms
''C  3 read eeprom table
''W  4 store cal coeff in prop RAM
''C  5 write osc trim val into addr $93
''C  6 write cfg val addr $92 (value read from eeprom or hard coded externally)
''C    set POR/brown out flag to 1 (bit 10 at $92
''  7 check BO flag. Cleared? Yes: repeat step 3 No: proceed to step 8
''W  8 read meas data (PTAT+desired IR data)
''W  9 Ta calc
''W  10 pix offset cancelling
''  11 therm grad comp
''  12 pix<->pix normalization
''  13 obj emissivity comp
''  14 obj temp calc
''  15 image process/correct
''  loop to step 7

PUB Stop

  i2c.terminate

PUB Ping
'' "Pings" device and returns TRUE if present
    i2c.start
    result := i2c.write (SLAVE_WR)
    i2c.stop
    return (result == i2c#ACK)

PUB Defaults

  Write_OSCTrim (peek_ee ($F7)) ' Write osc trimming val extracted from EEPROM address $F7
'  Write_Cfg ($4638)' (peek_ee($F6)<<8) | peek_ee($F5) )'($4E39) '463E
  _cfg_reg := 0  
  SetRefreshRate (1)
  SetADCRes (18)
  SetMeasureMode (MMODE_CONT)
  SetOperationMode (OPMODE_NORM) '0019 XXX OPMODE_SLEEP unverified (returns 463E)
  SetI2CFM (I2CFMODE_ENA)
  SetEEPROM (EE_ENA)
  SetADCReference (ADCREF_LO)

  time.MSleep (5)

  Read_Cfg
  Read_OSCTrim
  Calc_Consts

PUB command (cmd, addr_start, addr_step, num_reads) | cmd_packet[2], ackbit

  cmd_packet.byte[0] := SLAVE_WR
  cmd_packet.byte[1] := cmd
  cmd_packet.byte[2] := addr_start
  cmd_packet.byte[3] := addr_step
  cmd_packet.byte[4] := num_reads

  i2c.start
  ackbit := i2c.pwrite(@cmd_packet, 5)
  if ackbit == i2c#NAK
    _nak_count++

PRI readword: data_word | read_data

  i2c.start
  _ackbit := i2c.write (SLAVE_RD)
  i2c.pread (@read_data, 2, TRUE)
  i2c.stop

  data_word := (read_data.byte[1] << 8) | read_data.byte[0]

PUB CheckPOR: flag
' Power-On-Reset bit status
' Bit must be set when uploading configuration register
  Read_Cfg
  flag := ((_cfg_reg >> 10) &1)
  flag~~

PUB GetCompPixel | rawpix

  readData (@rawpix, $41, 0, 1)'buff_ptr, addr_start, addr_step, word_count)
'  if rawpix > 32767  'Two's-complement
'    rawpix:= rawpix - 65536
'  return _comp_pix := rawpix
  return _comp_pix := type.u16_s16 (rawpix)

PUB GetLine(ptr_line, line) | rawpix[8], col, pixel

  if not lookdown(line: 0..3)
    return

  readData (@rawpix, line, 4, 16)

  repeat col from 0 to 15
    pixel := (col * 4) + line
    word[ptr_line][pixel] := type.u16_s16 (rawpix.word[col])

PUB GetColumn(ptr_col, col) | rawpix[2], line, pixel

  if not lookdown(col: 0..15)
    return

  readData (@rawpix, col * 4, 1, 4)

  repeat line from 0 to 3
    pixel := (col * 4) + line
    word[ptr_col][pixel] := type.u16_s16 (rawpix.word[line])

PUB GetPixel(ptr_frame, col, line) | rawpix, pixel

  if not lookdown(col: 0..15) or not lookdown(line: 0..3)
    return

  pixel := (col * 4) + line 'Compute offset location in array of current pixel

  readData (@rawpix, pixel, 0, 1)

  word[ptr_frame][pixel] := type.u16_s16 (rawpix & $FFFF)

  return _ir_frame.word[pixel] := rawpix

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
  return _ackbit
 
PUB IsMeasureRunning: flag

  Read_Cfg
  flag := ((_cfg_reg >> 9) & 1)
  flag~~

PUB readData(buff_ptr, addr_start, addr_step, word_count) | ackbit, cmd_packet[2]
{  cmd_packet.byte[0] := SLAVE_WR
  cmd_packet.byte[1] := cmd
  cmd_packet.byte[2] := addr_start
  cmd_packet.byte[3] := addr_step
  cmd_packet.byte[4] := num_reads

  i2c.start
  ackbit := i2c.pwrite(@cmd_packet, 5)
  if ackbit == i2c#NAK
    _nak_count++
}
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
{
  command ($02, addr_start, addr_step, word_count)
  i2c.start
'  ackbit := i2c.write (MLX90621_ADDR|R)
  i2c.write (SLAVE_RD)
  i2c.pread (buff_ptr, word_count << 1, TRUE) '*2 = 81.6uS, << 1 = 74uS w/assign, 71.8uS w/out assign
  i2c.stop
  if ackbit == i2c#NAK
    _nak_count++
}
PUB readNAK

  return _nak_count

PUB Calc_Consts | Scale

  Scale := 100  'Scale factor for fixed-point math
'  _adc_res := pow(2, _adc_res)'3-((_cfg_reg >> 4) & %11))'(_cfg_reg & %0011_0000) >> 4) )

'' Consts for calculating Ta
  _Vth_h := _ee_data.byte[EE_OFFS_VTH+1] '645F
  _Vth_l := _ee_data.byte[EE_OFFS_VTH]
  _Kt1_h := _ee_data.byte[EE_OFFS_KT1+1] '545E
  _Kt1_l := _ee_data.byte[EE_OFFS_KT1]
  _Kt2_h := _ee_data.byte[EE_OFFS_KT2+1] '5EB9
  _Kt2_l := _ee_data.byte[EE_OFFS_KT2]
  _Kt1_Scl := (_ee_data.byte[EE_OFFS_KT1SCL] >> 4) & $0F '8B
  _Kt2_Scl := _ee_data.byte[EE_OFFS_KT2SCL] & $0F

  _Vth25 := type.s16(_Vth_h, _Vth_l)
  _Vth25 := (_Vth25/pow(2, _adc_res)) * Scale

  _Kt1 := type.s16(_Kt1_h, _Kt1_l)
  _Kt1 := (_Kt1*Scale)/((pow(2, _Kt1_Scl) * pow(2, _adc_res))){*Scale} 'Whole part

  _Kt2 := type.s16(_Kt2_h, _Kt2_l)
  _Kt2 := (_Kt2*Scale)/((pow(2, (_Kt2_Scl) + 10) * pow(2, _adc_res))) 'Page 15
  
  return _Vth25

PUB ReadPTAT | read_data, lsbyte, msbyte, Scale, cmd_packet

  Scale := 100
  msbyte := lsbyte := 0
  readData (@read_data, RAM_OFFS_PTAT, 0, 1)
  _PTAT := type.u16(read_data.byte[1], read_data.byte[0])' * Scale

PUB Ta
''  Ta = ((-_Kt1 + sqrt(_Kt1^2 - 4Kt2[Vth(25)-PTAT_data])) / 2Kt2) + 25  '(degC)
  return (_Kt1  * -1) + ^^(pow(_Kt1, 2) - (4 * _Kt2) * (_Vth25 - _PTAT) / (2 * _Kt2) ) + 25

PUB calc_pixOffsetCompensation(col, line) | Ai, Ai_del, Ai_scl, A_com, Bi, Bi_scl, Acp, Bcp, Vir
' XXX Change to process whole frame of a ptr_buf?
  Vir := GetPixel (@_ir_frame, col, line)'Changed because of GetPixel() change

  A_com := type.s16(_ee_data.byte[$D1], _ee_data.byte[$D0])
  Ai_del := (_ee_data.byte[$00 + (col * 4) + line])
  Ai_scl := (_ee_data.byte[$D9] & $F0)
  Ai := (A_com + Ai_del * pow(2, Ai_scl)) / pow (2, _adc_res)

  Bi := type.s8(_ee_data.byte[$40 + (col * 4) + line])
  Bi_scl := (_ee_data.byte[$D9] & $0F)
  _Vir_comp := Vir - (Ai + Bi * (Ta - 25))
  return _Vir_comp

PUB calc_comppixOffsetCompensation(col, line) | Ai, Ai_del, Ai_scl, A_com, Bi, Bi_scl, Acp, Bcp, Vir

  Vir := GetCompPixel

  A_com := (_ee_data.byte[$D1] << 4) | _ee_data.byte[$D0]
  Ai_del := (_ee_data.byte[$00 + (col * 4) + line])
  Ai_scl := (_ee_data.byte[$D9] & $F0)
  Ai := (A_com + Ai_del * (2 * Ai_scl)) / pow (2, _adc_res)

  Bi := _ee_data.byte[$40 + (col * 4) + line]
  Bi_scl := (_ee_data.byte[$D9] & $0F)
  _Vir_comp := Vir - (Ai + Bi * (Ta - 25))
  return _Vir_comp


PUB pow(a, b) | p

  if b == 0
    return 1
  elseif b // 2 == 1
    return a * pow(a, b - 1)
  else
    p := pow(a, b / 2)
    return p * p

PUB Read_Cfg | read_data, por_bit

'    command($02, $92, 0, 1)
'    read_data := readword
    readData (@read_data, core#REG_CFG, 0, 1)'(buff_ptr, addr_start, addr_step, word_count)
    
    _cfg_reg := (read_data.byte[1] << 8) | read_data.byte[0]
    por_bit := (_cfg_reg & %0000_0100_0000_0000) >> 10        'Check if POR bit (bit 10) is set
    return _cfg_reg
'XXX FIXME: por_bit isn't used...

PUB Read_OSCTrim | read_data, osctrim_data

  command($02, $93, 0, 1)
  read_data := readword
  
  osctrim_data := (read_data.byte[1] << 8) | read_data.byte[0]

PUB SetADCReference(mode)
' Set ADC reference high, low
'   ADCREF_HI (0) - ADC High reference enabled
'   ADCREF_LO (1) - ADC Low reference enabled (default)
' NOTE: Re-cal must be done after this method is called
  Read_Cfg
  case mode
    ADCREF_HI:
      mode := %0
    ADCREF_LO:
      mode := %1
    OTHER:
      mode := %1

  _cfg_reg |= (mode << 14)
  Write_Cfg (_cfg_reg)
  'TODO: Call Re-cal method here

PUB SetADCRes(bits)
' Set ADC resolution
' NOTE: Updates the VAR _adc_res, as this is used in the various thermal correction calculations
' TODO:
'   Create a wrapper re-cal method
  Read_Cfg
  case bits
    15:
      bits := %00
    16:
      bits := %01
    17:
      bits := %10
    18:
      bits := %11
    OTHER:
        return

  _cfg_reg |= (bits << 4)
  Write_Cfg (_cfg_reg)
  _adc_res := 3-((_cfg_reg >> 4) & %11)'(_cfg_reg & %0011_0000) >> 4)  'Update the VAR used in calculations

PUB SetEEPROM(mode)
' Enable/disable the EEPROM
'   EE_ENA, 0 - EEPROM enabled
'   EE_DIS, 1 - EEPROM disabled
  Read_Cfg
  case mode
    EE_ENA, 0:
      mode := %0
    EE_DIS, 1:
      mode := %1
    OTHER:
      mode := %0

  _cfg_reg |= (mode << 12)
  Write_Cfg (_cfg_reg)

PUB SetI2CFM(mode)
' Set I2C FM+ mode
'   I2CFMODE_ENA (0), 1000, 1_000_000 - Max I2C bus speed/bit transfer rate up to 1000kbit/sec (default)
'   I2CFMODE_DIS (1), 400, 400_000 - Max I2C bus speed/bit transfer rate up to 400kbit/sec
  Read_Cfg
  case mode
    I2CFMODE_ENA, 1000, 1_000_000:
      mode := %0
    I2CFMODE_DIS, 400, 400_000:
      mode := %1
    OTHER:
      mode := %0

  _cfg_reg |= (mode << 11)
  Write_Cfg (_cfg_reg)

PUB SetMeasureMode(mode) | refrate_tmp, adcres_tmp
' Set measurement mode
'   MMODE_CONT (0) - Continuous (default)
'   MMODE_STEP (1) - Step
    Read_Cfg
    case mode
        MMODE_CONT:
            mode := %0
        MMODE_STEP:
            mode := %1
        OTHER:
            return

    refrate_tmp := _cfg_reg & %1111
    adcres_tmp := (_cfg_reg >> 4) & %11
    _cfg_reg := ((_cfg_reg >> 7) << 7) | (mode << 6) | (adcres_tmp << 4) | refrate_tmp
    Write_Cfg (_cfg_reg)

PUB SetOperationMode(mode)
'Set Operation mode
' OPMODE_NORM (0) - Normal (default)
' OPMODE_SLEEP (1) - Sleep mode
  Read_Cfg
  case mode
    OPMODE_NORM:
      mode := %0
    OPMODE_SLEEP:
      mode := %1
    OTHER:
      mode := %0

  _cfg_reg |= (mode << 7)
  Write_Cfg (_cfg_reg)

'PUB SetPOR
' Power-On-Reset bit status
' Bit must be set when uploading configuration register
'  Read_Cfg
'  Write_Cfg (_cfg_reg | (%1 << 10))
'  _cfg_reg |= (%1 << 10)

PUB SetRefreshRate(Hz)
' Set sensor refresh rate
' Valid values are 0, 0.5 or 5 for 0.5Hz, or 1 to 512 in powers of 2
' NOTE: Higher rates will yield noisier images
  Read_Cfg
  case Hz
    0, 0.5, 5:
      Hz := %1111
    1:
      Hz := %1110
    2:
      Hz := %1101
    4:
      Hz := %1100
    8:
      Hz := %1011
    16:
      Hz := %1010
    32:
      Hz := %1001
    64:
      Hz := %1000
    128:
      Hz := %0111
    256:
      Hz := %0110
    512:
      Hz := %0101
    OTHER:
      Hz := %1110 'Default to 1 Hz

  _cfg_reg := (_cfg_reg >> 4) << 4
  _cfg_reg |= Hz
  Write_Cfg (_cfg_reg)
  return _cfg_reg

PUB Write_OSCTrim(val_word) | ck, lsbyte, lsbyte_ck, msbyte, msbyte_ck

  ck := $AA
  lsbyte := val_word.byte[0]
  msbyte := val_word.byte[1]
  lsbyte_ck := (lsbyte - ck)           'Generate simple checksum values
  msbyte_ck := (msbyte - ck)           'from least and most significant bytes 
  
  i2c.start
  _ackbit := i2c.write (SLAVE_WR)
  _ackbit := i2c.write (core#CMD_WRITEREG_OSCTRIM)
  _ackbit := i2c.write (lsbyte_ck)
  _ackbit := i2c.write (lsbyte)
  _ackbit := i2c.write (msbyte_ck)
  _ackbit := i2c.write (msbyte)
  i2c.stop

PUB Write_Cfg(val_word) | ck, lsbyte, lsbyte_ck, msbyte, msbyte_ck
'' val_word is MSB first
  ck := $55                             'XXX MAKE CONSTANT
  lsbyte := val_word.byte[0]
  msbyte := val_word.byte[1]
  msbyte |= %0100                       'Bit 10 of the data (i.e., bit 2 of the MSB) XXX REWRITE TO OR THIS IN BEFORE SEPARATING?
                                        'is the POR/Brown-Out flag, and MUST be set
                                        'to indicate the device hasn't been reset

  lsbyte_ck := (lsbyte - ck)            'Generate simple checksum values
  msbyte_ck := (msbyte - ck)            'from least and most significant bytes 

  i2c.start                             'XXX PACKETIZE, OR USE REUSABLE WRITE METHOD
  _ackbit := i2c.write (SLAVE_WR)
  _ackbit := i2c.write (core#CMD_WRITEREG_CFG)
  _ackbit := i2c.write (lsbyte_ck)
  _ackbit := i2c.write (lsbyte)
  _ackbit := i2c.write (msbyte_ck)
  _ackbit := i2c.write (msbyte)
  i2c.stop
  return (msbyte<<8)|lsbyte

PUB dump_ee(ptr)' | ee_offset
' Make sure ptr is 256 bytes in size!
'  repeat ee_offset from 0 to EE_SIZE-1
'    byte[ptr][ee_offset] := peek_ee(ee_offset)
  bytemove(ptr, @_ee_data, EE_SIZE-1)

PUB peek_ee(location)
'' Return byte at 'location' in EEPROM memory
  return _ee_data.byte[location]
  
PUB read_ee(ptr_ee_data) | ee_offset    'XXX CLEANUP...Use ptr_ee_data consistently
'' Read EEPROM contents into RAM
  bytefill (@_ee_data, $00, EE_SIZE)  'Make sure data in RAM copy of EEPROM image is clear

  i2c.start                           'Start reading at addr $00
  _ackbit := i2c.write (core#EE_SLAVE_ADDR)
  _ackbit := i2c.write ($00)

  i2c.start                           'Read in the EEPROM
  _ackbit := i2c.write (core#EE_SLAVE_ADDR|1)
  i2c.pread (@_ee_data, EE_SIZE-1, TRUE)
  i2c.stop

  repeat ee_offset from 0 to EE_SIZE-1
    byte[ptr_ee_data][ee_offset] := _ee_data.byte[ee_offset]


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
