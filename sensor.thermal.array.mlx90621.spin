{
    --------------------------------------------
    Filename:
    Author:
    Copyright (c) 20__
    See end of file for terms of use.
    --------------------------------------------
}

CON

  _clkmode  = cfg#_clkmode
  _xinfreq  = cfg#_xinfreq

  SLAVE_EE        = $50 << 1
  SLAVE_SENS      = $60 << 1
  W               = %0
  R               = %1
  EE_SIZE         = 256
  EE_CLK          = 400_000
  THERM_CLK       = 1_000_000
  PAGE            = 0
  SINGLE_LINE     = 1

  CMD_WRITE_CFG       = $03
  CMD_WRITE_OSC_TRIM  = $04


OBJ

  cfg   : "config.activityboard"
  ser   : "com.serial.terminal"
  time  : "time"
  i2c   : "jm_i2c_fast"
  debug : "debug"

VAR

  byte _ee_data[EE_SIZE]
  long scl, sda
  long memsize
  byte _osc_trim

PUB Main | check, i
''  TASKS:
''C  1 POR (see 8.3)
''C  2 wait 5ms
''C  3 read eeprom table
''  4 store cal coeff in prop RAM
''  5 write osc trim val into addr $93
''  6 write cfg val addr $92 (value read from eeprom or hard coded externally)
''    set POR/brown out flag to 1 (bit 10 at $92
''  7 check BO flag. Cleared? Yes: repeat step 3 No: proceed to step 8
''  8 read meas data (PTAT+desired IR data)
''  9 Ta calc
''  10 pix offset cancelling
''  11 therm grad comp
''  12 pix<->pix normalization
''  13 obj emissivity comp
''  14 obj temp calc
''  15 image process/correct
''  loop to step 7

''  1 POR, 2
  scl := 8
  sda := 7

  i2c.setupx (scl, sda, EE_CLK)
  ser.Start (115_200)
  ser.Clear
  ser.Str (string("I2C started - press any key"))
  ser.CharIn
  ser.NewLine
  
  i2c.start
  check := i2c.write(SLAVE_EE|W)
  i2c.stop
  if (check == i2c#ACK)
    ser.Str (string("MLX90621 EEPROM found, reading...", ser#NL))

'   3
  init
  ser.Str (string("Init finished, press a key to dump data:", ser#NL, ser#NL))
  ser.CharIn

  dump_ee
  ser.NewLine
  ser.NewLine

  debug.LEDFast ( cfg#LED2)
  Write_Cfg ($463E)
{
  repeat i from 0 to 511
    ser.Hex (_ee_data.byte[i], 2)
    ser.Char (" ")
    ifnot i//16
      ser.NewLine
  repeat
}

PUB Write_OSCTrim(value) | ackbit, ck, lsb, lsbck, msb, msbck, b

'  b := _ee_data.byte[$F7]
  b:=value
  ck := $AA
  lsb := b.byte[0]
  msb := b.byte[1]
  lsbck := (lsb - ck)
  msbck := (msb - ck)
  
  msg_two(string("b: "), b)
  msg_one(string("lsb: "), lsb)
  msg_one(string("msb: "), msb)
  msg_one(string("lsbck: "), lsbck)
  msg_one(string("msbck: "), msbck)

  debug.LEDFast (cfg#LED2)
  
  i2c.start
  i2c.write (SLAVE_EE|W)
  i2c.write (CMD_WRITE_OSC_TRIM)
  i2c.write (lsbck)
  i2c.write (lsb)
  i2c.write (msbck)
  i2c.write (msb)
  i2c.stop

PUB Write_Cfg(val_word) | ackbit, ck, lsb, lsbck, msb, msbck, b
'' val_word is MSB first
  b:=val_word
  ck := $55
  lsb := b.byte[0]
  msb := b.byte[1]
  lsbck := (lsb - ck)
  msbck := (msb - ck)
  
  msg_two(string("b: "), b)
  msg_one(string("lsb: "), lsb)
  msg_one(string("msb: "), msb)
  msg_one(string("lsbck: "), lsbck)
  msg_one(string("msbck: "), msbck)

  debug.LEDFast (cfg#LED2)

  i2c.start
  i2c.write (SLAVE_EE|W)
  i2c.write (CMD_WRITE_CFG)
  i2c.write (lsbck)
  i2c.write (lsb)
  i2c.write (msbck)
  i2c.write (msb)
  i2c.stop
  
PUB dump_ee | pg, row, val

  pg := 0
  
  ser.Chars (" ", 4)
  repeat val from 0 to 15
    ser.Hex (val, 2)
    ser.Char (" ")
  ser.NewLine
  ser.NewLine
  
  repeat row from 0 to 15
    ser.Hex (row*$10, 2)
    ser.Str (string(": "))
    repeat val from 0 to 15
      ser.Hex (_ee_data.byte[pg+val], 2)
      ser.Char (" ")
    ser.NewLine
    pg+=16

  
PUB init | ackbit
  
  bytefill (@_ee_data, $00, EE_SIZE)'(s, v, c)
  _ee_data.byte[EE_SIZE-2] := $BE   'Small identifier to make sure
  _ee_data.byte[EE_SIZE-1] := $EF   'the data being read is definitely 
  i2c.start
  ackbit := i2c.write (SLAVE_EE|W)
  ackbit := i2c.write ($00)

  i2c.start
  ackbit := i2c.write (SLAVE_EE|R)
  i2c.stop
  ackbit := i2c.pread (@_ee_data, EE_SIZE-1, TRUE)'p_dest, count, ackbit)
  i2c.stop

PRI msg_one(msg, val)
'' Writes string and one byte-sized value to terminal
  ser.Str (msg)
  ser.Hex (val, 2)
  ser.NewLine

PRI msg_two(msg, val)
'' Writes string and one byte-sized value to terminal
  ser.Str (msg)
  ser.Hex (val, 4)
  ser.NewLine
  
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
