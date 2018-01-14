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

  SLAVE           = $50 << 1
  W               = %0
  R               = %1
  EE_SIZE         = 256
  EE_CLK          = 400_000
  THERM_CLK       = 1_000_000
  PAGE            = 0
  SINGLE_LINE     = 1

  WRITE_OSC_TRIM  = $04

OBJ

  cfg   : "config.activityboard"
  ser   : "com.serial.terminal"
  time  : "time"
  i2c   : "jm_i2c_fast"

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
  check := i2c.write(SLAVE|W)
  i2c.stop
  if (check == i2c#ACK)
    ser.Str (string("MLX90621 EEPROM found, reading...", ser#NL))
'   3
    init
  ser.Str (string("Init finished, press a key to dump data:", ser#NL, ser#NL))
  ser.CharIn

  hexdump(@_ee_data)
  repeat
{
  repeat i from 0 to 511
    ser.Hex (_ee_data.byte[i], 2)
    ser.Char (" ")
    ifnot i//16
      ser.NewLine
  repeat
}
  
PUB hexdump(base_ptr) | seg, i, j, cmd, display_mode, pageSize, lines, s, e

  display_mode := PAGE
  memsize := EE_SIZE-1
  pageSize := 16
  lines := 1

  repeat
    ser.Clear
    ser.Str (string("SEG        +0  1  2  3   4  5  6  7   8  9  A  B   C  D  E  F       CONTENTS:", ser#NL, {
                    }"----------------------------------------------------------------------------", ser#NL))

    case display_mode
      SINGLE_LINE:
        ser.Hex (base_ptr, 4)                    'Print segment
        ser.Chars (32, 2)

        repeat j from base_ptr to base_ptr+15         'Print hex value of the 16 memory locations relative to current segment
          ser.Hex (byte[base_ptr+j], 2)
          ser.Char (32)
          if not (j+1) // 4                   'Draw extra space between each long
            ser.Char (32)
        ser.Chars (32, 2)

        repeat j from base_ptr to base_ptr+15         'Print the contents of those memory locations
          case byte[base_ptr+j]
            32..127:                          'Literally if it's a printable character...
              ser.Char (byte[base_ptr+j])
            OTHER:                            '...or a period if it's not
              ser.Char (".")

        ser.NewLine

      PAGE:
        s:=cnt
        repeat seg from base_ptr to base_ptr + memsize-31 step 16'(pageSize-(lines*1))
          ser.Hex ((seg-base_ptr)+16, 4)
          ser.Char ("/")
          ser.Hex (seg, 4)                   'Print segment
          ser.Chars (32, 2)
          repeat j from seg to seg + 15       'Print hex value of the 16 memory locations relative to current segment
            ser.Hex (byte[j], 2)
            ser.Char (32)
            if not ((j+1)-seg) // 4           'Draw extra space between each long
              ser.Char (32)
          ser.Chars (32, 2)

          repeat j from seg to seg + 15       'Print the contents of those memory locations
            case byte[j]
              32..127:                        'Literally if it's a printable character...
                ser.Char (byte[j])
              OTHER:                          '...or a period if it's not
                ser.Char (".")
          ser.NewLine
        e:=cnt-s
        ser.Str (string("Took "))
        ser.Dec (e)
        ser.Str (string(" cycles to display", ser#NL))
        ser.NewLine
      OTHER:                                  'Default to page mode
        display_mode := PAGE

    repeat until cmd := ser.CharIn           'Wait for user input
    case cmd                                  'and decide what to do based on it...
      8:
        base_ptr := base_ptr - 16 #> 0
      13:
        base_ptr := base_ptr + 16 <# memsize-15
      "e", "E":
        base_ptr := memsize-15
      "s", "S":
        base_ptr := 0
      "n", "N", 32:
        base_ptr := base_ptr + 512 <# memsize-15
      "p", "P":
        base_ptr := base_ptr - 512 #> 0
      "=", "+":
        base_ptr := base_ptr + 1 <# memsize-15
      "-", "_":
        base_ptr := base_ptr - 1 #> 0
      "c", 16:
        ser.Clear
      "1":
        display_mode := SINGLE_LINE
      "2":
        display_mode := PAGE
      "[":
        lines := lines + 1 <#32
      "]":
        lines := lines - 1 #> 1
      "{":
        lines := 32
      "}":
        lines := 1
      "j", "J":
        ser.Str (string("Jump to (hex)? $"))
        base_ptr := 0 #> ser.HexIn <# memsize-15
      OTHER:                                  '...if anything

PUB init | ackbit

  _ee_data.byte[EE_SIZE-2] := $BE
  _ee_data.byte[EE_SIZE-1] := $EF
  i2c.start
  ackbit := i2c.write (SLAVE|W)
  ackbit := i2c.write ($00)

  i2c.start
  ackbit := i2c.write (SLAVE|R)
  i2c.stop
  ackbit := i2c.pread (@_ee_data, EE_SIZE-1, TRUE)'p_dest, count, ackbit)
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
