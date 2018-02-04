{
    --------------------------------------------
    Filename:
    Author:
    Copyright (c) 20__
    See end of file for terms of use.
    --------------------------------------------
}

CON

  _clkmode = cfg#_clkmode
  _xinfreq = cfg#_xinfreq

OBJ

  cfg   : "config.activityboard"
  ser   : "com.serial.terminal"
  time  : "time"
  therm : "sensor.thermal.array.mlx90621"

VAR


PUB Main

  ser.Start (115_200)
  ser.Clear

  therm.Start (8, 7, 100_000)

  Write_OSCTrim ($0010)'(peek_ee ($F7)) ' Write osc trimming val extracted from EEPROM address $F7
  ser.NewLine
  Write_Cfg ($4E3B) '463E
  time.MSleep (5)
  
  ser.NewLine
  Read_Cfg
  Read_OSCTrim
  ser.CharIn

{
  repeat
    Read_PTAT
    time.mSleep (100)
}
  wordfill(@_ir_frame, 0, 64)


  repeat
'    GetColumn(0)
    Getline (1)
'    GetPixel (3, 2)
'    GetFrame
    repeat line from 0 to 3
      repeat col from 0 to 15
        ser.Hex (_ir_frame.word[((line*16)+col)], 4)
        ser.Char (" ")
      ser.NewLine
    ser.NewLine
    time.MSleep (100)


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
