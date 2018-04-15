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

'' OLED-96 I/O Pin assignments
  CS        = 2
  RST       = 4
  DC        = 3
  CLK       = 1
  DATA      = 0

OBJ

  cfg   : "core.con.client.activityboard"
  ser   : "com.serial.terminal"
  time  : "time"
  therm : "sensor.thermal.array.mlx90621"
  oled  : "display.oled96"
  debug : "debug"
  
VAR

  long _drawframe_stack[100]
  word _ir_frame[64]

PUB Main

  Setup

  wordfill(@_ir_frame, 0, 64)


  repeat
    therm.GetFrame (@_ir_frame)

PUB DrawFrame | col, line, k, sx, sy, width, height, color

  sx := 7
  sy := 5
  width := 4
  height := 4

  repeat
    repeat line from 0 to 3
      repeat col from 0 to 15
        k := (col * 4) + line
        color := (65535-_ir_frame.word[k])
        oled.box((col * 5) + sx, line * 5, (col * 5) + sx + width, (line * 5) + height, color, color)
'        oled.PlotPoint (col + sx, line + sy, _ir_frame.word[k], _ir_frame.word[k])

PUB Setup

  ser.Start (115_200)
  ser.Clear

  oled.Init(CS, DC, DATA, CLK, RST)
  oled.clearDisplay
  oled.AutoUpdateOn
  oled.clearDisplay
  oled.boxFillOn
  cognew(DrawFrame, @_drawframe_stack)

  therm.Start (8, 7, 400_000)

'  debug.LEDFast (26)


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
