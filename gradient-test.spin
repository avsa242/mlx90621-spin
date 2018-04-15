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
  oled  : "display.oled96"

VAR

  long a, b, c, d
  long MinTemp, MaxTemp
  long val

PUB Main

  ser.Start (115_200)
  

PUB GetColor | red, green, blue

{
    pass in value and figure out R G B
    several published ways to do this I basically graphed R G B and developed simple linear equations
    again a 5-6-5 color display will not need accurate temp to R G B color calculation

    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html

}

  red := constrain(255 / (c - b) * val - ((b * 255) / (c - b)), 0, 255)

  if ((val > MinTemp) & (val < a))
    green := constrain(255 / (a - MinTemp) * val - (255 * MinTemp) / (a - MinTemp), 0, 255)
  elseif ((val => a) & (val =< c))
    green := 255
  elseif (val > c)
    green := constrain(255 / (c - d) * val - (d * 255) / (c - d), 0, 255)
  elseif ((val > d) | (val < a))
    green := 0

  if (val =< b)
    blue := constrain(255 / (a - b) * val - (255 * b) / (a - b), 0, 255)
  elseif ((val > b) & (val =< d))
    blue := 0
  elseif (val > d)
    blue := constrain(240 / (MaxTemp - d) * val - (d * 240) / (MaxTemp - d), 0, 240)

'   use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
'  return Display.color565(red, green, blue)

PUB Constrain(val, lower, upper)

  return lower #> val <# upper

PUB SetTempScale | i

  if (DefaultTemp < 0)
    MinTemp := 25
    MaxTemp := 35
    Getabcd
'    DrawLegend
  else
    val := 0'.0
    repeat i from 0 to 63
      val := val + pixels[i]
    val := val / 64

    MaxTemp := val + 2
    MinTemp := val - 2
    Getabcd
'    DrawLegend();

PUB Getabcd
'Scaled by 10_000
  a := MinTemp + (MaxTemp - MinTemp) * 2121'0.2121
  b := MinTemp + (MaxTemp - MinTemp) * 3182'0.3182
  c := MinTemp + (MaxTemp - MinTemp) * 4242'0.4242
  d := MinTemp + (MaxTemp - MinTemp) * 8182'0.8182

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
