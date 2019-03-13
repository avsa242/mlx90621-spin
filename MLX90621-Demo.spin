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
    CS      = 2
    RST     = 4
    DC      = 3
    CLK     = 1
    DATA    = 0

    SCL     = 8
    SDA     = 7
    I2C_FREQ= 1_000_000

OBJ

  cfg   : "core.con.boardcfg.flip"
  ser   : "com.serial.terminal"
  time  : "time"
  therm : "sensor.thermal.array.mlx90621"
  oled  : "display.oled96"
  int   : "string.integer"

VAR

  word _ir_frame[64]        '64 words for displayable frame, plus two more: PTAT and compensation pixel
  long _drawframe_stack[50]
  long _serframe_stack[50]
  long _adc_stack[50]
  long _keydaemon_stack[50]
  byte _therm_cog
  byte _ee_img[256]

  long a_min, a_max, a_range
  long b_min, b_max, b_range
  long c_min, c_max, c_range
  long d_min, d_max, d_range

  long Vin_100
  long _offset
  word _cfg_reg
  word _ptat_raw

PUB Main

  a_min := 0
  a_max := 16383
  a_range := a_max - a_min
 
  b_min := a_max + 1
  b_max := 32767
  b_range := b_max - b_min

  c_min := b_max + 1
  c_max := 49151
  c_range := c_max - c_min
  
  d_min := c_max + 1
  d_max := 65535
  d_range := d_max - d_min
  
  Setup

  wordfill(@_ir_frame, 0, 64)

  repeat
'    ser.Hex (therm.Read_Cfg, 8)
'    ser.NewLine
'    time.mSleep (100)
'    therm.GetFrameExt (@_ir_frame)
    therm.GetFrame (@_ir_frame)

'    therm.GetLine (@_ir_frame, 1)
'    therm.GetPixel (@_ir_frame, 6, 3)
'    therm.GetColumn (@_ir_frame, 4)

PUB Constrain(val, lower, upper)

  return lower #> val <# upper

PUB DrawFrame | col, line, sx, sy, width, height, color, i, k, buf

    sx := 7           ' Starting position of thermal image
    sy := 5
    width := 4        ' Size of each pixel
    height := 4

'' Draw box surrounding thermal image
    oled.box(sx-2, sy-2, (width*20) + sx+1, (height*5) + sy+1, oled#White, 0)

'' Draw Color Scale
''  65535 max range from thermal sensor
''  95 max horizontal pixel range of OLED display
''  ...so each step in the color scale is 689 (65535/95)
    repeat i from 0 to 95
        oled.line (i, (height*5)+sy+3, i, (height*5)+sy+11, GetColor (i*689))

'' Draw the thermal image
    buf := oled.getBuffer
    repeat
        repeat line from 0 to 3
            repeat col from 0 to 15
                k := (col * 4) + line
                word[buf][k] := GetColor ((_ir_frame.word[k])+_offset)
        oled.WRITEBUFF (DATA, CLK, CS, 1024, buf)
'        wordmove(buf, @_ir_frame, 64)
{
    repeat
        repeat line from 0 to 3
            repeat col from 0 to 15
                k := (col * 4) + line
                color := GetColor ((_ir_frame.word[k])+_offset)
                oled.box((col * 5) + sx, (line * 5)+sy, (col * 5) + sx + width, (line * 5) + sy + height, color, color)
}
PUB GetColor(val) | red, green, blue, inmax, outmax, divisor

    inmax := 65535
    outmax := 255
    divisor := Constrain (inmax, 0, 65535)/outmax

    case val
        a_min..a_max:
            red := Constrain ((val/divisor), 0, 255)
            green := 0
            blue := Constrain ((val/divisor), 0, 255)
        b_min..b_max:
            red := Constrain (255-(val/divisor), 0, 255)
            green := 0
            blue := 255
        c_min..c_max:
            red := Constrain ((val/divisor), 0, 255)
            green := Constrain ((val/divisor), 0, 255)
            blue := Constrain (255-(val/divisor), 0, 255)
        d_min..d_max:
            red := 255
            green := 255
            blue := Constrain ((val/divisor), 0, 255)
        OTHER:
' RGB888 format
'    return (red << 16) | (green << 8) | blue

' RGB565 format
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)

PUB serframe | line, col, k, color, tmax, tmin

  tmin := 32767
  tmax := 32767

  repeat
    ser.Position (0, 0)
    repeat line from 0 to 3
      repeat col from 0 to 15
        k := (col * 4) + line
        color := GetColor (_ir_frame.word[k])
        if color < tmin
          tmin := color
          ser.Str (string("Min: "))
          ser.Dec (tmin)
          ser.NewLine
        if color > tmax
          tmax := color
          ser.Str (string("Max: "))
          ser.Dec (tmax)
          ser.NewLine
'        ser.Dec (color)
'        ser.Char (" ")
'      ser.NewLine

PUB keydaemon | cmd

    repeat until ser.Start (115_200)
    ser.Clear
    ser.Str (string("Serial terminal started", ser#NL))
    if _therm_cog
        ser.Str (string("MLX90621 object started on cog "))
        ser.Dec (_therm_cog)
        ser.NewLine
    else
        ser.Str (string("Error: MLX90621 object failed to start - halting", ser#NL))
        time.MSleep (5)
        oled.stop
        therm.Stop
        ser.Stop
        repeat
    ser.Str (string(ser#NL, "cfg reg: "))
    ser.Hex (_cfg_reg := therm.Read_Cfg, 8)
    ser.Char (" ")
    ser.Bin (_cfg_reg, 16)
    ser.NewLine

    repeat
        repeat until cmd := ser.CharIn
        case cmd
            "-":
                _offset-=1000
                if _offset < 0
                    _offset := 0
                ser.Str (string("Offset: "))
                ser.Dec (_offset)
                ser.NewLine
            "=":
                _offset+=1000
                if _offset > 65535
                    _offset := 65535
                ser.Str (string("Offset: "))
                ser.Dec (_offset)
                ser.NewLine
            "c":
                ser.Str (string("Init cfg reg: "))
                ser.Hex (_cfg_reg, 8)
                ser.NewLine
            "e":
                hexdump(@_ee_img)
            "t":
                dump_frame
            OTHER:

PUB hexdump(ptr) | seg, off

    therm.dump_ee (ptr)
    repeat seg from 0 to therm#EE_SIZE-8 step 8
        ser.Hex (seg, 2)
        ser.Str (string(": "))
        repeat off from 0 to 7
            ser.Hex (byte[ptr][seg+off], 2)
            ser.Char (" ")
        ser.NewLine

PUB dump_frame | line, col, k

    repeat line from 0 to 3
        repeat col from 0 to 15
            k := (col * 4) + line
            ser.Position (col * 5, line)
            ser.Hex (_ir_frame.word[k], 4)
            ser.Char (" ")
{  k++
  ser.Hex (_ir_frame.word[k], 4)
  ser.NewLine
  k++
  ser.Hex (_ir_frame.word[k], 4)
  ser.NewLine
}
PUB Setup

    _therm_cog := therm.Startx (SCL, SDA, I2C_FREQ)
    therm.Defaults
    therm.SetRefreshRate (32)

    cognew(keydaemon, @_keydaemon_stack)
    oled.Init(CS, DC, DATA, CLK, RST)
    oled.clearDisplay
    oled.AutoUpdateOn
    oled.clearDisplay
    oled.boxFillOn
    cognew(DrawFrame, @_drawframe_stack)

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
