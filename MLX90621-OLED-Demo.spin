 {
    --------------------------------------------
    Filename: MLX90621-OLED-Demo.spin
    Author: Jesse Burt
    Description: Demo of the MLX90621 driver using
        an SSD1331 OLED as a display
    Copyright (c) 2020
    Started: Dec 2, 2019
    Updated: Mar 7, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

    LED         = cfg#LED1
    SER_RX      = 31
    SER_TX      = 30
    SER_BAUD    = 115_200

' OLED-96 I/O Pin assignments
    RES_PIN     = 4
    DC_PIN      = 3
    CS_PIN      = 2
    CLK_PIN     = 1
    DIN_PIN     = 0

    I2C_SCL     = 14
    I2C_SDA     = 15
    I2C_HZ      = 1_000_000

    BUFFSZ      = 96*64
    XMAX        = 95
    YMAX        = 63

OBJ

    cfg     : "core.con.boardcfg.activityboard"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    io      : "io"
    mlx     : "sensor.thermal-array.mlx90621.i2c.spin"
    oled    : "display.oled.ssd1331.spi"
    int     : "string.integer"

VAR

    long _drawframe_stack[50]
    long _keydaemon_stack[50]

    long _a_min, _a_max, _a_range
    long _b_min, _b_max, _b_range
    long _c_min, _c_max, _c_range
    long _d_min, _d_max, _d_range

    long _offset

    word _ir_frame[64], _frame_buff[BUFFSZ]

    byte _ee_img[256]
    byte _sx, _sy, _width, _height
    byte _ser_cog

PUB Main | x, y

    Setup

    SetColorScale
    wordfill(@_ir_frame, 0, 64)

    _sx := 0           ' Starting position of thermal image
    _sy := 0
    _width := 5        ' Size of each pixel
    _height := 5

    DrawScale

    repeat
        mlx.GetFrame (@_ir_frame)
        DrawFrame

PUB Constrain(val, lower, upper)

    return lower #> val <# upper

PUB DrawFrame | x, y, color_c, k, ex, ey
' Draw the thermal image
    ex := _sx + _width
    ey := _sy + _height

    repeat y from 0 to 3
        repeat x from 0 to 15
            k := (x * 4) + y
            color_c := GetColor ((_ir_frame.word[k]+_offset))   ' Computed color
'            color_c := _ir_frame.word[k]+_offset               ' Raw value from sensor
            oled.box((x * 6) + _sx, (y * 6) + _sy, (x * 6) + ex, (y * 6) + ey, color_c, TRUE)

    oled.Update

PUB DrawScale | i
' Draw Color Scale
'  65535 max range from thermal sensor
'  95 max horizontal pixel range of OLED display
'  ...so each step in the color scale is 689 (65535/95)
    repeat i from 0 to 95
        oled.line (i, (_height*5)+_sy+4, i, (_height*5)+_sy+11, GetColor (i*689))

' Draw box surrounding thermal image
'    oled.box(_sx-2, _sy-2, (_width*20) + sx+1, (_height*5) + sy+1, $FFFF)
    oled.Update

PUB GetColor(val) | red, green, blue, inmax, outmax, divisor
' Get color of input value based on 4-point scale
'   XXX broken
    longfill(@red, $00, 6)  ' initialize all of the variables to 0
    inmax := 65536
    outmax := 256
    divisor := inmax / outmax

    case val
        _a_min.._a_max:
            red := 0
            green := 0
            blue := val/divisor
        _b_min.._b_max:
            red := 0
            green := val/divisor
            blue := 255
        _c_min.._c_max:
            red := val/divisor
            green := 255
            blue := 255-(val/divisor)
        _d_min.._d_max:
            red := 255
            green := 255-(val/divisor)
            blue := 0
        OTHER:
' RGB888 format
'    return (red << 16) | (green << 8) | blue

' RGB565 format
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)

PUB keydaemon | cmd

    repeat
        repeat until cmd := ser.CharIn
        case cmd
            "-":
                _offset := 0 #> (_offset - 1000)
                if _offset < 0
                    _offset := 0
                ser.Position(0, 11)
                ser.str(string("Offset: "))
                ser.Dec (_offset)
                ser.NewLine
            "=":
                _offset := (_offset + 1000) <# 65535
                ser.Position(0, 11)
                ser.str(string("Offset: "))
                ser.Dec (_offset)
                ser.NewLine
            "e":
                HexDump(@_ee_img)
            "t":
                DumpFrame
            OTHER:

PUB HexDump(ptr) | seg, off

    ser.Position(0, 14)
    mlx.dump_ee (ptr)
    repeat seg from 0 to mlx#EE_SIZE-8 step 8
        ser.Hex (seg, 2)
        ser.str(string(": "))
        repeat off from 0 to 7
            ser.Hex (byte[ptr][seg+off], 2)
            ser.Char (" ")
        ser.NewLine

PUB DumpConfig

    ser.str(string("Refresh rate: "))
    ser.Dec (mlx.RefreshRate (-2))
    ser.NewLine

    ser.str(string("ADC Res: "))
    ser.Dec (mlx.ADCRes (-2))
    ser.NewLine

    ser.str(string("Reset: "))
    ser.Dec (mlx.Reset (-2))
    ser.NewLine

    ser.str(string("ADC Ref: "))
    ser.Dec (mlx.ADCReference (-2))
    ser.NewLine

    ser.str(string("Measure mode: "))
    ser.Dec (mlx.MeasureMode (-2))
    ser.NewLine

    ser.str(string("Operation mode: "))
    ser.Dec (mlx.OperationMode (-2))
    ser.NewLine

    ser.str(string("I2C Fast Mode+: "))
    ser.Dec (mlx.I2CFM (-2))
    ser.NewLine

    ser.str(string("Oscillator Trim val: $"))
    ser.Hex (mlx.OSCTrim (-2), 2)
    ser.NewLine

PUB DumpFrame | line, col, k

    repeat line from 0 to 3
        repeat col from 0 to 15
            k := (col * 4) + line
            ser.Position (1+(col * 5), 5+line)
            ser.Hex (_ir_frame.word[k], 4)
            ser.Char (" ")

PUB SetColorScale

    _a_min := 0
    _a_max := 16383
    _a_range := _a_max - _a_min

    _b_min := _a_max + 1
    _b_max := 32767
    _b_range := _b_max - _b_min

    _c_min := _b_max + 1
    _c_max := 49151
    _c_range := _c_max - _c_min

    _d_min := _c_max + 1
    _d_max := 65535
    _d_range := _d_max - _d_min

PUB Setup

    repeat until _ser_cog := ser.StartRXTX (SER_RX, SER_TX, 0, SER_BAUD)
    time.MSleep(30)
    ser.Clear
    ser.str(string("Serial terminal started", ser#CR, ser#LF))

    if oled.Start (CS_PIN, DC_PIN, DIN_PIN, CLK_PIN, RES_PIN)
        ser.str(string("SSD1331 driver started", ser#CR, ser#LF))
        oled.Defaults
        oled.Address(@_frame_buff)
        oled.ClockDiv(1)
        oled.ClockFreq(956)
        oled.AddrIncMode (oled#ADDR_HORIZ)
        oled.MirrorH (FALSE)
        oled.SubpixelOrder (oled#SUBPIX_RGB)
        oled.VertAltScan (FALSE)
        oled.MirrorV (FALSE)
        oled.Interlaced (FALSE)
        oled.ColorDepth (oled#COLOR_65K)
        oled.ClearAll
        oled.Fill(TRUE)
        oled.Contrast (127)
        oled.Update
    else
        ser.str(string("SSD1331 driver failed to start - halting", ser#CR, ser#LF))
        time.MSleep (5)
        oled.stop
        mlx.Stop
        repeat

    if mlx.Startx (I2C_SCL, I2C_SDA, I2C_HZ)
        ser.str(string("MLX90621 object started", ser#CR, ser#LF))
        ser.NewLine
    else
        ser.str(string("MLX90621 object failed to start - halting", ser#CR, ser#LF))
        time.MSleep (5)
        oled.stop
        mlx.Stop
        repeat

    mlx.Defaults
    mlx.MeasureMode(mlx#MMODE_CONT)
    mlx.ADCRes (18)
    mlx.RefreshRate (32)

    cognew(keydaemon, @_keydaemon_stack)

#include "lib.utility.spin"

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
