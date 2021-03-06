{
    --------------------------------------------
    Filename: MLX90621-VGA-Demo.spin
    Author: Jesse Burt
    Description: Demo of the MLX90621 driver using a
        VGA display
    Copyright (c) 2021
    Started: Jun 27, 2020
    Updated: Jan 12, 2021
    See end of file for terms of use.
    --------------------------------------------
}
CON

    _clkmode        = cfg#_clkmode
    _xinfreq        = cfg#_xinfreq

' -- User-modifiable constants
    LED             = cfg#LED1
    SER_BAUD        = 115_200

' MLX90621
    I2C_SCL         = 16
    I2C_SDA         = 17
    I2C_HZ          = 1_000_000

    VGA_PINGROUP    = 1                         ' 0, 1, 2, 3
' --

    WIDTH           = 160
    HEIGHT          = 120
    XMAX            = WIDTH-1
    YMAX            = HEIGHT-1
    CENTERX         = XMAX/2
    CENTERY         = YMAX/2
    BUFFSZ          = WIDTH * HEIGHT
    BPP             = 1
    BPL             = WIDTH * BPP

OBJ

    cfg     : "core.con.boardcfg.quickstart-hib"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    mlx     : "sensor.thermal-array.mlx90621.i2c"
    vga     : "display.vga.bitmap.160x120"
    fnt     : "font.5x8"
    int     : "string.integer"

VAR

    long _keyinput_stack[50]
    long _ir_frame[66]
    long _offset
    long _settings_changed

    word _mlx_refrate

    word _fx, _fy, _fw, _fh

    byte _framebuffer[BUFFSZ]
    byte _palette[64]
    byte _mlx_adcres, _mlx_adcref
    byte _invert_x, _col_scl
    byte _hotspot_mark

PUB Main{}

    setup{}
    drawvscale(XMAX-5, 0, YMAX)

    repeat
        if _settings_changed
            updatesettings{}
        mlx.getframe(@_ir_frame)
        drawframe(_fx, _fy, _fw, _fh)

PUB DrawFrame(fx, fy, pixw, pixh) | x, y, color_c, ir_offset, pixsx, pixsy, pixex, pixey, maxx, maxy, maxp
' Draw the thermal image
    vga.waitvsync{}                             ' wait for vertical sync
    repeat y from 0 to mlx#YMAX
        repeat x from 0 to mlx#XMAX
            if _invert_x                        ' Invert X display if set
                ir_offset := ((mlx#XMAX-x) * 4) + y
            else
                ir_offset := (x * 4) + y
            ' compute color
            color_c := _palette[(_ir_frame[ir_offset] * _col_scl) >> 10 + _offset]
            pixsx := fx + (x * pixw)            ' start and end image pixel
            pixsy := fy + (y * (pixh + 1))      '   coords
            pixex := pixsx + pixw
            pixey := pixsy + pixh
            if _ir_frame[ir_offset] > maxp      ' Check if this is the hottest
                maxp := _ir_frame[ir_offset]    '   spot in the image
                maxx := pixsx
                maxy := pixsy
            vga.box(pixsx, pixsy, pixex, pixey, color_c, TRUE)

    if _hotspot_mark                            ' Mark hotspot
        ' white box
'        vga.box(maxx, maxy, maxx+pixw, maxy+pixh, vga#MAX_COLOR, false)

        ' white cross-hair
        vga.line(maxx, maxy+(pixh/2), maxx+pixw, maxy+(pixh/2), vga#MAX_COLOR)
        vga.line(maxx+(pixw/2), maxy, maxx+(pixw/2), maxy+pixh, vga#MAX_COLOR)

PUB DrawVScale(x, y, ht) | idx, color, scl_width, bottom, top, range
' Draw the color scale setup at program start
    range := bottom := y+ht
    top := 0
    scl_width := 5

    repeat idx from bottom to top+(YMAX-vga#MAX_COLOR)
        color := _palette[(range-idx)]
        vga.line(x, idx, x+scl_width, idx, color)

PUB UpdateSettings{} | col, row, reftmp
' Settings have been changed by the user - update the sensor and the
'   displayed settings
    mlx.adcres(_mlx_adcres)                     ' Update sensor with current
    mlx.refreshrate(_mlx_refrate)               '   settings
    mlx.adcreference(_mlx_adcref)

    reftmp := mlx.adcreference(-2)              ' read from sensor for display
    col := 0
    row := (vga.textrows{}-1) - 5               ' Position at screen bottom
    vga.fgcolor(vga#MAX_COLOR)
    vga.position(col, row)

    vga.printf1(string("X-axis invert: %s\n"), lookupz(_invert_x: string("No "), string("Yes")))
    vga.printf1(string("FPS: %dHz   \n"), mlx.refreshrate(-2))
    vga.printf1(string("ADC: %dbits\n"), mlx.adcres(-2))
    vga.printf1(string("ADC reference: %s\n"), lookupz(reftmp: string("High"), string("Low  ")))

    _fx := CENTERX - ((_fw * 16) / 2)           ' Approx center of screen
    _fy := 10
    vga.box(0, 0, XMAX-10, CENTERY, 0, TRUE)    ' Clear out last thermal image
                                                ' (in case resizing smaller)
    _settings_changed := FALSE

PUB cog_keyInput{} | cmd

    repeat
        repeat until cmd := ser.charin{}
        case cmd
            "A":                                ' ADC resolution (bits)
                _mlx_adcres := (_mlx_adcres + 1) <# 18
            "a":
                _mlx_adcres := (_mlx_adcres - 1) #> 15
            "C":                                ' Color scaling/contrast
                _col_scl := (_col_scl + 1) <# 16' ++
            "c":
                _col_scl := (_col_scl - 1) #> 1 ' --
            "F":                                ' sensor refresh rate (Hz)
                _mlx_refrate := (_mlx_refrate * 2) <# 512
            "f":
                _mlx_refrate := (_mlx_refrate / 2) #> 1
            "h":                                ' mark hotspot on/off
                _hotspot_mark ^= 1
            "r":                                ' sensor ADC reference (hi/low)
                _mlx_adcref ^= 1
            "S":                                ' thermal image pixel size
                _fw := (_fw + 1) <# 9           ' ++
                _fh := (_fh + 1) <# 9
            "s":
                _fw := (_fw - 1) #> 1           ' --
                _fh := (_fh - 1) #> 1
            "-":                                ' thermal image reference level
                _offset := 0 #> (_offset - 1)   '   or color offset
            "=":
                _offset := (_offset + 1) <# vga#MAX_COLOR
            "x":                                ' invert thermal image X-axis
                _invert_x ^= 1
            other:
                next
        _settings_changed := TRUE               ' trigger for main loop to call
                                                '   UpdateSettings()
PUB Setup{}

    ser.start(SER_BAUD)
    time.msleep(30)
    ser.clear{}
    ser.strln(string("Serial terminal started"))

    setuppalette{}
    vga.start(VGA_PINGROUP, WIDTH, HEIGHT, @_framebuffer)
        ser.strln(string("VGA 8bpp driver started"))
        vga.fontaddress(fnt.baseaddr{})
        vga.fontscale(1)
        vga.fontsize(6, 8)
        vga.clear{}

    if mlx.startx(I2C_SCL, I2C_SDA, I2C_HZ)
        ser.strln(string("MLX90621 driver started"))
        mlx.defaults{}
        mlx.opmode(mlx#CONT)
        _mlx_adcres := 18                       ' Initial sensor settings
        _mlx_refrate := 32
        _mlx_adcref := 1
    else
        ser.strln(string("MLX90621 driver failed to start - halting"))
        time.msleep(5)
        vga.stop{}
        mlx.stop{}
        repeat

    _col_scl := 16
    _fw := 6
    _fh := 6
    _invert_x := 0
    cognew(cog_keyinput, @_keyinput_stack)
    _settings_changed := TRUE

PUB SetupPalette{} | i, r, g, b, c, d
' Set up palette
    d := 4
    r := g := b := c := 0
    repeat i from 0 to vga#MAX_COLOR
        case i
            0..7:                                           ' violet
                ifnot i // d                                ' Step color only every (d-1)
                    r += 1 <# 3
                    g := 0
                    b += 1 <# 3
            8..15:                                          ' blue
                ifnot i // d
                    r -= 1 #> 0
                    g := 0
                    b := b
            16..23:                                         ' cyan
                ifnot i // d
                    r := 0
                    g += 1 <# 3
                    b := b
            24..31:                                         ' green
                ifnot i // d
                    r := 0
                    g := g
                    b -= 1 #> 0
            32..39:                                         ' yellow
                ifnot i // d
                    r += 1 <# 3
                    g := g
                    b := b
            40..47:                                         ' red
                ifnot i // d
                    r := r
                    g -= 1 #> 0
                    b := 0
            48..55:                                         ' pink
                ifnot i // d
                    r := r
                    g += 1 <# 3
                    b += 1 <# 3
            56..62:                                         ' grey
                ifnot i // d
                    r -= 1 #> 0
                    g -= 1 #> 0
                    b -= 1 #> 0
            63:                                             ' white
                r := g := b := 3
        c := (r << 4) | (g << 2) | b
        _palette[i] := c
    _palette[0] := $00

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
