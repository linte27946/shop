#coding=utf-8
import time
import smbus
import RPi.GPIO as GPIO  # 测试

# 命令字
LCD_CLEARDISPLAY = 0x01  # 清屏
LCD_RETURNHOME = 0x02  # 光标复位

LCD_SETCGRAMADDR = 0x40  # 字符发生器地址
LCD_SETDDRAMADDR = 0x80  # 显示数据存储器地址

# 输入方式控制
LCD_ENTRYMODESET = 0x04  # 输入方式标志位
LCD_ENTRYRIGHT = 0x00  # 输入新数据光标右移动
LCD_ENTRYLEFT = 0x02  # 输入新数据光标左移动
LCD_ENTRYSHIFTINCREMENT = 0x01  # 显示移动
LCD_ENTRYSHIFTDECREMENT = 0x00  # 不显示移动

# 显示开关控制
LCD_DISPLAYCONTROL = 0x08  # 显示开关控制标志位
LCD_DISPLAYON = 0x04  # 整体显示开
LCD_DISPLAYOFF = 0x00  # 整体显示关
LCD_CURSORON = 0x02  # 光标开
LCD_CURSOROFF = 0x00  # 光标关
LCD_BLINKON = 0x01  # 闪烁开
LCD_BLINKOFF = 0x00  # 闪烁关

# 光标或字符移位控制
LCD_CURSORSHIFT = 0x10  # 光标、字符移位标志位
LCD_DISPLAYMOVE = 0x08  # 移动显示文字
LCD_CURSORMOVE = 0x00  # 移动光标
LCD_MOVERIGHT = 0x04  # 右移
LCD_MOVELEFT = 0x00  # 左移

# 功能设置
LCD_FUNCTIONSET = 0x20  # 功能设置标志位
LCD_8BITMODE = 0x10  # 8总线模式
LCD_4BITMODE = 0x00  # 4总线模式
LCD_2LINE = 0x08  # 2行显示
LCD_1LINE = 0x00  # 1行显示
LCD_5x10DOTS = 0x04  # 5x10 字符点阵
LCD_5x8DOTS = 0x00  # 5x8字符点阵


class LCD_1602(object):
    # 初始化 Address: I2C 芯片物理地址 bus_id 总线编号 bl 是非开背光
    def __init__(self, Address=0x27, bus_id=1, bl=1):  # bl=1表示打开背光灯
        self.bus_id = bus_id
        self.Address = Address
        self.BUS = smbus.SMBus(self.bus_id)
        self.bl = bl

    # 设置背景光
    def set_BL(self, bl):
        self.bl = bl

    def write_word(self, data):
        # 根据背景光标志，设置写数据方式
        temp = data
        if self.bl == 1:
            temp |= 0x08
        else:
            temp &= 0xF7
        self.BUS.write_byte(self.Address, temp)

    # 发送命令字
    def send_command(self, comm):
        # 先送高4位
        buf = comm & 0xF0
        buf |= 0x04  # RS = 0, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)

        # 再送第4位
        buf = (comm & 0x0F) << 4
        buf |= 0x04  # RS = 0, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)

    # 发送数据
    def send_data(self, data):
        # 先送高4位
        buf = data & 0xF0
        buf |= 0x05  # RS = 1, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)

        # 再送低四位
        buf = (data & 0x0F) << 4
        buf |= 0x05  # RS = 1, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB  # Make EN = 0
        self.write_word(buf)

    # LCD 初始化
    def lcd_init(self):
        try:

            # 先初始化为 8总线模式 光标复位 清屏
            self.send_command(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_CLEARDISPLAY | LCD_RETURNHOME)
            time.sleep(0.005)
            # 初始化为 8总线模式 光标复位
            self.send_command(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_RETURNHOME)
            time.sleep(0.005)

            # 4总线模式 2行显示 5x8点阵
            self.send_command(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS)
            time.sleep(0.005)

            # 设置数据进入方式
            self.lcd_inputset(bDirection=False, bShift=False)

            # 显示方式设置 整体显示开 、 光标关 、闪烁关
            self.lcd_displaySwitch(bDisplay=True, bCursor=False, bBlink=False)

            # 清屏
            self.clear()

            # 背光打开
            self.openlight()

        except:
            return False
        else:
            return True

    # 清屏
    def clear(self):
        self.send_command(LCD_CLEARDISPLAY)
        time.sleep(0.005)

    # 打开背光
    def openlight(self):
        self.BUS.write_byte(self.Address, 0x08)
        time.sleep(0.005)

    # 设置光标的位置  x 列  y 行
    def lcd_set_cursor(self, x, y):
        if x < 0:
            x = 0
        if x > 15:
            x = 15
        if y < 0:
            y = 0
        if y > 1:
            y = 1

        # Move cursor
        addr = 0x80 + 0x40 * y + x
        self.send_command(addr)

    # 在指定位置输出字符串
    def lcd_display_string(self, x, y, string):
        # 设置光标位置
        self.lcd_set_cursor(x, y)
        for ch in string:
            self.send_data(ord(ch))

    # 设置显示方式 bDisplay 是否开显示  bCursor 是否显示光标  bBlink 是否闪烁
    def lcd_displaySwitch(self, bDisplay=True, bCursor=False, bBlink=False):
        cmd = LCD_DISPLAYCONTROL | \
              (LCD_DISPLAYON if bDisplay else LCD_DISPLAYOFF) | \
              (LCD_CURSORON if bCursor else LCD_CURSOROFF) | \
              (LCD_BLINKON if bBlink else LCD_BLINKOFF)
        self.send_command(cmd)
        time.sleep(0.005)

    # 光标或字符移位控制 bTarget 1 文字移动 0 光标移动 bDirection 1 右移  0 左移
    def lcd_shit(self, bTarget, bDirection):
        cmd = LCD_CURSORSHIFT | \
              (LCD_DISPLAYMOVE if bTarget else LCD_CURSORMOVE) | \
              (LCD_MOVERIGHT if bDirection else LCD_MOVELEFT)
        self.send_command(cmd)
        time.sleep(0.005)

    # 输入移动方式设置
    def lcd_inputset(self, bDirection, bShift):
        cmd = LCD_ENTRYMODESET | \
              (LCD_ENTRYLEFT if bDirection else LCD_ENTRYRIGHT) | \
              (LCD_ENTRYSHIFTINCREMENT if bShift else LCD_ENTRYSHIFTDECREMENT)
        self.send_command(cmd)
        time.sleep(0.005)

    # 光标复位 指向显示数据的起始位置
    def lcd_cursorReturn(self):
        self.send_command(LCD_RETURNHOME)
        time.sleep(0.005)


if __name__ == "__main__":

    m_lcd = LCD_1602(Address=0x27, bus_id=1, bl=1)

    try:
        flag = m_lcd.lcd_init()
        print(flag)

        # 在指定位置显示字符串
        m_lcd.lcd_display_string(0, 0, 'Welcome!')
        m_lcd.send_data(0x7E)
        m_lcd.lcd_display_string(0, 1, 'Hello USTB')
        m_lcd.send_data(0xDF)
        time.sleep(5)

        # 设置光标位置，并改变该位置的光标显示方式
        m_lcd.lcd_set_cursor(10, 0)
        m_lcd.lcd_displaySwitch(bDisplay=True, bCursor=True, bBlink=True)
        time.sleep(5)

        # 向右移动光标
        m_lcd.lcd_shit(bTarget=0, bDirection=True)
        time.sleep(5)

        # 向右移动文字
        m_lcd.lcd_shit(bTarget=1, bDirection=True)
        time.sleep(2)

        # 光标复位 改变显示形式
        m_lcd.lcd_cursorReturn()
        m_lcd.lcd_displaySwitch(bDisplay=True, bCursor=True, bBlink=False)
        time.sleep(5)

        space = '                '
        greetings = 'Thank you for watching the lesson of Raspberry LCD! ^_^'
        greetings = space + greetings

        while True:
            tmp = greetings
            for i in range(0, len(greetings)):
                m_lcd.lcd_display_string(0, 0, tmp)
                tmp = tmp[1:]
                time.sleep(0.8)
                m_lcd.clear()

    except KeyboardInterrupt:
        pass




