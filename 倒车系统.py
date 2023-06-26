#coding=utf-8
#导入一些必要的包
import RPi.GPIO as GPIO  # 导入控制GPIO的模块，RPi.GPIO
import time
import smbus
import PCF8591 as ADC
from test_LCD1602 import LCD_1602
import cv2


#超声波传感器管脚号
TrigPin = 29
EchoPin = 31

# 定义红、绿两种颜色的引脚
redPin = 11
greenPin = 12

#有源蜂鸣器管脚号
Buzzer = 15

#液晶显示屏
Address=0x27
BUS=smbus.SMBus(1)
bl=1

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

#*************以下为超声波模块*********************
#超声波传感器初始化
def csb_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(TrigPin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(EchoPin, GPIO.IN)
#返回距离
def distance():
    # 发送触发信号
    GPIO.output(TrigPin, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TrigPin, GPIO.LOW)
    # 检测信号回响值
    while not GPIO.input(EchoPin):
        pass
    t1 = time.time()
    while GPIO.input(EchoPin):
        pass
    t2 = time.time()
    # 计算距离
    dis = round((t2 - t1) * 340 / 2, 2)
    return dis

#********************以下为双色LED灯模块****************
#初始化双色LED灯的接口
def rgb_setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(redPin, GPIO.OUT)
    GPIO.setup(greenPin, GPIO.OUT)
#设置双色LED灯的颜色值
def setColor(r_val, g_val):
    GPIO.output(redPin, r_val)
    GPIO.output(greenPin, g_val)
#灯灭
def rgb_off():
    GPIO.output(redPin, GPIO.LOW)
    GPIO.output(greenPin, GPIO.LOW)
def rgb_destroy():
    rgb_off()
    GPIO.cleanup()
#***********************以下为有源蜂鸣器模块***************
#初始化有源蜂鸣器
def bee_setup(pin):
    global BuzzerPin
    BuzzerPin = pin
    GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
    GPIO.setup(BuzzerPin, GPIO.OUT)
    GPIO.output(BuzzerPin, GPIO.HIGH)
def on():
    GPIO.output(BuzzerPin, GPIO.LOW)
    #低电平是响
def off():
    GPIO.output(BuzzerPin, GPIO.HIGH)
    #高电平是停止响
def beep(x):    #响3秒后停止3秒
    on()
    time.sleep(x)
    off()
    time.sleep(x)
def bee_destroy():#结束
    GPIO.output(BuzzerPin, GPIO.HIGH)
    GPIO.cleanup()

#*************以下为PCF8591模块*********************
bus = smbus.SMBus(1)

# 通过 sudo i2cdetect -y -1 可以获取到IIC的地址
def pc_setup(Addr):
    global address
    address = Addr
# 读取模拟量信息
def read(chn):  # 通道选择，范围是0-3之间
    try:
        if chn == 0:
            bus.write_byte(address, 0x40)
        if chn == 1:
            bus.write_byte(address, 0x41)
        if chn == 2:
            bus.write_byte(address, 0x42)
        if chn == 3:
            bus.write_byte(address, 0x43)
        if chn == 4:
            bus.write_byte(address, 0x44)
        bus.read_byte(address)  # 开始进行读取转换
    except Exception as e:
        print("Address: %s" % address)
        print(e)
    return bus.read_byte(address)

# 模块输出模拟量控制，范围为0-255
def write(val):
    try:
        temp = val  # 将数值赋给temmp 变量
        temp = int(temp)  # 将字符串转换为整型
        # 在终端上打印temp以查看，否则将注释掉
        bus.write_byte_data(address, 0x40, temp)
    except Exception as e:
        print("Error: Device address: 0x%2X" % address)
        print(e)

#******************以下为遥感器模块********************
def yg_setup():
    ADC.setup(0x48)  # Setup PCF8591
    global state
def direction():  # 获取操纵杆方向结果
    state = ['开始', '退出', '距离下限降低slow', '距离下限降低quick', '距离下限升高slow', '距离下限升高quick']
    i = 0 #0是sw,1是y,2是x
    #print(ADC.read(0),ADC.read(1),ADC.read(2))
    if (ADC.read(0) <=210 and ADC.read(0) >=190) and (ADC.read(1) <=210 and ADC.read(1) >=190) and (ADC.read(2) <=140 and ADC.read(2) >=120):
        i = 0  #start
    if ADC.read(0) <=10 and (ADC.read(1) <=210 and ADC.read(1) >=190) and (ADC.read(2) <=140 and ADC.read(2) >=120):
        i = 1  #按下sw按钮，结束
    if (ADC.read(0) <= 210 and ADC.read(0) >= 190) and ADC.read(1) >= 240 and (ADC.read(2) <= 140 and ADC.read(2) >= 120):
        i = 2  #向上拨动操作杆，距离下限降低slow
    if (ADC.read(0) <= 210 and ADC.read(0) >= 190) and ADC.read(1) <= 10 and (ADC.read(2) <= 140 and ADC.read(2) >= 120):
        i = 3  # 向下拨动操作杆，距离下限降低quick
    if (ADC.read(0) <= 210 and ADC.read(0) >= 190) and (ADC.read(1) <=210 and ADC.read(1) >=190) and ADC.read(2) >= 240:
        i = 4  #向左拨动操作杆，距离上限降低slow
    if (ADC.read(0) <= 210 and ADC.read(0) >= 190) and (ADC.read(1) <=210 and ADC.read(1) >=190) and ADC.read(2) <= 10:
        i = 5  # 向下拨动操作杆，距离上限降低quick
    return state[i]
def yg_destroy():
    pass

#****************以下为主程序*********************
m_lcd = LCD_1602(Address=0x27, bus_id=1, bl=1)

def setup_all():
    m_lcd.clear()
    bee_setup(Buzzer)#初始化有源蜂鸣器
    rgb_setup()#初始化三色LED灯
    m_lcd.lcd_init()#初始化液晶显示屏
    csb_setup()#初始化超声波传感器
    yg_setup()#初始化遥感器
    global lowl
    lowl = 0.5
#调整距离最小限制'开始', '距离下限降低', '距离下限升高'
def edge():
    global lowl
    dir = direction()
    if dir == '退出':  # 当按下摇杆时，程序退出
        m_lcd.clear()
        destroy()
        quit()
    if dir == '距离下限降低slow' and lowl >0.21:  # 下限值不低于0.2
        lowl -= 0.01
    if dir == '距离下限降低quick' and lowl >0.25:  # 下限值不低于0.2
        lowl -= 0.05
    if dir == '距离下限升高slow' and lowl < 0.99:  # 保证下限值不超过1
        lowl += 0.01
    if dir == '距离下限升高quick' and lowl < 0.95:  # 保证下限值不超过1
        lowl += 0.05

def loop():
    #打开摄像头
    cap = cv2.VideoCapture(0)
    #haarcascade_fullbody.xml是从网上下载的检测行人的模型
    body_classifier = cv2.CascadeClassifier('haarcascade_fullbody.xml')
    while True:
        #获取每一帧的图像
        success, img = cap.read()
        edge()# 获取障碍物距离dis
        print('The lower limit of distance : ', lowl)
        dis = distance()
        if success:
            #在图像上添加文字
            text = "distance:%s" % (dis)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            thickness = 2
            #在图像上显示距离障碍物的距离
            img = cv2.putText(img, text, (50, 50), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
            gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            bodies = body_classifier.detectMultiScale(gray, 1.2, 3)
            #如果检测到车辆后方有行人，则将行人用黄色方框框出来并在图像中提示"Be careful of pedestrians!!"
            for (x, y, w, h) in bodies:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
                img = cv2.putText(img, "Be careful of pedestrians!!", (50, 100), font, font_scale, (0,0,255), thickness, cv2.LINE_AA)
            #显示图像
            cv2.imshow("video", img)
        #按下键盘上的'q'键，停止程序
        if cv2.waitKey(1) == ord('q'):
            destroy()
            break
        print(dis)
        print('Current distance : ', dis)
        text = "distance:%s"%(dis)
        m_lcd.lcd_display_string(0, 0, text)
        if dis < lowl:
            setColor(1,0)  # 距离超下限时LED灯显红色
            for i in range(0, 3):
                beep(0.5)  # 蜂鸣3次，每次0.5秒
        if dis >=lowl:
            setColor(0, 1)#距离未超下限时LED灯显绿色


def destroy():
    m_lcd.clear()
    bee_destroy()
    rgb_destroy()
    yg_destroy()
    GPIO.cleanup()


if __name__ == "__main__":
    try:
        setup_all()
        loop()
    except KeyboardInterrupt:
        destroy()
        cap.release()



