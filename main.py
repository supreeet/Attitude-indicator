from machine import Pin, I2C, SPI, PWM, ADC, UART, Timer
import time, framebuf, math
from bmp280 import *
from micropyGPS import MicropyGPS
from sx127x import SX127x
from examples import LoRaSender
from examples import LoRaReceiver

def rotate_coordinates(x, y, angle_degrees):
    # Convert angle from degrees to radians
    angle_radians = math.radians(angle_degrees)

    # Clockwise rotation formulas
    new_x = x * math.cos(angle_radians) + y * math.sin(angle_radians)
    new_y = -x * math.sin(angle_radians) + y * math.cos(angle_radians)

    return new_x, new_y


def math_conversion(a1,b1,a2,b2,rotation_angle,elevation):
    a1 = a1 - 120
    b1 = b1 - 120
    a2 = a2 - 120
    b2 = b2 - 120
    x1, y1 = rotate_coordinates(a1,b1,rotation_angle)
    x2, y2 = rotate_coordinates(a2,b2,rotation_angle)
    return int(x1+120),int(y1+90+elevation),int(x2+120),int(y2+90+elevation)

    
gps_module = UART(0, baudrate=9600, rx=Pin(1))
TIMEZONE = 5
my_gps = MicropyGPS(TIMEZONE)
def convert(parts):
    if (parts[0] == 0):
        return None
        
    data = parts[0]+(parts[1]/60.0)
    # parts[2] contain 'E' or 'W' or 'N' or 'S'
    if (parts[2] == 'S'):
        data = -data
    if (parts[2] == 'W'):
        data = -data

    data = '{0:.6f}'.format(data) # to 6 decimal places
    return str(data)


I2C_SDA = 6
I2C_SDL = 7

DC = 8
CS = 9
SCK = 10
MOSI = 11
RST = 12

Vbat_Pin = 29

buzzer = PWM(Pin(14))
buzzer.freq(600)
buzzer.duty_u16(0)

lcd_backlight = PWM(Pin(25))
lcd_backlight.freq(1000)
lcd_backlight.duty_u16(30000)

white_pwm = PWM(Pin(22)) 
white_pwm.freq(1000)
white_pwm.duty_u16(5)

green_pwm = PWM(Pin(18))
green_pwm.freq(1000)
green_pwm.duty_u16(65000)

orange_pwm = PWM(Pin(17))
orange_pwm.freq(1000)
orange_pwm.duty_u16(65000)

red_pwm = PWM(Pin(16))
red_pwm.freq(1000)
red_pwm.duty_u16(65000)

key1 = Pin(21,Pin.IN, Pin.PULL_UP)
key2 = Pin(20,Pin.IN, Pin.PULL_UP)
key3 = Pin(19,Pin.IN, Pin.PULL_UP)

key_debounce_timer = time.ticks_ms()
def key1_callback(pin):
    global key_debounce_timer
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 200:
        return
    time.sleep_ms(5)
    print("key 1")
    
    button_time = time.ticks_ms()
    while True:
        x = time.ticks_diff(time.ticks_ms(), button_time)
        if x > 10000:
            key_debounce_timer = time.ticks_ms()
            return
        if key1()==1:
            break
        
        if x > 200:

            if x < 3000:
                white_pwm.duty_u16((x-200)*20)
                
            if x > 4000:
                y = white_pwm.duty_u16()
                white_pwm.duty_u16(0)
                time.sleep_ms(300)
                white_pwm.duty_u16(y)
                time.sleep_ms(200)
                white_pwm.duty_u16(0)
                time.sleep_ms(200)
                white_pwm.duty_u16(65535)
                key_debounce_timer = time.ticks_ms()
                return
        else:
            white_pwm.duty_u16(3)
        
    if x < 2:
        key_debounce_timer = time.ticks_ms()
        return
#     if white_pwm.duty_u16() < 50001:
#         white_pwm.duty_u16(white_pwm.duty_u16() + 1000)
#     else:
#         white_pwm.duty_u16(0)
    key_debounce_timer = time.ticks_ms()
    
    
key1.irq(trigger=Pin.IRQ_FALLING, handler=key1_callback)

def key2_callback(pin):
    global key_debounce_timer
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 200:
        return
    time.sleep_ms(5)
    print("key 2")
    
    button_time = time.ticks_ms()
    while True:
        x = time.ticks_diff(time.ticks_ms(), button_time)
        if x > 4000:
            key_debounce_timer = time.ticks_ms()
            return
        if key2()==1:
            break
    if x < 2:
        key_debounce_timer = time.ticks_ms()
        return
    for i in range(1000,50,-25):
        buzzer.duty_u16(0)
        time.sleep_ms(i)
        buzzer.duty_u16(5000)
        time.sleep_ms(i)

    for i in range(50,0,-1):
        buzzer.duty_u16(0)
        time.sleep_ms(i)
        buzzer.duty_u16(5000)
        time.sleep_ms(i)
    time.sleep(1)
    buzzer.duty_u16(0)
    key_debounce_timer = time.ticks_ms()
    
key2.irq(trigger=Pin.IRQ_FALLING, handler=key2_callback)

def key3_callback(pin):
    global key_debounce_timer
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 200:
        return
    time.sleep_ms(5)
    print("key 3")
    
    button_time = time.ticks_ms()
    while True:
        x = time.ticks_diff(time.ticks_ms(), button_time)
        if x > 4000:
            key_debounce_timer = time.ticks_ms()
            return
        if key3()==1:
            break
    if x < 2:
        key_debounce_timer = time.ticks_ms()
        return
    key_debounce_timer = time.ticks_ms()
    
#key3.irq(trigger=Pin.IRQ_FALLING, handler=key3_callback)


bmp280_object = BMP280(I2C(1, sda = Pin(6), scl = Pin(7), freq = 400000),addr = 0x76, use_case = BMP280_CASE_WEATHER)

bmp280_object.power_mode = BMP280_POWER_NORMAL
bmp280_object.oversample = BMP280_OS_HIGH
bmp280_object.temp_os = BMP280_TEMP_OS_8
bmp280_object.press_os = BMP280_TEMP_OS_4
bmp280_object.standby = BMP280_STANDBY_250
bmp280_object.iir = BMP280_IIR_FILTER_2

def altitude_IBF(pressure):
    local_pressure = pressure    # Unit : hPa
    sea_level_pressure = 1015 # Unit : hPa
    pressure_ratio = local_pressure / sea_level_pressure
    altitude = 44330*(1-(pressure_ratio**(1/5.255)))
    return altitude

class LCD_1inch28(framebuf.FrameBuffer):
    def __init__(self):
        self.width = 240
        self.height = 240
        
        self.cs = Pin(CS,Pin.OUT)
        self.rst = Pin(RST,Pin.OUT)
        
        self.cs(1)
        self.spi = SPI(1,100_000_000,polarity=0, phase=0,sck=Pin(SCK),mosi=Pin(MOSI),miso=None)
        self.dc = Pin(DC,Pin.OUT)
        self.dc(1)
        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)
        self.init_display()
        
        self.red   =   0x07E0
        self.green =   0x001f
        self.blue  =   0xf800
        self.white =   0xffff
        
        self.fill(self.white)
        self.show()

        
    def write_cmd(self, cmd):
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(bytearray([buf]))
        self.cs(1)
        
    def init_display(self):
        """Initialize dispaly"""  
        self.rst(1)
        time.sleep(0.01)
        self.rst(0)
        time.sleep(0.01)
        self.rst(1)
        time.sleep(0.05)
        
        self.write_cmd(0xEF)
        self.write_cmd(0xEB)
        self.write_data(0x14) 
        
        self.write_cmd(0xFE) 
        self.write_cmd(0xEF) 

        self.write_cmd(0xEB)
        self.write_data(0x14) 

        self.write_cmd(0x84)
        self.write_data(0x40) 

        self.write_cmd(0x85)
        self.write_data(0xFF) 

        self.write_cmd(0x86)
        self.write_data(0xFF) 

        self.write_cmd(0x87)
        self.write_data(0xFF)

        self.write_cmd(0x88)
        self.write_data(0x0A)

        self.write_cmd(0x89)
        self.write_data(0x21) 

        self.write_cmd(0x8A)
        self.write_data(0x00) 

        self.write_cmd(0x8B)
        self.write_data(0x80) 

        self.write_cmd(0x8C)
        self.write_data(0x01) 

        self.write_cmd(0x8D)
        self.write_data(0x01) 

        self.write_cmd(0x8E)
        self.write_data(0xFF) 

        self.write_cmd(0x8F)
        self.write_data(0xFF) 


        self.write_cmd(0xB6)
        self.write_data(0x00)
        self.write_data(0x20)

        self.write_cmd(0x36)
        self.write_data(0x98)

        self.write_cmd(0x3A)
        self.write_data(0x05) 


        self.write_cmd(0x90)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x08) 

        self.write_cmd(0xBD)
        self.write_data(0x06)
        
        self.write_cmd(0xBC)
        self.write_data(0x00)

        self.write_cmd(0xFF)
        self.write_data(0x60)
        self.write_data(0x01)
        self.write_data(0x04)

        self.write_cmd(0xC3)
        self.write_data(0x13)
        self.write_cmd(0xC4)
        self.write_data(0x13)

        self.write_cmd(0xC9)
        self.write_data(0x22)

        self.write_cmd(0xBE)
        self.write_data(0x11) 

        self.write_cmd(0xE1)
        self.write_data(0x10)
        self.write_data(0x0E)

        self.write_cmd(0xDF)
        self.write_data(0x21)
        self.write_data(0x0c)
        self.write_data(0x02)

        self.write_cmd(0xF0)   
        self.write_data(0x45)
        self.write_data(0x09)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x26)
        self.write_data(0x2A)

        self.write_cmd(0xF1)    
        self.write_data(0x43)
        self.write_data(0x70)
        self.write_data(0x72)
        self.write_data(0x36)
        self.write_data(0x37)  
        self.write_data(0x6F)


        self.write_cmd(0xF2)   
        self.write_data(0x45)
        self.write_data(0x09)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x26)
        self.write_data(0x2A)

        self.write_cmd(0xF3)   
        self.write_data(0x43)
        self.write_data(0x70)
        self.write_data(0x72)
        self.write_data(0x36)
        self.write_data(0x37) 
        self.write_data(0x6F)

        self.write_cmd(0xED)
        self.write_data(0x1B) 
        self.write_data(0x0B) 

        self.write_cmd(0xAE)
        self.write_data(0x77)
        
        self.write_cmd(0xCD)
        self.write_data(0x63)


        self.write_cmd(0x70)
        self.write_data(0x07)
        self.write_data(0x07)
        self.write_data(0x04)
        self.write_data(0x0E) 
        self.write_data(0x0F) 
        self.write_data(0x09)
        self.write_data(0x07)
        self.write_data(0x08)
        self.write_data(0x03)

        self.write_cmd(0xE8)
        self.write_data(0x34)

        self.write_cmd(0x62)
        self.write_data(0x18)
        self.write_data(0x0D)
        self.write_data(0x71)
        self.write_data(0xED)
        self.write_data(0x70) 
        self.write_data(0x70)
        self.write_data(0x18)
        self.write_data(0x0F)
        self.write_data(0x71)
        self.write_data(0xEF)
        self.write_data(0x70) 
        self.write_data(0x70)

        self.write_cmd(0x63)
        self.write_data(0x18)
        self.write_data(0x11)
        self.write_data(0x71)
        self.write_data(0xF1)
        self.write_data(0x70) 
        self.write_data(0x70)
        self.write_data(0x18)
        self.write_data(0x13)
        self.write_data(0x71)
        self.write_data(0xF3)
        self.write_data(0x70) 
        self.write_data(0x70)

        self.write_cmd(0x64)
        self.write_data(0x28)
        self.write_data(0x29)
        self.write_data(0xF1)
        self.write_data(0x01)
        self.write_data(0xF1)
        self.write_data(0x00)
        self.write_data(0x07)

        self.write_cmd(0x66)
        self.write_data(0x3C)
        self.write_data(0x00)
        self.write_data(0xCD)
        self.write_data(0x67)
        self.write_data(0x45)
        self.write_data(0x45)
        self.write_data(0x10)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)

        self.write_cmd(0x67)
        self.write_data(0x00)
        self.write_data(0x3C)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x01)
        self.write_data(0x54)
        self.write_data(0x10)
        self.write_data(0x32)
        self.write_data(0x98)

        self.write_cmd(0x74)
        self.write_data(0x10)
        self.write_data(0x85)
        self.write_data(0x80)
        self.write_data(0x00) 
        self.write_data(0x00) 
        self.write_data(0x4E)
        self.write_data(0x00)
        
        self.write_cmd(0x98)
        self.write_data(0x3e)
        self.write_data(0x07)

        self.write_cmd(0x35)
        self.write_cmd(0x21)

        self.write_cmd(0x11)
        time.sleep(0.12)
        self.write_cmd(0x29)
        time.sleep(0.02)
        
        self.write_cmd(0x21)

        self.write_cmd(0x11)

        self.write_cmd(0x29)

    def show(self):
        self.write_cmd(0x2A)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xef)
        
        self.write_cmd(0x2B)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xEF)
        
        self.write_cmd(0x2C)
        
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)


class QMI8658(object):
    def __init__(self,address=0X6B):
        self._address = address
        self._bus = I2C(id=1,scl=Pin(I2C_SDL),sda=Pin(I2C_SDA))
        bRet=self.WhoAmI()
        if bRet :
            self.Read_Revision()
        else    :
            return NULL
        self.Config_apply()

    def _read_byte(self,cmd):
        rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
        return rec[0]
    def _read_block(self, reg, length=1):
        rec=self._bus.readfrom_mem(int(self._address),int(reg),length)
        return rec
    def _read_u16(self,cmd):
        LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
        MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
        return (MSB[0] << 8) + LSB[0]
    def _write_byte(self,cmd,val):
        self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
        
    def WhoAmI(self):
        bRet=False
        if (0x05) == self._read_byte(0x00):
            bRet = True
        return bRet
    def Read_Revision(self):
        return self._read_byte(0x01)
    def Config_apply(self):
        # REG CTRL1
        self._write_byte(0x02,0x60)
        # REG CTRL2 : QMI8658AccRange_8g  and QMI8658AccOdr_1000Hz
        self._write_byte(0x03,0x23)
        # REG CTRL3 : QMI8658GyrRange_512dps and QMI8658GyrOdr_1000Hz
        self._write_byte(0x04,0x53)
        # REG CTRL4 : No
        self._write_byte(0x05,0x00)
        # REG CTRL5 : Enable Gyroscope And Accelerometer Low-Pass Filter 
        self._write_byte(0x06,0x11)
        # REG CTRL6 : Disables Motion on Demand.
        self._write_byte(0x07,0x00)
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        self._write_byte(0x08,0x03)

    def Read_Raw_XYZ(self):
        xyz=[0,0,0,0,0,0]
        raw_timestamp = self._read_block(0x30,3)
        raw_acc_xyz=self._read_block(0x35,6)
        raw_gyro_xyz=self._read_block(0x3b,6)
        raw_xyz=self._read_block(0x35,12)
        timestamp = (raw_timestamp[2]<<16)|(raw_timestamp[1]<<8)|(raw_timestamp[0])
        for i in range(6):
            # xyz[i]=(raw_acc_xyz[(i*2)+1]<<8)|(raw_acc_xyz[i*2])
            # xyz[i+3]=(raw_gyro_xyz[((i+3)*2)+1]<<8)|(raw_gyro_xyz[(i+3)*2])
            xyz[i] = (raw_xyz[(i*2)+1]<<8)|(raw_xyz[i*2])
            if xyz[i] >= 32767:
                xyz[i] = xyz[i]-65535
        return xyz
    def Read_XYZ(self):
        xyz=[0,0,0,0,0,0]
        raw_xyz=self.Read_Raw_XYZ()  
        #QMI8658AccRange_8g
        acc_lsb_div=(1<<12)
        #QMI8658GyrRange_512dps
        gyro_lsb_div = 64
        for i in range(3):
            xyz[i]=raw_xyz[i]/acc_lsb_div#(acc_lsb_div/1000.0)
            xyz[i+3]=raw_xyz[i+3]*1.0/gyro_lsb_div
        return xyz


def color565(red, green=0, blue=0):
    """
    Convert red, green and blue values (0-255) into a 16-bit 565 encoded color.
    """
    try:
        red, green, blue = red  # see if the first var is a tuple/list
    except TypeError:
        pass
    return (red & 0xf8) << 8 | (green & 0xfc) << 3 | blue >> 3
color1 = color565(20,20,20)
color2 = color565(0,28,100)
LCD = LCD_1inch28()


while True:
    if key3()==0:
        break
    LCD.fill(color565(0,0,0))
    length = gps_module.any()
    if length>0:
        b = gps_module.read(length)
        for x in b:
            msg = my_gps.update(chr(x))
    #_________________________________________________
    latitude = convert(my_gps.latitude)
    longitude = convert(my_gps.longitude)
    #_________________________________________________
    if (latitude == None and latitude == None):

        LCD.text("no data",100,10,LCD.white)
        LCD.show()
        continue
    #_________________________________________________
    t = my_gps.timestamp
    #t[0] => hours : t[1] => minutes : t[2] => seconds
    gpsTime = '{:02d}:{:02d}:{:02}'.format(t[0], t[1], t[2])
    
    gpsdate = my_gps.date_string('long')
    speed = my_gps.speed_string('kph') #'kph' or 'mph' or 'knot'
    #_________________________________________________
    print('Lat:', latitude)
    print('Lng:', longitude)
    print('time:', gpsTime)
    print('Date:', gpsdate)
    print('speed:', speed)
    LCD.text(latitude,35,40,LCD.white)
    LCD.text(longitude,28,50,LCD.white)
    LCD.text((gpsTime),20,60,LCD.white)
    LCD.text((speed),15,70,LCD.white)
    LCD.show()
    
lora_default = {
    'frequency': 469000000,
    'frequency_offset':0,
    'tx_power_level': 14,
    'signal_bandwidth': 125e3,
    'spreading_factor': 9,
    'coding_rate': 10,
    'preamble_length': 8,
    'implicitHeader': False,
    'sync_word': 0x12,
    'enable_CRC': True,
    'invert_IQ': False,
    'debug': False,
}

lora_pins = {
    'dio_0':27,
    'ss':5,
    'reset':15,
    'sck':2,
    'miso':4,
    'mosi':3,
}

lora_spi = SPI(0,
    baudrate=20_000_000, polarity=0, phase=0,
    bits=8, firstbit=SPI.MSB,
    sck=Pin(lora_pins['sck'], Pin.OUT, Pin.PULL_DOWN),
    mosi=Pin(lora_pins['mosi'], Pin.OUT, Pin.PULL_UP),
    miso=Pin(lora_pins['miso'], Pin.IN, Pin.PULL_UP),
)
lora = SX127x(lora_spi, pins=lora_pins, parameters=lora_default)


qmi8658=QMI8658()
Vbat= ADC(Pin(Vbat_Pin))   
temperature = bmp280_object.temperature
pressure_hPa = ( bmp280_object.pressure * 0.01 )
altitude = altitude_IBF(pressure_hPa)
    
    
lcd_start_time1 = time.ticks_ms()
x = 0
def main_loop(source):
    global lcd_start_time1, x
    x = x + 1
    lcd_start_time = time.ticks_ms()

    temperature = bmp280_object.temperature
    pressure_hPa = ( bmp280_object.pressure * 0.01 )
    altitude = altitude_IBF(pressure_hPa)
    
    xyz = qmi8658.Read_XYZ()
    sample = int(40 - (40*(xyz[2])))
    if xyz[0] < 0 :
        rotation_q = (85*(xyz[1]))-10
    if xyz[0] > 0 :
        rotation_q = 190 - (85*(xyz[1]))
    else:
        rotation_q = 85*(xyz[1])
        
    LCD.fill(color565(0,0,0))
    
    LCD.fill_rect(0,0,240,119,color565(80,235,129))
    LCD.fill_rect(0,119,240,2,LCD.white) 
    LCD.fill_rect(0,121,240,120,0x29)


    LCD.line(*math_conversion(45,105,105,105,rotation_q,sample), color2)
    LCD.line(*math_conversion(105,105,105,120,rotation_q,sample), color2)
    LCD.line(*math_conversion(105,120,135,120,rotation_q,sample), color2)
    LCD.line(*math_conversion(135,105,135,120,rotation_q,sample), color2)
    LCD.line(*math_conversion(135,105,195,105,rotation_q,sample), color2)

    LCD.line(*math_conversion(45,104,106,104,rotation_q,sample), color2)
    LCD.line(*math_conversion(106,104,106,119,rotation_q,sample), color2)
    LCD.line(*math_conversion(106,119,134,119,rotation_q,sample), color2)
    LCD.line(*math_conversion(134,119,134,104,rotation_q,sample), color2)
    LCD.line(*math_conversion(134,104,195,104,rotation_q,sample), color2)
    
    LCD.line(*math_conversion(45,106,104,106,rotation_q,sample), color2)
    LCD.line(*math_conversion(136,106,195,106,rotation_q,sample), color2)
    LCD.line(*math_conversion(45,103,104,103,rotation_q,sample), color2)
    LCD.line(*math_conversion(136,103,195,103,rotation_q,sample), color2)
    
    LCD.text(str("{:.0f}".format(altitude)) + "M",102,10,LCD.white)
    LCD.text(str("{:.1f}".format(temperature)) + "C",102,20,LCD.white)

    reading = Vbat.read_u16()*3.3/65535*2
    LCD.text("Vbat={:.2f}".format(reading),80,225,LCD.white)
    
    LCD.show()
    if time.ticks_diff(time.ticks_ms(), lcd_start_time1)>1000:
        lcd_start_time1 = time.ticks_ms()
        print(x)
        x = 0
    
main_timer = Timer(period=100, mode=Timer.PERIODIC, callback=main_loop)
LoRaReceiver.receive(lora)
