from machine import Pin, I2C, SPI, PWM, Timer, UART, ADC
import time, framebuf
import gc9a01py as gc9a01
from fonts import vga1_8x8 as font1
from fonts import vga1_8x16 as font2
from fonts import vga1_16x32 as font3
from bmp280 import *

key1 = Pin(21,Pin.IN, Pin.PULL_UP)   # up
key2 = Pin(20,Pin.IN, Pin.PULL_UP)   # select
key3 = Pin(19,Pin.IN, Pin.PULL_UP)   # flash

white_pwm = PWM(Pin(22)) 
white_pwm.freq(1000)
white_pwm.duty_u16(3)

green_pwm = PWM(Pin(18))
green_pwm.freq(1000)
green_pwm.duty_u16(65500)

orange_pwm = PWM(Pin(17))
orange_pwm.freq(1000)
orange_pwm.duty_u16(65500)

red_pwm = PWM(Pin(16))
red_pwm.freq(1000)
red_pwm.duty_u16(65500)

lcd_backlight = PWM(Pin(25))
lcd_backlight.freq(1000)
lcd_backlight.duty_u16(20000)

key_debounce_timer = time.ticks_ms()

def key3_callback(pin):
    global key_debounce_timer
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 200:
        return
    time.sleep_ms(5)
    print("key 3")

    button_time = time.ticks_ms()
    while True:
        x = time.ticks_diff(time.ticks_ms(), button_time)
        
        if key3()==1:
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

    key_debounce_timer = time.ticks_ms()
    
key3.irq(trigger=Pin.IRQ_FALLING, handler=key3_callback)


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



vbat_pin = ADC(Pin(29))

spi = SPI(1, baudrate=40000000, sck=Pin(10), mosi=Pin(11))
LCD = gc9a01.GC9A01(
    spi,
    dc=Pin(8, Pin.OUT),
    cs=Pin(9, Pin.OUT),
    reset=Pin(12, Pin.OUT),
    backlight=Pin(25, Pin.OUT),
    rotation=1)


modes = [" GPS ", " INA219 ", " Lora ", " TEA5767 ", " I2C scan ",
         " mode 6 "," mode 7 ", " mode 8 "]

num_modes = len(modes)
selected_mode = 0

LCD.fill(gc9a01.BLACK)
start = time.ticks_ms()

key_debounce_timer = time.ticks_ms()
def key1_callback(pin):
    global selected_mode, key_debounce_timer
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 300:
        return
    time.sleep_ms(5)
    
    if selected_mode<7:
        selected_mode = selected_mode + 1
    else:
        selected_mode = 0
    print(modes[selected_mode])
    key_debounce_timer = time.ticks_ms()


key1.irq(trigger=Pin.IRQ_FALLING, handler=key1_callback)
mode_selection_complete = False
def key2_callback(pin):
    global key_debounce_timer, mode_selection_complete
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 300:
        return
    time.sleep_ms(5)
    print("mode confirmed")
    mode_selection_complete = True
    key_debounce_timer = time.ticks_ms()
    key1.irq(trigger=Pin.IRQ_FALLING, handler=None)

key2.irq(trigger=Pin.IRQ_FALLING, handler=key2_callback)

while True:
    vbat = vbat_pin.read_u16()*3.3/65535*2
    LCD.text(font1, "{:.2f}v".format(vbat),100,225,gc9a01.WHITE)
    LCD.text(font1, "{:.1f}".format(bmp280_object.temperature) + 'C ' + "{:.0f}".format(altitude_IBF(bmp280_object.pressure * 0.01)) + 'M', 80, 215,gc9a01.WHITE)

    for i in range(len(modes)):
        if i != selected_mode:
            background_text_color = gc9a01.WHITE
            foreground_text_color = gc9a01.BLACK
        else:
            background_text_color = gc9a01.BLACK
            foreground_text_color = gc9a01.WHITE
        
        LCD.text(font2, modes[i], 50, 30 + i*20,
             background_text_color,foreground_text_color)
    if mode_selection_complete == True:
        key2.irq(trigger=Pin.IRQ_FALLING, handler=None)
        print(modes[selected_mode], 'confirmed')
        break
    
LCD.fill(gc9a01.BLACK)
LCD.text(font2, modes[selected_mode], 90, 5,gc9a01.WHITE)

def key2_reset(pin):
    global key_debounce_timer, mode_selection_complete
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 300:
        return
    time.sleep_ms(5)
    button_time = time.ticks_ms()
    while True:
        x = time.ticks_diff(time.ticks_ms(), button_time)
        
        if key2()==1:
            break
        
        if x > 2000:
            machine.reset()
                
            
    if x < 2:
        key_debounce_timer = time.ticks_ms()
        return
    mode_selection_complete = True
    key_debounce_timer = time.ticks_ms()

key2.irq(trigger=Pin.IRQ_FALLING, handler=key2_reset)


if selected_mode == 0:
    from micropyGPS import MicropyGPS
    gps_module = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
    my_gps = MicropyGPS(5.5) # timezone
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

    while True:
        vbat = vbat_pin.read_u16()*3.3/65535*2
        LCD.text(font1,"{:.2f}v".format(vbat),100,225,gc9a01.WHITE)
        LCD.text(font1, "{:.1f}".format(bmp280_object.temperature) + 'C ' + "{:.0f}".format(altitude_IBF(bmp280_object.pressure * 0.01)) + 'M', 80, 215,gc9a01.WHITE)

        time.sleep_ms(100)
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
            print(' no data')
            LCD.text(font2, 'no data', 30, 65,gc9a01.WHITE)
            continue
        #_________________________________________________
        t = my_gps.timestamp
        #t[0] => hours : t[1] => minutes : t[2] => seconds
        gpsTime = '{:02d}:{:02d}:{:02}'.format(t[0], t[1], t[2])
        
        gpsdate = my_gps.date_string('long')
        speed = my_gps.speed_string('kph') #'kph' or 'mph' or 'knot'
        #_________________________________________________
        
        LCD.text(font2, latitude, 30, 65,gc9a01.WHITE)
        LCD.text(font2, longitude, 30, 85,gc9a01.WHITE)
        LCD.text(font2, gpsTime, 30, 105,gc9a01.WHITE)
        LCD.text(font2, gpsdate, 30, 125,gc9a01.WHITE)
        LCD.text(font2, speed, 30, 145,gc9a01.WHITE)
        
        print('Lat:', latitude)
        print('Lng:', longitude)
        print('time:', gpsTime)
        print('Date:', gpsdate)
        print('speed:', speed)


if selected_mode == 1:
    from ina219 import INA219
    from ina219 import DeviceRangeError
    ina_available = True
    try:
        ina = INA219(0.002,I2C(1, sda = Pin(6), scl = Pin(7)),address=0x40)
        ina.configure(ina.RANGE_32V)
    except Exception as e:
        print(e)
        LCD.text(font2, 'ina219 error', 30, 65,gc9a01.WHITE)
        ina_available = False
        
    wh=0
    v=5
    i=0
    p=0
    def watthour(source):
        global wh,v, i ,p
        try:
            v = ina.voltage()
            i = ina.current()/1000
            p = ina.power()/1000
            wh=wh+p/36000
        except DeviceRangeError as e:
            print(e)
            
    if ina_available == True:
        soc_timer = Timer(period=100, mode=Timer.PERIODIC, callback=watthour)
        last_soc_save = time.ticks_ms()

        while True:
            vbat = vbat_pin.read_u16()*3.3/65535*2
            LCD.text(font1,"{:.2f}v".format(vbat),100,225,gc9a01.WHITE)
            LCD.text(font1, "{:.1f}".format(bmp280_object.temperature) + 'C ' + "{:.0f}".format(altitude_IBF(bmp280_object.pressure * 0.01)) + 'M', 80, 215,gc9a01.WHITE)

            print("v: %.2f" % v ,", i: %.2f" % i )
            print("P: %.2f" % p , "wh: %.2f" % wh)
            
            LCD.text(font2,"v: %.2f" % v + ", i: %.2f " % i,42,40,gc9a01.WHITE)
            LCD.text(font2,"P: %.2f" % p + ", wh: %.2f " % wh,42,55,gc9a01.WHITE)

            if time.ticks_diff(time.ticks_ms(), last_soc_save)>30000:
                with open("ina219 log.txt", "w") as file:
                    file.write(str(wh))
                last_soc_save = time.ticks_ms()


freq_number = 0
freq_list = ['91.1', '92.7', '93.5', '95.0']
def radio_frequency(freq):
        freqB = 4 * (freq * 1000000 + 225000) / 32768
        tea5767.writeto(0x60, bytearray([int(freqB) >> 8, int(freqB) & 0XFF, 0X90, 0X1E, 0X00]))
def key1_frequency_up(pin):
    global freq_number, key_debounce_timer
    if time.ticks_diff(time.ticks_ms(), key_debounce_timer) < 300:
        return
    time.sleep_ms(5)
    button_time = time.ticks_ms()
    print('key 1')
    
    while True:
        x = time.ticks_diff(time.ticks_ms(), button_time)
        if key1()==1:
            break
        
        
    if x < 2:
        key_debounce_timer = time.ticks_ms()
        return
    
    if freq_number > 2:
        freq_number = 0
    
    else:
        freq_number = freq_number+1
        
    LCD.text(font2,'freq: ' + str(freq_list[freq_number]),42,55,gc9a01.WHITE)
    print(freq_list[freq_number])
    radio_frequency(float(freq_list[freq_number]))

    key_debounce_timer = time.ticks_ms()



if selected_mode == 3:
    tea_available = True

    try:
        tea5767 = I2C(1, scl=Pin(7), sda=Pin(6))
    except Exception as e:
        print(e)
        tea_available = False
    
    if tea_available == True: 
        key1.irq(trigger=Pin.IRQ_FALLING, handler=key1_frequency_up)
        try:
            radio_frequency(float(freq_list[freq_number]))
            LCD.text(font2,'freq: ' + str(freq_list[freq_number]),42,55,gc9a01.WHITE)
        except Exception as e:
            print(e)
            LCD.text(font2,'TEA5767 error',42,55,gc9a01.WHITE)
            
    while True:
        vbat = vbat_pin.read_u16()*3.3/65535*2
        LCD.text(font1,"{:.2f}v".format(vbat),100,225,gc9a01.WHITE)
        LCD.text(font1, "{:.1f}".format(bmp280_object.temperature) + 'C ' + "{:.0f}".format(altitude_IBF(bmp280_object.pressure * 0.01)) + 'M', 80, 215,gc9a01.WHITE)

if selected_mode == 4:
    i2c=I2C(1, sda=6, scl=7)
    def scan_show():
        
        LCD.text(font2, modes[selected_mode], 90, 5,gc9a01.WHITE)
        
        devices = i2c.scan()
        if len(devices) == 0:
            LCD.text(font2,'NO I2C DEVICE',30,38,gc9a01.WHITE)
        else:
            LCD.text(font2,'i2c devices:' + str(len(devices)),30,38,gc9a01.WHITE)
        
        for device in devices:
            LCD.text(font2,"Decimal:" + str(device) + " ,Hexa:" + str(hex(device)),30,55 + (18 * devices.index(device)),gc9a01.WHITE)
    scan_show()
    
    while True:
        if key1()==0:
            LCD.fill(gc9a01.BLACK)
            scan_show()
            time.sleep_ms(300)
        
        
        