from machine import Pin, SPI
import sdcard, uos, re 



cs = Pin(13, machine.Pin.OUT)
spi = SPI(0,
                  baudrate=1000000,
                  polarity=0,
                  phase=0,
                  bits=8,
                  firstbit=SPI.MSB,
                  sck=machine.Pin(2),
                  mosi=machine.Pin(3),
                  miso=machine.Pin(4))
try:
    sd = sdcard.SDCard(spi, cs)
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")

    sd_mount_point = '/sd'
except:
    sd_inserted = False
    
# Define a regular expression pattern to match "dataX.txt"
pattern = re.compile(r'data(\d+)\.txt')

# List all files on the SD card
try:
    file_list = uos.listdir(sd_mount_point)
    max_number = -1  # Initialize with a value that ensures any file number will be greater
    for file_name in file_list:
        match = pattern.match(file_name)
        if match:
            number = int(match.group(1))
            if number > max_number:
                max_number = number
    if max_number >= 0:
        highest_data_file = f"data{max_number}.txt"
        print(f"The highest 'data' file is: {highest_data_file}")
    else:
        print("No 'data' files found.")
except OSError as e:
    print("Error:", e)
    