# TinyFloodCamML OpenMV code - By: ebgoldstein - Mon Oct 2nd 2023
# Modified by Christopher Little on Febraury 10, 2024

import sensor, image, time, pyb, tf, os, machine, utime
from pyb import UART
from pyb import Pin

def getdatetime():
    # Extract the date and time from the RTC object.
    dateTime = rtc.datetime()
    year = str(dateTime[0])
    month = "%02d" % dateTime[1]
    day = "%02d" % dateTime[2]
    hour = "%02d" % dateTime[4]
    minute = "%02d" % dateTime[5]
    second = "%02d" % dateTime[6]
    timestring = "I" + year + month + day+ '_' + hour + minute + second
    return timestring

def saveimage(file_name, image):
    if not "images" in os.listdir():
        os.mkdir("images")  # Make a temp directory

    image.save('images/' + file_name, quality=90)

def setdatetime():
    with open("rtc_time.txt", "r") as file:
        rtc_time_str = file.read().strip()
        rtc_time = (
            int(rtc_time_str[1:5]),  # year
            int(rtc_time_str[5:7]),  # month
            int(rtc_time_str[7:9]),  # day
            0,  # weekday (not used in setting RTC)
            int(rtc_time_str[10:12]),  # hours
            int(rtc_time_str[12:14]),  # minutes
            int(rtc_time_str[14:16]),  # seconds
            0  # subseconds (not used in setting RTC)
        )
        rtc.datetime(rtc_time)

def recoverdatetime():
    with open("rtc_time.txt", "r") as file:
        rtc_time_str = file.read().strip()
        rtc_time_int = [0] * 8
        rtc_time_int[0] = int(rtc_time_str[1:5]) # year
        rtc_time_int[1] = int(rtc_time_str[5:7]) # month
        rtc_time_int[2] = int(rtc_time_str[7:9]) # day
        rtc_time_int[4] = int(rtc_time_str[10:12]) # hours
        rtc_time_int[5] = int(rtc_time_str[12:14]) # minutes
        rtc_time_int[6] = int(rtc_time_str[14:16]) # seconds

        if((rtc_time_int[5] + 6) >= 60):
            rtc_time_int[4] = rtc_time_int[4] + 1
            rtc_time_int[5] = (rtc_time_int[5] + 6) - 60
        else:
            rtc_time_int[5] = rtc_time_int[5] + 6

        rtc_time = (
            rtc_time_int[0],  # year
            rtc_time_int[1],  # month
            rtc_time_int[2],  # day
            0,  # weekday (not used in setting RTC)
            rtc_time_int[4],  # hours
            rtc_time_int[5],  # minutes
            rtc_time_int[6],  # seconds
            0  # subseconds (not used in setting RTC)
        )
        rtc.datetime(rtc_time)

# Initialize Pin 9, set to either high or low based on flood or no flood
p9 = pyb.Pin("P9", pyb.Pin.OUT_PP)

try:
    with open("high_or_low.txt", "r") as file:
        pin_state = int(file.read().strip())
except OSError:
    # If the file doesn't exist or can't be read, default to low
    pin_state = 0

# Write the updated pin state to the file
with open("high_or_low.txt", "w") as file:
    file.write(str(int(not pin_state)))

# Setup LEDs and set into known off state
redLED   = pyb.LED(1)
blueLED = pyb.LED(3)
greenLED = pyb.LED(2)

# Setup RTC when deploying
rtc = machine.RTC()

# If reset_cause is 0 it's system startup, if reset_cause is 4 it's from RTC deep sleep
reset_cause = machine.reset_cause()
if(reset_cause==0):
    # (year, month, day, weekday, hours, minutes, seconds, subseconds)
    # (monday = 1)
    #rtc.datetime((2024, 2, 10, 6, 16, 50, 0, 0))
    setdatetime()
    curr_time = getdatetime()
    # Add code to create a txt file that puts the rtc time in it
    with open("rtc_time.txt", "w") as file:
        file.write(curr_time)

elif(reset_cause==4):
    curr_time = getdatetime()
    # Add code to modify text in text file to update rtc time
    with open("rtc_time.txt", "w") as file:
        file.write(curr_time)

elif(reset_cause==1):
    # Change above ^ to grab the text from the RTC file and set rtc again from it.
    recoverdatetime()
    curr_time = getdatetime()
    with open("rtc_time.txt", "w") as file:
        file.write(curr_time)

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

# Red light during setup
redLED.on()

# Load the TFlite model and the labels
net = tf.load('/MNv2Flood_cat.tflite', load_to_fb=True)
labels = ['Flood', 'NoFlood']

# Turn led off when model is loaded
redLED.off()

# Take a picture
img = sensor.snapshot()

# Do the classification and get the object returned by the inference.
TF_objs = net.classify(img)

# The object has an output, which is a list of classifcation scores
# for each of the output channels. this model only has 2 (flood, no flood).
Flood = TF_objs[0].output()[0]
NoFlood = TF_objs[0].output()[1]
time.sleep(1)

##Print to serial terminal and write to GPIO
#if Flood > NoFlood:
#    p9.high() # Set pin to high, indicating flood
#    print('Flood')
#    blueLED.on()
#    file_name = curr_time + "_FLOOD_" + str(reset_cause)
#else:
#    p9.low() # Set pin to low, indicating no flood
#    print('No Flood')
#    greenLED.on()
#    file_name = curr_time + "_NOFLOOD_" + str(reset_cause)

#Print to serial terminal and write to I2C
if pin_state:
    p9.high() # Set pin to high, indicating flood
    print('Flood')
    blueLED.on()
    file_name = curr_time + "_FLOOD_" + str(reset_cause)
else:
    p9.low() # Set pin to low, indicating no flood
    print('No Flood')
    greenLED.on()
    file_name = curr_time + "_NOFLOOD_" + str(reset_cause)


# Save file
#print(file_name)
saveimage(file_name, img)

time.sleep(30)
# Turn off LEDs
blueLED.off()
greenLED.off()

# Begin power down ------------------------------------
# Enable sensor softsleep
sensor.sleep(True)

# Shutdown the sensor (pulls PWDN high).
sensor.shutdown(True)

# Enable RTC interrupts every 6 minutes (in microseconds)
# camera will RESET after wakeup from deepsleep Mode.
rtc.wakeup(329 * 1000)

# Enter Deepsleep Mode (i.e. the OpenMV Cam effectively turns itself off except for the RTC).
machine.deepsleep()
