# TinyFloodCamML OpenMV code - By: ebgoldstein - Mon Oct 2nd 2023

import sensor, image, time, pyb, tf, os
from pyb import UART
from pyb import RTC

#setup LEDs and set into known off state
redLED   = pyb.LED(1)
blueLED = pyb.LED(3)
greenLED = pyb.LED(2)

#setup RTC when deploying
# (year, month, day, weekday, hours, minutes, seconds, subseconds)
# Monday = 1
rtc = RTC()
rtc.datetime((2024, 1, 29, 1, 18, 55, 0, 0))

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

#red light during setup
redLED.on()

#Load the TFlite model and the labels
net = tf.load('/MNv2Flood_cat.tflite', load_to_fb=True)
labels = ['Flood', 'NoFlood']

#turn led off when model is loaded
redLED.off()

# Always pass UART 3 for the UART number for your OpenMV Cam.
# The second argument is the UART baud rate. For a more advanced UART control
# example see the BLE-Shield driver.
uart = UART(3, 9600)

def getdatetime():

    # Extract the date and time from the RTC object.
    dateTime = rtc.datetime()
    year = str(dateTime[0])
    month = "%02d" % dateTime[1]
    day = "%02d" % dateTime[2]
    hour = "%02d" % dateTime[4]
    minute = "%02d" % dateTime[5]
    second = "%02d" % dateTime[6]
    newName = "I" + year + month + day+ '_' + hour + minute + second
    return newName

def saveimage(file_name):
    if not "images" in os.listdir():
        os.mkdir("images")  # Make a temp directory

    img.save('images/' + file_name, quality=90)

#MAIN LOOP

while(True):
    img = sensor.snapshot()

    #Do the classification and get the object returned by the inference.
    TF_objs = net.classify(img)
    #print(TF_objs)

    #The object has a output, which is a list of classifcation scores
    #for each of the output channels. this model only has 2 (flood, no flood).
    Flood = TF_objs[0].output()[0]
    NoFlood = TF_objs[0].output()[1]


    #This loop just prints to the serial terminal (and optionally the LCD screen),
    # but you could also blink an LED, write a number (or datetime), etc.

    if Flood > NoFlood:
        print('Flood')
        uart.write(bytes([1]))
        blueLED.on()
        file_name =  getdatetime() + "_FLOOD"
    else:
        print('No Flood')
        uart.write(bytes([0]))
        greenLED.on()
        file_name = getdatetime() +  "_NOFLOOD"

    print(file_name)
    saveimage(file_name)

    time.sleep(1)
    blueLED.off()
    greenLED.off()

    time.sleep(360)



