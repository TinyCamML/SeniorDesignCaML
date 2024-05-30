# ML Camera
This is *almost* all the software needed to get a Machine Learning Sunny Day Flooding Camera Sensor up and running besides the [sparrow-lora](https://github.com/blues/sparrow-lora) and [note-c](https://github.com/blues/note-c) reference node firmware created by Blues. <br>
Exact implementation and build details for the camera can be found in our [google docs](https://docs.google.com/document/d/1LVNdJl5ZswD-HA9_FXcrh3cxxb9wmkbudHEnJVct6j8/edit?usp=sharing). This readme gives a brief overview of what each of these folders and files contains.

### Deepsleep
The main.py in this folder is what is running on all cameras. It relies on rtc_time.txt, high_or_low.txt, and the tflite model. Keeps track of real time, takes then saves images, runs saved images through the tflite model to determine if there is a flood or no flood, sends flood or no flood signal through GPIO, then enters deepsleep mode to save power.

### Sleep
Does many of the same things as above but does not enter deep sleep, however we kept it in case you need to see console during testing.

### Firmware
The firmware folder holds the files we changed from sparrow-lora which need to be replaced after you clone sparrow-lora onto your machine.

### Model
A Tensorflow lite machine learning model which the camera uses to determine if the images it takes have flooding or no flooding in them. Currently being updated by the NCSU Biosystems Analytics Lab.

### JSON Script
This ipynb script takes JSON data from notehub and prints graphs. Download data from notehub to use.

### high_or_low.txt
A text file with simply a 0 or 1 bit in it, used for example usage on the camera.

### rtc_time.txt
A text file that holds the time which the camera uses when it initializes.
