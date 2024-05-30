# Spatial Extent of Sunny Day Flooding
NCSU ECE senior design project 2023-2024 for measuring the spatial extent of sunny day flooding.
This project aims to measure the spatial extent of sunny day flooding by implementing a 
system called "The Pole" consisting of three main components:
1. Gateway to communicate data from sensors to database
2. Camera Sensor to determine if street is flooded
3. Water Quality Sensor to determine if flood water is salt or fresh water

## Three Components:
### Gateway
The gateway recieves LoRa transmissions from both sensors and sends them over WiFi/Cellular to the [Sunny Day Flooding
Database](https://sunnydayflood.apps.cloudapps.unc.edu/).

### Camera Sensor
There are two main components to the spatial detection sensor. 
    1. OpenMV Cam H7
    2. Blues Sparrow Reference Board
These two components communicate with each other serially over UART. The OpenMV camera takes a picture and uses the 
on board machine learning algorithm to decide whether the surface is flooded. The OpenMV board will then send a 
1 for flood detected or 0 for no flood detected to the sparrow baord. The sparrow board will then send that data 
to the gateway over Long range radio (LoRa). 

### Water Quality Sensor
The water quality sensor uses a variety of sensors to determine a number of things about water quality such as
salinity, temperature, pressure, and more.

## Visualization
Here is an abstract breakdown of how our software and hardware components work together.
![Communications Software](https://media.github.ncsu.edu/user/19853/files/22d30442-cd83-49e3-a8f7-0315449f073f)

## Documentation
Additional documentation for hardware and firmware is available on Group 42's [Google Drive](https://docs.google.com/document/d/1LVNdJl5ZswD-HA9_FXcrh3cxxb9wmkbudHEnJVct6j8/edit?usp=sharing)

[How to build camera](https://docs.google.com/document/d/1JD7ymHJgXSGrHXuFFZmUTH-4FP1dzlTKBDabuKNdUmM/edit?usp=sharing)<br>
[How to build water quality sensor](https://docs.google.com/document/d/1j1SxQgsIZfERQFCrNCYzoqGrytbeL06MTDBeYCC6oAA/edit?usp=sharing)<br>
[How to build gateway](https://docs.google.com/document/d/1NmCvBrbC7-rqT89v1PB9AVDzkco4Czy6VmIGGNDywA0/edit?usp=sharing)

## Firmware
All firmware running on Blues Sparrow boards comes from [sparrow-lora](https://github.com/blues/sparrow-lora/) and [note-c](https://github.com/blues/note-c/), created by the Blues team. All firmware code within this Github are the files that have been changed from the base firmware and need to be replaced for their specific use case. To use the firmware, clone note-c and sparrow-lora as directed in [documentation](https://docs.google.com/document/d/1LVNdJl5ZswD-HA9_FXcrh3cxxb9wmkbudHEnJVct6j8/edit?usp=sharing).
