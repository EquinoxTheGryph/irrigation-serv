# irrigation-serv
Irrigation Manager - Made using Python with Docker support!  
Includes Arduino Sketch  
Made for Python 3  
Throughout this readme I will be refering the part running the Python code as the **Host** and the part running the arduino sketch as the **Client**


## Installation (Host)
### Dependencies
The host depends on these libraries:
 - Asyncio
    - For Asyncronous operation between sending values to the broker and recieving serial data
 - PySerial
    - For communication between the host (Running this script) and the target arduino
 - PahoMQTT
    - For sending and receiving data between the host and the broker
    
#### Vanilla Python
 - Clone this repository to the destination of choice:  
   `git clone https://github.com/STcraft/irrigation-serv.git`

 - Then move into the downloaded repository:  
   `cd irrigation-serv`
 
 - To make instalation easier, first make sure the `python-pip` package is installed.

 - Install the required libraries:
   `pip3 install -r requirements.txt`

 - To run the code: 
   `python3 main.py <PARAMETERS>` 
   Note: Make sure the script is running using Python version 3.6 or above for the best experience.
   For parameters, see the Configuration guide.
 
#### Docker
 - Build the image:
   `docker build https://github.com/STcraft/irrigation-serv.git --tag 'irrigation-serv:master'`

 - Run the image (Replace `/dev/ttyXXX` with your serial device.): 
   `docker run --device=/dev/ttyXXX irrigation-serv <PARAMETERS>`
   For parameters, see the Configuration guide.

#### Hass.io (HomeAssistant)
 - Follow the guide documented here:
   https://github.com/STcraft/hassio-irrigation-serv


## Installation (Client)
### Dependencies
The client depends on these libraries: (Can be found inside the library manager of the Arduino IDE)
 - `ArduinoJson` (Version >= 6.10.0)
    - Adds JSON support to the arduino (Makes parsing serial input easier)
 - `ADS1115`
    - Add support for I2C ADCs (Up to 4 devices)
 - *TODO!*
 
#### Arduino IDE
 - Make sure the dependencies are installed
 
 - Load the sketch (arduino_irrigation/arduino_irrigation.ino)
 
 - Flash the sketch to the designated board
 
 
## Configuration
TODO!


## MQTT Topics
Every topic string begins with the currently set `mqtt_base_topic` name (E.g. `irrigation/state`)
 - Read: 
    ('/#' = index of individual sensors)
    
    Name | Description
    --- | ---
    state | Program online state
    timeStamp | Timestamp for when the data has been gathered (since the arduino has booted)
    soilHumidity[/#] | Average/individual soil humidity value 
    flowRate[/#] | Combined/individual flow rate
    airTemperature | Greenhouse temperature value (in °C)
    enclosureTemperature | Enclosure temperature value (in °C)
    airHumidity | Greenhouse humidity value
    movementDetected | Movement detected since last scan
    currentValvePos | Estimated valve position (in %)
    
 - Write: 
    (Most of these can also be read by adding '/get' at the end)
    
    Name | Description | Payload Value
    --- | --- | ---
    mode | Valve control mode | 0 = Normal mode (set the valve position directly), 1 = Flow limit mode (Only let a set amount of flow through)
    targetValvePos/# | Set the valve's position at index # (When normal mode used) | Percentage value (%)
    flowLimit | Set the flow limit (TODO) | TODO (Haven't yet decided if i'm actually gonna implement this)
    reportRate | Set the report rate of the arduino client | Value in milliseconds. Value below <1000ms will reset to the default report rate
