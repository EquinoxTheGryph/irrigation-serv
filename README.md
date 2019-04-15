# irrigation-serv
Irrigation Manager - Made using Python with Docker support!  
Includes Arduino Sketch  
Made for Python 3  
Througout this readme I will be refering the part running the Python code as the *Host* and the part running the arduino sketch as the *Client*  
 
## Dependecies
This program depends on these libraries:
 - Asyncio
    - For Asyncronous operation between sending values to the broker and recieving serial data
 - PySerial
    - For communication between the host (Running this script) and the target arduino
 - PahoMQTT
    - For sending and receiving data between the host and the broker

## Installation (Host)
### Vanilla Python
 - Clone this repository to the destination of choice:  
   `git clone https://github.com/STcraft/irrigation-serv.git`

 - Then move into the downloaded repository:  
   `cd irrigation-serv`
 
 - To make instalation easier, first make sure the `python-pip` package is installed.

 - Install the required libraries:  
   `pip3 install -r requirements.txt`

 - Before running the script, Configure the program with `server-config.json`  
   See the topic below for more information.

 - To run the code:  
   `python3 main.py`  
   Note: Make sure the script is running using version 3.6 or above for the best experience.
 
### Docker
 - Build the image:  
   `docker build https://github.com/STcraft/irrigation-serv.git --tag 'irrigation-serv:master'`

 - Run the image:  
   `docker run --device=/dev/ttyUSB0 irrigation-serv`

### Hass.io (HomeAssistant)
 - Follow the guide documented here:  
   https://github.com/STcraft/hassio-irrigation-serv

## Installation (Client)
TODO!

## Configuration
TODO!

## MQTT Topics
Every topic string begins with the currently set `mqtt_base_topic` name (E.g. `irrigation/state`)
 - Read:
    - `/state`
    - `/timeStamp`
    - `/soilHumidity[/#]` (`'/#'` = index of sensor, Otherwise returns an avarage)
    - `/flowRate`
    - `/airTemperature`
    - `/enclosureTemperature`
    - `/airHumidity`
    - `/movementDetected`
    - `/currentValvePos`
- Write: (Most of these can also be read by adding '/get' at the end)
    - `/mode`
    - `/targetValvePos`
    - `/flowLimit`
    - `/reportRate`
