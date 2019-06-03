"""
    TODO: 
        Maybe report current valve position back, as well as the other input (this -> arduino) commands
        Also report back if the valve has been fully opened or fully closed (Switches inside the valve)
"""

import paho.mqtt.client as mqtt
import json
import serial
import asyncio

import GracefulKiller
import SpecialMessages
import Config

# CONSTANTS
POLLING_RATE = 3  # How many times per second it should check for serial events

# Topics to subscribe to and relay to the serial output {Syntax = "topic_name" : data_type}
_subscribed_topics = {
    "targetValvePos0": int,
    "targetValvePos1": int,
    "reportInterval": int
}

# Topics to parse from the serial stream to publish to mqtt {Syntax = "topic_name" : {extra_data} OR None}
# Extra data keys:
#   - "from_key" : str (key to look for when recieving data)
#   - "getter" : bool (suffix topic with '/get')
#   - "as_array" : bool (parse key data as array, suffix topic with '/#', where # is the index)
_published_topics = {
    "timeStamp" : None,
    "airHumidity" : None,
    "airTemperature" : None,
    "enclosureTemperature" : None,
    
    "soilHumidity" : {
        "from_key": "avgSoilHumidity"
    },
    
    "targetValvePos" : {
        # "getter" : True,
        "as_array": True
    },
    
    "soilHumidity" : {
        "as_array" : True
    },
    
    "flowRate" : {
        "as_array" : True
    }
}

# GLOBAL VARIABLES
should_exit_loop = False

# CLASS DEFINITIONS
config = Config.Config('/data/options.json')
client = mqtt.Client(config.get("mqtt_base_topic"))
serial_port = serial.Serial()
killer = GracefulKiller.GracefulKiller()
msg = SpecialMessages.SpecialMessages(config.get("log_level"))


"""
    TOP FUNCTIONS
"""


# Program Main
async def main():
    msg.s("Starting up!")

    # Attempt connection to the serial device
    msg.n("Opening Serial port '%s'." % config.get("serial_port"))
    serial_port.port = config.get("serial_port")
    serial_port.baudrate = config.get("serial_baud")
    try:
        serial_port.open()
    except Exception as e:
        msg.e(e)
        msg.e("Failed connecting to the serial port! Make sure you connected to the right serial port!")
        msg.n("Closing program...")
        # Exit program!
        exit(1)

    # Attemt clearing the serial input buffer
    serial_port.reset_input_buffer()

    # Attempt connection to the MQTT broker
    msg.n("Connecting MQTT client '%s' to %s:%s" % (config.get("mqtt_base_topic"), config.get("mqtt_server"), config.get("mqtt_port")))
    try:
        client.on_message = on_message
        if config.get("mqtt_user") is not None:
            client.username_pw_set(config.get("mqtt_user"), password=config.get("mqtt_pass"))
            msg.i("User name set to %s" % config.get("mqtt_user"))
            msg.i("If no data seems to be sent to the server, make sure you are using the correct credentials first!")
        client.connect(config.get("mqtt_server"), config.get("mqtt_port"))
    except Exception as e:
        msg.e(e)
        msg.e("Failed connecting to the host! Make sure you put in the right hostname/IP!")
        msg.n("Closing program...")
        # Exit program!
        exit(2)

    # Send a status notification
    msg.n("Publishing status online message.")
    client.publish(config.get("mqtt_base_topic") + "/status", 1)

    # Subscribe to enable recieving specific keys
    subscribe_topics()

    # Loop - Don't worry, A SIGINT or SIGTERM will safely exit any loops (Hopefully! ;) )
    msg.s("Ready!")
    client.loop_start()
    await monitor_serial_input()


# Program Stop
def stop():
    msg.s("Shutting down!")
    client.loop_stop()

    msg.n("Publishing status offline message.")
    client.publish(config.get("mqtt_base_topic") + "/status", 0)

    msg.n("Disconnecting.")
    client.disconnect()

    # Making sure the Serial I/O buffer is cleared
    serial_port.flush()
    serial_port.reset_input_buffer()
    serial_port.close()
    msg.s("Program ended.")


# Monitor Serial input, parse it and then publish the values to MQTT
async def monitor_serial_input():
    while not killer.kill_now:
        try:
            if serial_port.inWaiting() > 0:
                # Read Serial input
                _data = serial_port.readline().decode("utf-8").rstrip()

                msg.d("UART -> This    " + _data)

                # Parse JSON and publish sensor data via MQTT
                try:
                    _j = json.loads(_data)

                    msg.d("This -> MQTT    Publishing data...")
                    publish_data(_j)
                    msg.d("This -> MQTT    Publishing done!")

                except ValueError as e:
                    msg.e("monitor_serial_input():  %s" % e)

        except UnicodeDecodeError as e:
            msg.e(e)

        await asyncio.sleep(1 / POLLING_RATE)


# Subscribe to topics
def subscribe_topics():
    for topic in _subscribed_topics.keys():
        client.subscribe("%s/%s" % (config.get("mqtt_base_topic"), topic))


# Publish sensor data to MQTT
def publish_data(json_obj):
    # Iterate through _published_topics, relay json_obj keys to mqtt
    for topic, extra_data in _published_topics.items():
        if extra_data is None:
            attempt_publish_data(json_obj, topic)
        else:
            getter = extra_data.get("getter", False)
            from_key = extra_data.get("from_key", None)
            as_array = extra_data.get("as_array", False)
            attempt_publish_data(json_obj, topic, getter, from_key, as_array)


# If the provided key exists, get the payload and publish it, otherwise ignore it and do not publish
def attempt_publish_data(json_obj, key, getter = False, from_key = None, as_array = False):
    try:
        # If getter is set to True, append '/get' to the topic
        topic_suffix = "/get" if getter else ""

        # Check if the key should be parsed as an array
        if not as_array:
            if from_key is None:
                client.publish("%s/%s%s" % (config.get("mqtt_base_topic"), key, topic_suffix), json_obj[key])
            else:
                client.publish("%s/%s%s" % (config.get("mqtt_base_topic"), key, topic_suffix), json_obj[from_key])
        else:
            # Iterate through the supplied array key
            for index in range(0, len(json_obj[key])):
                if from_key is None:
                    client.publish("%s/%s/%s%s" % (config.get("mqtt_base_topic"), key, index, topic_suffix), json_obj[key][index])
                else:
                    client.publish("%s/%s/%s%s" % (config.get("mqtt_base_topic"), key, index, topic_suffix), json_obj[from_key][index])
    except KeyError as e:
        msg.e("attempt_publish_data():  %s" % e)
        pass


# Perform this function when a MQTT message has been received
def on_message(client, userdata, message):
    try:
        # Get and decode payload message
        payload = str(message.payload.decode("utf-8"))

        msg.d("MQTT -> This    Topic: %s    Payload: %s" % (message.topic, payload))

        # Get which message should be relayed (Note: Values below 0 will supposedly get ignored)
        output_msg = {}

        for topic, data_type in _subscribed_topics.items():
            #output_msg[topic] = -1
            if message.topic == "%s/%s" % (config.get("mqtt_base_topic"), topic):
                if data_type is bool:
                    output_msg[topic] = str2bool(payload)
                elif data_type is int:
                    output_msg[topic] = int(payload)
                elif data_type is str:
                    output_msg[topic] = payload
                elif data_type is float:
                    output_msg[topic] = float(payload)
                else:
                    msg.e("Topic '%s' with payload '%s' could not be parsed due to expected type mismatch (expected %s)" % (topic, payload, data_type))
                    #output_msg[topic] = -1

        # Relay any matched message to Serial and make sure it's written properly
        output_json = json.dumps(output_msg)
        msg.d("This -> UART    %s" % str(output_json))
        serial_port.write(str(str(output_json) + "\n").encode('utf-8'))
        serial_port.flush()

    except Exception as e:
        msg.e("on_message():  %s" % e)


# Convert string to boolean - Supports a variety of values for ease of use
def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1", "on", "enable")


# START THE MAIN LOOP - DON'T TOUCH
if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    stop()
