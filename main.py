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

# GLOBAL VARIABLES
should_exit_loop = False

# CLASS DEFINITIONS
config = Config.Config('/data/options.json', 'server-config.json')
client = mqtt.Client()
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
    client.reinitialise(config.get("mqtt_base_topic"))
    msg.n("Connecting MQTT client '%s' to %s:%s" % (
    config.get("mqtt_base_topic"), config.get("mqtt_server"), config.get("mqtt_port")))
    try:
        client.on_message = on_message
        client.connect(config.get("mqtt_server"), config.get("mqtt_port"))
    except Exception as e:
        msg.e(e)
        msg.e("Failed connecting to the host! Make sure you put in the right hostname/IP!")
        msg.n("Closing program...")
        # Exit program!
        exit(2)

    # Send a status notification
    msg.n("Publishing status message.")
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
    should_exit_loop = True
    client.loop_stop()

    msg.n("Publishing status message.")
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
    client.subscribe(config.get("mqtt_base_topic") + "/mode")
    client.subscribe(config.get("mqtt_base_topic") + "/flowLimit")
    client.subscribe(config.get("mqtt_base_topic") + "/targetValvePos")
    client.subscribe(config.get("mqtt_base_topic") + "/reportInterval")


# Publish sensor data to MQTT
def publish_data(json_obj):
    # Get json data and publish it - If a certain key wasn't received,
    # it will not publish any data with that key (avoids sudden null payloads)
    attempt_publish_data(json_obj, "timeStamp")
    attempt_publish_data(json_obj, "airHumidity")
    attempt_publish_data(json_obj, "flowRate")
    attempt_publish_data(json_obj, "airTemperature")
    attempt_publish_data(json_obj, "enclosureTemperature")
    attempt_publish_data(json_obj, "movementDetected")
    attempt_publish_data(json_obj, "currentValvePos")
    attempt_publish_data(json_obj, "soilHumidity", jkey="avgSoilHumidity")

    attempt_publish_data(json_obj, "targetValvePos", "/get")
    attempt_publish_data(json_obj, "mode", "/get")
    attempt_publish_data(json_obj, "flowLimit", "/get")

    # soilHumidity Will need to be split up into sub values
    try:
        for i in range(0, len(json_obj["soilHumidity"])):
            client.publish(config.get("mqtt_base_topic") + "/soilHumidity/%s" % i, json_obj["soilHumidity"][i])
    except KeyError as e:
        msg.d("attempt_publish_data():  %s" % e)
        pass


# If the provided key exists, get the payload and publish it, otherwise ignore it and do not publish
def attempt_publish_data(json_obj, key, topic_suffix="", jkey=None):
    try:
        if jkey is None:
            client.publish("%s/%s%s" % (config.get("mqtt_base_topic"), key, topic_suffix), json_obj[key])
        else:
            client.publish("%s/%s%s" % (config.get("mqtt_base_topic"), key, topic_suffix), json_obj[jkey])
    except KeyError as e:
        msg.d("attempt_publish_data():  %s" % e)
        pass


# Perform this function when a MQTT message has been received
def on_message(client, userdata, message):
    try:
        # Get and decode payload message
        payload = str(message.payload.decode("utf-8"))

        msg.d("MQTT -> This    Topic: %s    Payload: %s" % (message.topic, payload))

        # Get which message should be relayed (Note: Values below 0 will supposedly get ignored)
        output_msg = {"mode": -1, "flowLimit": -1, "targetValvePos": -1, "reportInterval": -1}

        if message.topic == config.get("mqtt_base_topic") + "/mode":
            output_msg["mode"] = str2bool(payload)
        elif message.topic == config.get("mqtt_base_topic") + "/flowLimit":
            output_msg["flowLimit"] = int(payload)
        elif message.topic == config.get("mqtt_base_topic") + "/targetValvePos":
            output_msg["targetValvePos"] = int(payload)
        elif message.topic == config.get("mqtt_base_topic") + "/reportInterval":
            output_msg["reportInterval"] = int(payload)
        else:
            msg.e("Unhandled Topic: %s" % message.topic)

        # Relay any matched message to Serial and make sure it's written properly
        output_json = json.dumps(output_msg)
        msg.d("This -> UART    %s" % str(output_json))
        serial_port.write(str(str(output_json) + "\n").encode('utf-8'))
        serial_port.flush()

    except Exception as e:
        print("on_message():  %s" % e)


# Convert string to boolean - Supports a variety of values for ease of use
def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1", "on", "enable")


# START THE MAIN LOOP - DON'T TOUCH
if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    stop()
