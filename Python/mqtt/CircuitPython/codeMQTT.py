import time
import wifi
import socketpool
import board
import digitalio
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from os import getenv

#---------------------------------------------------------------------------------
# Settings
#---------------------------------------------------------------------------------

mqtt_broker = "192.168.37.111"
mqtt_topic = "control/conveyor"

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

conveyor = digitalio.DigitalInOut(board.GP1)
conveyor.direction = digitalio.Direction.OUTPUT

#---------------------------------------------------------------------------------
# MQTT Callback
#---------------------------------------------------------------------------------

def message(client, topic, message):
    if(topic == mqtt_topic):
        if(message == 'start'):
            print("Starting conveyor")
            led.value = True
            conveyor.value = True
        elif(message == 'stop'):
            print("Stopping conveyor")
            led.value = False
            conveyor.value = False
        else:
            print(f"ERROR: Invalid message {message}")

#---------------------------------------------------------------------------------
# Connecting to WiFi
#---------------------------------------------------------------------------------

ssid = getenv("CIRCUITPY_WIFI_SSID")
password = getenv("CIRCUITPY_WIFI_PASSWORD")

print(f"Connecting to WiFi {ssid}")
try:
    wifi.radio.connect(ssid, password)
    pool = socketpool.SocketPool(wifi.radio)
    print(f"Connected to WiFi {ssid}")
except TypeError:
    print(f"Could not connect to WiFi {ssid}")
    raise

#---------------------------------------------------------------------------------
# Setting up MQTT client
#---------------------------------------------------------------------------------

mqtt_client = MQTT.MQTT(broker=mqtt_broker, port=1883, socket_pool=pool)

# Connect callback handlers to client
mqtt_client.on_message = message

# Connect and subscribe to topic
mqtt_client.connect()
mqtt_client.subscribe(mqtt_topic)

#---------------------------------------------------------------------------------
# Main loop
#---------------------------------------------------------------------------------

try:
    while True:
        mqtt_client.loop()
except KeyboardInterrupt:
    mqtt_client.unsubscribe(mqtt_topic)
    mqtt_client.disconnect()
