import time
import wifi
import socketpool
import board
import digitalio
import busio
import pwmio
import supervisor
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from rgb1602 import Screen
from os import getenv

#---------------------------------------------------------------------------------
# Settings
#---------------------------------------------------------------------------------

ssid = getenv("CIRCUITPY_WIFI_SSID")
password = getenv("CIRCUITPY_WIFI_PASSWORD")
device_id = getenv("DEVICE_ID")

mqtt_broker = "192.168.37.111"
mqtt_broker_port = 1883
mqtt_topic_action = device_id+"/action"
mqtt_topic_config = device_id+"/config"
mqtt_topic_running = device_id+"/running"

#---------------------------------------------------------------------------------

# On board LED
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Display
i2c = busio.I2C(board.GP13, board.GP12)
screen = Screen(i2c)

# Motor controller
conveyor = digitalio.DigitalInOut(board.GP1)
conveyor.direction = digitalio.Direction.OUTPUT

#---------------------------------------------------------------------------------
# LCD wrapper
#---------------------------------------------------------------------------------

def screen_update(message, color):
    try:
        screen.set_css_colour(color)
        screen.update(f"{wifi.radio.ipv4_address}", message)
    except:
        pass

#---------------------------------------------------------------------------------
# MQTT Callback
#---------------------------------------------------------------------------------

def message(client, topic, message):

    # Action commands
    if(topic == mqtt_topic_action):
        if(message == 'start'):
            screen_update(f"{device_id}: ON", "red")
            led.value = True
            conveyor.value = True
            mqtt_client.publish(mqtt_topic_running, "True")
        elif(message == 'stop'):
            screen_update(f"{device_id}: OFF", "yellow")
            led.value = False
            conveyor.value = False
            mqtt_client.publish(mqtt_topic_running, "False")

    # Config commands
    elif(topic == mqtt_topic_config):
        if(message == 'reset'):
            screen_update("RESET", "yellow")
            supervisor.reload()

#---------------------------------------------------------------------------------
# Connecting to WiFi
#---------------------------------------------------------------------------------

screen_update("Starting ..", "WiFi")

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

screen_update("Starting ..", "MQTT")

mqtt_client = MQTT.MQTT(broker=mqtt_broker, port=mqtt_broker_port, socket_pool=pool)

# Connect callback handlers to client
mqtt_client.on_message = message

# Connect and subscribe to topic
try:
    mqtt_client.connect()
    mqtt_client.subscribe(mqtt_topic_action)
    mqtt_client.subscribe(mqtt_topic_config)
    screen_update(f"{device_id}: OFF", "yellow")
except:
    screen_update("MQTT Error!!", "red")
    raise

#---------------------------------------------------------------------------------
# Main loop
#---------------------------------------------------------------------------------

try:
    while True:
        mqtt_client.loop()
except KeyboardInterrupt:
    mqtt_client.unsubscribe(mqtt_topic)
    mqtt_client.disconnect()
    screen.set_css_colour("black")
