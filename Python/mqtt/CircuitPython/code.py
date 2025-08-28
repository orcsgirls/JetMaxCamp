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
mqtt_broker = getenv("MQTT_BROKER")
mqtt_broker_port = getenv("MQTT_PORT")
device_id = getenv("DEVICE_ID")

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
conveyor_forward = digitalio.DigitalInOut(board.GP20)
conveyor_forward.direction = digitalio.Direction.OUTPUT
conveyor_backward = digitalio.DigitalInOut(board.GP21)
conveyor_backward.direction = digitalio.Direction.OUTPUT

# Switch
switch_left = digitalio.DigitalInOut(board.GP18)
switch_left.switch_to_input(pull=digitalio.Pull.UP)
switch_right = digitalio.DigitalInOut(board.GP19)
switch_right.switch_to_input(pull=digitalio.Pull.UP)

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
# Check manual
#---------------------------------------------------------------------------------

def check_manual(manual_forward, manual_backward):
    if(manual_forward != (not switch_left.value)) or (manual_backward != (not switch_right.value)):

        manual_forward = not switch_left.value
        manual_backward = not switch_right.value

        if manual_forward:
            screen_update(f"{device_id}: M<<", "green")
            led.value = True
            conveyor_forward.value = True
            conveyor_backward.value = False
            mqtt_client.publish(mqtt_topic_running, "manual forward")
        elif manual_backward:
            screen_update(f"{device_id}: M>>", "green")
            led.value = True
            conveyor_backward.value = True
            conveyor_forward.value = False
            mqtt_client.publish(mqtt_topic_running, "manual backward")
        else:
            led.value = False
            conveyor_forward.value = False
            conveyor_backward.value = False
            screen_update(f"{device_id}: OFF", "green")
            mqtt_client.publish(mqtt_topic_running, "stopped")

    return manual_forward, manual_backward

#---------------------------------------------------------------------------------
# MQTT Callback
#---------------------------------------------------------------------------------

def message(client, topic, message):

    # Action commands
    if(topic == mqtt_topic_action):
        if(manual_forward or manual_backward):
            mqtt_client.publish(mqtt_topic_running, "manual")
        else:
            if(message == 'forward'):
                screen_update(f"{device_id}: <<<", "red")
                led.value = True
                conveyor_forward.value = True
                conveyor_backward.value = False
                mqtt_client.publish(mqtt_topic_running, "forward")
            elif(message == 'backward'):
                screen_update(f"{device_id}: >>>", "red")
                led.value = True
                conveyor_backward.value = True
                conveyor_forward.value = False
                mqtt_client.publish(mqtt_topic_running, "backward")
            elif(message == 'stop'):
                screen_update(f"{device_id}: OFF", "yellow")
                led.value = False
                conveyor_forward.value = False
                conveyor_backward.value = False
                mqtt_client.publish(mqtt_topic_running, "stopped")

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

manual_forward = False
manual_backward = False

try:
    while True:
        manual_forward, manual_backward = check_manual(manual_forward, manual_backward)
        if not(manual_forward or manual_backward):
            mqtt_client.loop()

except KeyboardInterrupt:
    mqtt_client.unsubscribe(mqtt_topic)
    mqtt_client.disconnect()
    screen.set_css_colour("black")
