import time
import wifi
import socketpool
import board
import digitalio
import busio
import pwmio
import supervisor
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from adafruit_motor import motor
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
pwm_a = pwmio.PWMOut(board.GP20, frequency=50)
pwm_b = pwmio.PWMOut(board.GP21, frequency=50)
conveyor = motor.DCMotor(pwm_a, pwm_b)
conveyor.decay_mode = motor.SLOW_DECAY
conveyor_speed = 0.8
conveyor_manual_speed = 0.6

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
        if wifi_connected:
            screen.update(f"{wifi.radio.ipv4_address}", message)
        else:
            screen.update(f"WiFi not connected", message)
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
            screen_update(f"{device_id}: M<<", "yellow")
            led.value = True
            conveyor.throttle = conveyor_manual_speed
            if mqtt_connected:
                mqtt_client.publish(mqtt_topic_running, "manual forward")
        elif manual_backward:
            screen_update(f"{device_id}: M>>", "yellow")
            led.value = True
            conveyor.throttle = -conveyor_manual_speed
            if mqtt_connected:
                mqtt_client.publish(mqtt_topic_running, "manual backward")
        else:
            led.value = False
            conveyor.throttle = 0.0
            if mqtt_connected:
                screen_update(f"{device_id}: OFF", "yellow")
                mqtt_client.publish(mqtt_topic_running, "stopped")
            else:
                screen_update("MQTT not found", "red")

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
            command = message.split(':')
            speed = float(command[1])/100. if len(command)>1 else conveyor_speed
            if(command[0] == 'forward'):
                screen_update(f"{device_id}: <<<", "red")
                led.value = True
                conveyor.throttle = speed
                mqtt_client.publish(mqtt_topic_running, "forward")
            elif(command[0] == 'backward'):
                screen_update(f"{device_id}: >>>", "red")
                led.value = True
                conveyor.throttle = -speed
                mqtt_client.publish(mqtt_topic_running, "backward")
            elif(command[0] == 'stop'):
                screen_update(f"{device_id}: OFF", "yellow")
                led.value = False
                conveyor.throttle = 0.0
                mqtt_client.publish(mqtt_topic_running, "stopped")

    # Config commands
    elif(topic == mqtt_topic_config):
        if(message == 'reset'):
            screen_update("RESET", "yellow")
            supervisor.reload()

#---------------------------------------------------------------------------------
# Connecting to WiFi
#---------------------------------------------------------------------------------

wifi_connected = False
mqtt_connected = False

screen_update("Connecting ..", "red")

try:
    wifi.radio.connect(ssid, password)
    print(f"Connected to WiFi {ssid}")
    wifi_connected = True
except ConnectionError:
    screen_update("WiFi failed", "red")
    print(f"Could not connect to WiFi {ssid}")

#---------------------------------------------------------------------------------
# Setting up MQTT client
#---------------------------------------------------------------------------------

if wifi_connected:
    screen_update("MQTT connecting ..", "red")

    pool = socketpool.SocketPool(wifi.radio)
    mqtt_client = MQTT.MQTT(broker=mqtt_broker, port=mqtt_broker_port, socket_pool=pool)
    mqtt_client.on_message = message

    try:
        mqtt_client.connect()
        mqtt_client.subscribe(mqtt_topic_action)
        mqtt_client.subscribe(mqtt_topic_config)
        screen_update(f"{device_id}: OFF", "yellow")
        mqtt_connected = True
    except:
        screen_update("MQTT not found", "red")

#---------------------------------------------------------------------------------
# Main loop
#---------------------------------------------------------------------------------

manual_forward = False
manual_backward = False

try:
    while True:
        manual_forward, manual_backward = check_manual(manual_forward, manual_backward)
        if not(manual_forward or manual_backward) and mqtt_connected:
            mqtt_client.loop()

except KeyboardInterrupt:
    mqtt_client.unsubscribe(mqtt_topic)
    mqtt_client.disconnect()
    screen.set_css_colour("black")
