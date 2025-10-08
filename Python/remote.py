# Remote control your jetmax using MQTT

from myjetmax import *
import paho.mqtt.client as mqtt
import socket
import time


# This routine is called when we reeive a message
def on_message(mqttc, userdata, message):
    if message.topic == device_id+"/action":
        if message.payload.decode('utf-8') == 'right':
            jetmax_right()
        elif message.payload.decode('utf-8') == 'left':
            jetmax_left()
        elif message.payload.decode('utf-8') == 'home':
            jetmax_home()

# Setting up the MQTT client and regstering callback
mqttc = mqtt.Client()
mqttc.connect("192.168.37.111", 1883, 60)
mqttc.on_message = on_message

# Listening to messages on this topic
device_id = socket.gethostname()
mqttc.subscribe(device_id+"/action")

# JetMax routines we can call remotely
def jetmax_left():
    print("Going left ..")
    mqttc.publish(device_id+"/running","left")
    jetmax.move_to(-20, 0, 0, 1, relative=True)
    mqttc.publish(device_id+"/running","stopped")

def jetmax_right():
    print("Going right ..")
    mqttc.publish(device_id+"/running","right")
    jetmax.move_to( 20, 0, 0, 1, relative=True)
    mqttc.publish(device_id+"/running","stopped")

def jetmax_home():
    print("Going home ..")
    mqttc.publish(device_id+"/running","homing")
    jetmax.go_home()
    mqttc.publish(device_id+"/running","stopped")

# Main loop
with myJetMax() as jetmax:
    try:
        print(f"{device_id} waiting for MQTT commands ..")
        while True:
            mqttc.loop()
    except KeyboardInterrupt:
        print("Aborting ..")

