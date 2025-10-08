import paho.mqtt.client as mqtt
import time

# Test program to control the conveyor. Make changes in the next
# lines to match the ID of your conveyor

device_id = 'conveyor01'  # name of the conveyor
distance = 25.0           # Distance in cm to drive
conversion = 1.0 / 10.5   # How any cm dies it drive in one second - CALIBRATE this


# This sets up the MQTT connection

mqttc = mqtt.Client()
mqttc.connect("192.168.37.111", 1883, 60)
mqtt_topic_action = device_id+"/action"

# To drive we publish 'forward' to the conveyor, wait and then publish 'stop'

print('Starting conveyor')
mqttc.publish(mqtt_topic_action,"forward")
time.sleep(conversion*distance)
print('Stopping conveyor')
mqttc.publish(mqtt_topic_action,"stop")

# All done and we disconnect from the MQTT broker

mqttc.disconnect()
print('Done')
