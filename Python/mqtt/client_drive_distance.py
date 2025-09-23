import paho.mqtt.client as mqtt
import time
import argparse

def on_connect(mqttc, obj, flags, reason_code):
    print("reason_code: " + str(reason_code))

def on_disconnect(mqttc, obj, reason_code):
    print("reason_code: " + str(reason_code))

parser = argparse.ArgumentParser(description="Conveyor driver")
parser.add_argument("--device", nargs="?", type=str, default="conveyor02", help="Device ID")
parser.add_argument("--distance", nargs="?", type=float, default=5.0, help="Distance in cm")
args = parser.parse_args()

device_id = args.device
print(f"Controlling device {device_id}")
distance = args.distance

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_disconnect = on_disconnect

mqttc.connect("192.168.37.111", 1883, 60)
mqtt_topic_action = device_id+"/action"

convert = 2.0 / 21.0

# Drive 10cm forward
mqttc.publish(mqtt_topic_action,"forward")
print(convert*distance)
time.sleep(convert*distance)
mqttc.publish(mqtt_topic_action,"stop")

mqttc.disconnect()
print('Done')
