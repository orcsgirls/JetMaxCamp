import paho.mqtt.client as mqtt
import time

running = True

def on_connect(mqttc, obj, flags, reason_code):
    print("reason_code: " + str(reason_code))

def on_disconnect(mqttc, obj, reason_code):
    print("reason_code: " + str(reason_code))

def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid))

def on_unsubscribe(mqttc, obj, mid):
    print("Unsubscribed: " + str(mid))

def on_message(mqttc, userdata, message):
    global running

    if message.topic == mqtt_topic_running:
        running = message.payload

device_id = "conveyor01"

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.on_disconnect = on_disconnect

mqttc.connect("192.168.37.111", 1883, 60)
mqtt_topic_action = device_id+"/action"
mqtt_topic_config = device_id+"/config"
mqtt_topic_running = device_id+"/running"

mqttc.subscribe(mqtt_topic_running)

while True:
    mqttc.loop()
    command = input("Command (f,b,s,c,?,x,q):")
    if command == 'f':
        mqttc.publish(mqtt_topic_action,"forward")
    elif command == 'b':
        mqttc.publish(mqtt_topic_action,"backward")
    elif command == 's':
        mqttc.publish(mqtt_topic_action,"stop")
    elif command == '?':
        print (f"{device_id} running - {running}")
    elif command == 'x':
        mqttc.publish(mqtt_topic_config,"reset")
    elif command == 'c':
        payload = input("Payload: ")
        mqttc.publish(mqtt_topic_action, payload)
    elif command == 'q':
        break

mqttc.disconnect()
