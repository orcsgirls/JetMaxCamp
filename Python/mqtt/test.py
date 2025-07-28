import paho.mqtt.client as mqtt
import time

def on_connect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: " + str(reason_code))

def on_disconnect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: " + str(reason_code))

def on_message(mqttc, obj, msg):
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: " + str(mid) + " " + str(reason_code_list))

def on_unsubscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Unsubscribed: " + str(mid) + " " + str(reason_code_list))


def on_log(mqttc, obj, level, string):
    print(string)


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe
mqttc.on_disconnect = on_disconnect

mqttc.connect("192.168.37.173", 1883, 60)
mqtt_topic = "control/conveyor"
mqttc.subscribe(mqtt_topic)

mqttc.publish(mqtt_topic,"start")
time.sleep(1.0)
mqttc.publish(mqtt_topic,"stop")

try:
    mqttc.loop_forever()
except KeyboardInterrupt:
    print(f"Keyboard interrupt received")

mqttc.unsubscribe(mqtt_topic)
mqttc.disconnect()
