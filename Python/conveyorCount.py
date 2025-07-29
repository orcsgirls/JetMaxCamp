
import paho.mqtt.client as mqtt
import time
import sys
from myjetmax import *

def on_connect(mqttc, obj, flags, reason_code):
    print("reason_code: " + str(reason_code))

def on_disconnect(mqttc, obj, reason_code):
    print("reason_code: " + str(reason_code))

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: " + str(mid))

def on_unsubscribe(mqttc, obj, mid):
    print("Unsubscribed: " + str(mid))


mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe
mqttc.on_disconnect = on_disconnect

mqttc.connect("192.168.37.111", 1883, 60)
mqtt_topic = "control/conveyor"
mqttc.subscribe(mqtt_topic)

#------------------------------------------------------------------------
# Start the conveyor and wait for a block
#------------------------------------------------------------------------


nblocks = 0

with myAIBlocks(what="color") as blocks, myJetMax() as jetmax:

  print('Sending start ..')
  mqttc.publish(mqtt_topic,"start")

  jetmax.go_home()
  time.sleep(1.0)

  try:
      while True:
        found = blocks.get_data_wait
        mqttc.publish(mqtt_topic,"stop")
        time.sleep(1.0)
        nblocks+=1
        print(f"Block count {nblocks} ..")
        mqttc.publish(mqtt_topic,"start")
        time.sleep(2.0)
  except KeyboardInterrupt:
    print(f"Keyboard interrupt received")

mqttc.publish(mqtt_topic,"stop")
mqttc.unsubscribe(mqtt_topic)
mqttc.disconnect()
jetmax.release()
