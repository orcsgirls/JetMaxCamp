import paho.mqtt.client as mqtt
import time
from threading import Timer

#---------------------------------------------------------------------------------
# Simplified class for control of conveyor
#---------------------------------------------------------------------------------

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)
            
class Conveyor:
    
    def __init__(self, device_id="conveyor01", mqtt_server="192.168.37.111"):
        self.mqttc = mqtt.Client()
        self.mqttc.on_message = self.on_message
        self.mqttc.connect(mqtt_server, 1883, 60)
        self.mqtt_topic_action = device_id+"/action"
        self.mqtt_topic_running = device_id+"/running"
        self.mqttc.subscribe(self.mqtt_topic_running)
        self.timer = RepeatTimer(0.5, self.mqttc.loop)

        self.message="None"        
        self.timer.start()
    
    def shutdown(self):
        self.timer.cancel()
        self.mqttc.disconnect()

    def on_message(self, mqttc, userdata, message):
        if message.topic == self.mqtt_topic_running:
            self.message = message.payload.decode("utf-8")
        
    def forward(self):
        self.mqttc.publish(self.mqtt_topic_action,"forward")
    
    def backward(self):
        self.mqttc.publish(self.mqtt_topic_action,"backward")
        
    def stop(self):
        self.mqttc.publish(self.mqtt_topic_action,"stop")
    
    @property
    def status(self):
        return self.message


#---------------------------------------------------------------------------------

if __name__ == "__main__":
    conveyor = Conveyor()
    while True:
        command = input("Command (f,b,s,?,q):")
        if command == 'f':
            conveyor.forward()
        elif command == 'b':
            conveyor.backward()
        elif command == 's':
            conveyor.stop()
        elif command == '?':
            print(f"Status: {conveyor.status}")
        elif command == 'q':
            break

    conveyor.shutdown()  
    print('Done')