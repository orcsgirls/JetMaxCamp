import time
import hiwonder
import rospy
import json
import threading
from std_msgs.msg import String

# Simple interface to color position AI
# Note that myjetmax/publisher/color_position_publisher.py needs to be running

class myColorBlocks():
    
    def __init__(self):
        rospy.init_node('color_subscriber')
        self.sub = rospy.Subscriber("color_location", String, self._callback)
        self.data = []
        self._expireTimer()

    def _callback(self, data):
        self.data = json.loads(data.data)

    def _expireTimer(self):
        self.data = []
        self.timer = threading.Timer(1, self._expireTimer)
        self.timer.start()

    def exit(self):
        self.sub.unregister()
        self.timer.cancel()

    @property
    def blocks_detected(self):
        return len(self.data)

    @property
    def angle(self, id=0):
        time.sleep(0.1)
        return self.data[id]['angle']

    @property
    def color(self, id=0):
        time.sleep(0.1)
        return self.data[id]['color']

    @property
    def position(self, id=0):
        time.sleep(0.1)
        return (self.data[id]['x'], self.data[id]['y'])


# Simplified class to be used with coordinate matte

class myJetMax(hiwonder.JetMax):

    def __init__(self):
        print("JetMax initializing ..")
        super().__init__()
        self.sucker=hiwonder.Sucker()
        self.origin_x = 0.0
        self.origin_y = -178.94
        self.origin_z = 60.80
        self.go_home()
        print("JetMax ready ..")

    def move_to(self,x,y,z,duration,relative=False):
        if relative:
            (cx,cy,cz) = self.location(log=False)
            x=x+cx
            y=y+cy
            z=z+cz

        print(f"Moving to {x:6.1f} {y:6.1f} {z:6.1f}")
        self.set_position((x+self.origin_x,y+self.origin_y,z+self.origin_z),duration)
        time.sleep(duration)

    def go_home(self):
        super().go_home()
        hiwonder.pwm_servo1.set_position(90, 0.1)
        time.sleep(3.0)

    def location(self, log=True):
        (x,y,z) = self.position
        if log:
            print(f"Position  {x-self.origin_x:6.1f} {y-self.origin_y:6.1f} {z-self.origin_z:6.1f}")
        return (x-self.origin_x,y-self.origin_y,z-self.origin_z)

    def suck(self):
        print("Sucker active ..")
        self.sucker.suck()
        time.sleep(1.0)

    def release(self):
        print("Sucker released ..")
        self.sucker.release(3.0)
        time.sleep(3.0)

    def set_angle(self, angle):
        if (angle>=0 and angle <=180):
            hiwonder.pwm_servo1.set_position(angle, 0.1)
        else:
            print(f"ERROR: Invalid angle {angle:6.1f}")

    def get_angle(self):
        angle=hiwonder.pwm_servo1.get_position()
        return angle
