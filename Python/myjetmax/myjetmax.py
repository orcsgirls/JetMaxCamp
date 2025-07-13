import time
import hiwonder
import rospy
import json
import threading
from std_msgs.msg import String

#---------------------------------------------------------------------------------
# Simple interface to color position AI
# Note that myjetmax/publisher/color_position_publisher.py
#        or myjetmax/publisher/apriltag_position_publisher.py needs to be running
#---------------------------------------------------------------------------------

class myAIBlocks():
    
    def __init__(self, what="apriltag"):
        if what != "color" and what != "apriltag":
            print('ERROR: Unknown block type')
            return

        print(f"AI {what} block detector initializing ..")
        rospy.init_node(what+'_subscriber', disable_signals=True)
        self.sub = rospy.Subscriber(what+"_location", String, self._callback)
        self.timer = threading.Timer(1, self._expireTimer)
        self.data = []
        print(f"AI {what} detector ready ..")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.sub.unregister()
        self.timer.cancel()

    def _callback(self, data):
        self.data = json.loads(data.data)

    def _expireTimer(self):
        self.timer.stop()
        self.data = []
        self.timer.start()

    @property
    def get_data(self):
        return self.data

    @property
    def get_data_wait(self):
        data = []
        while len(data) < 1:
            data = self.data
        return data

#---------------------------------------------------------------------------------
# Simplified class for JetMax Arm to be used with coordinate matte
#---------------------------------------------------------------------------------

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

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.go_home()

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
