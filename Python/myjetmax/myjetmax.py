import time
import hiwonder

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
        print("JetMax ready ..\n")

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
        time.sleep(3.0)

    def location(self, log=True):
        (x,y,z) = self.position
        if log:
            print(f"Position  {x-self.origin_x:6.1f} {y-self.origin_y:6.1f} {z-self.origin_z:6.1f}")
        return (x-self.origin_x,y-self.origin_y,z-self.origin_z)

    def suck(self):
        print("Sucker active ..\n")
        self.sucker.suck()

    def release(self):
        print("Sucker released ..\n")
        self.sucker.release(3.0)
        time.sleep(3.0)
