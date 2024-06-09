import time
import hiwonder

jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()

jetmax.go_home()
time.sleep(1.0)

# Getting the current position
cur_x, cur_y, cur_z = jetmax.position
print("Current position x,y,z: ", cur_x, cur_y, cur_z)
time.sleep(0.5)

# Moving to absolute values
jetmax.set_position((cur_x, cur_y, cur_z-100.), 1)
time.sleep(2.0)

# Moving to relative values (from the current position)
jetmax.set_position_relatively((0,0,100), 1)
time.sleep(2.0)

# Operate the sucker
sucker.suck()
time.sleep(2.0)

sucker.release(3.0)
time.sleep(2.0)

# Quick left and right for fun
jetmax.set_position_relatively((  80,0,0), 0.25)
time.sleep(1.0)
jetmax.set_position_relatively((-160,0,0), 0.5)
time.sleep(1.0)
jetmax.set_position_relatively((  80,0,0), 2)
