# This asks for two locations in x,y and modes a block between them

import time
import hiwonder

jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()

# Go home to start

print('Going home')
jetmax.go_home()
sucker.release(3.0)
time.sleep(1)

# We got the origin of the grid by mofind the nozzle to the center 
# and touching the grid. The x,y,z coodinates came from the web control

origin_x = 0.0
origin_y = -178.94
origin_z = 60.80

pick_z = 30.
move_z = 80.
drop_z = 35.

# Ask for location

pick_x = float(input("Pickup location x:"))
pick_y = float(input("Pickup location y:"))
drop_x = float(input("Drop location x:"))
drop_y = float(input("Drop location y:"))

print("Going to pickup location")
jetmax.set_position((pick_x+origin_x, pick_y+origin_y, move_z+origin_z), 1)
time.sleep(2.0)

print("Picking up")
jetmax.set_position((pick_x+origin_x, pick_y+origin_y, pick_z+origin_z), 1)
time.sleep(2.0)
sucker.suck()
time.sleep(2.0)
jetmax.set_position((pick_x+origin_x, pick_y+origin_y, move_z+origin_z), 1)
time.sleep(2.0)

print("Dropping at destination")
jetmax.set_position((drop_x+origin_x, drop_y+origin_y, move_z+origin_z), 1)
time.sleep(2.0)
jetmax.set_position((drop_x+origin_x, drop_y+origin_y, drop_z+origin_z), 1)
time.sleep(2.0)
sucker.release(3.0)

print("Returning home")
jetmax.go_home()

