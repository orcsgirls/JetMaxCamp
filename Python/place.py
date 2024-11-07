# This asks for two locations in x,y and modes a block between them

from myjetmax import *

jetmax = myJetMax()

# Setting heights for moving, picking and dropping a block

pick_z = 30.
move_z = 80.
drop_z = 35.

# Ask for location

pick_x = float(input("Pickup location x:"))
pick_y = float(input("Pickup location y:"))
drop_x = float(input("Drop location x:"))
drop_y = float(input("Drop location y:"))

print("Going to pickup location")
jetmax.move_to(pick_x, pick_y, move_z, 1)

print("Picking up")
jetmax.move_to(pick_x, pick_y, pick_z, 1)
jetmax.suck()
jetmax.move_to(pick_x, pick_y, move_z, 1)

print("Dropping at destination")
jetmax.move_to(drop_x, drop_y, move_z, 1)
jetmax.move_to(drop_x, drop_y, drop_z, 1)
jetmax.release()

print("Returning home")
jetmax.go_home()

