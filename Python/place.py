# This asks for two locations in x,y and modes a block between them

from myjetmax import *

with myJetMax() as jetmax:

  # Ask for location

  pick_x = float(input("Pickup location x:"))
  pick_y = float(input("Pickup location y:"))
  pick_z = float(input("Pickup location z:"))
  drop_x = float(input("Drop location x:"))
  drop_y = float(input("Drop location y:"))
  drop_z = float(input("Drop location z:"))
  move_z = max(drop_z, pick_z)
  
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

