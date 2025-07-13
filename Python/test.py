from myjetmax import *

with myJetMax() as jetmax:

  # This moves to the given coordinates x,y and height z as on the grid.
  # Units are mm - here we make a square
  # The last number is the duration for the move - here 1 second

  jetmax.move_to( 0, 0,30, 1)
  jetmax.move_to(80, 0,30, 1)
  jetmax.move_to(80,80,30, 1)
  jetmax.move_to( 0,80,30, 1)
  jetmax.move_to( 0, 0,30, 1)

  # This moves relative to the last position 
  # Here 40 mm up.

  jetmax.move_to( 0, 0,40, 1, relative=True)

  # This returns the current location of the sucker
  (cx,cy,cz) = jetmax.location()

  # Set angle of the sucker
  jetmax.set_angle(10)

  # Turn sucker on and off
  jetmax.suck()
  time.sleep(1.0)
  jetmax.release()
