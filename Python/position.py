from myjetmax import *

with myJetMax() as jetmax:

  # Ask for x,y,z and move to that position
  while True:
      try:
          x,y,z = input("Enter location x y z (separated by space): ").split()
          jetmax.move_to(float(x), float(y), float(z), 1)
      except KeyboardInterrupt:
          print('Bye ..')

