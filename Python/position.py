from myjetmax import *

with myJetMax() as jetmax:
    while True:
        jetmax.location(log=True)
        x,y,z = input("Enter new x y z (separated by space): ").split()
        jetmax.move_to(float(x), float(y), float(z), 1)

