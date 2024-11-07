import hiwonder
import time

mtx = hiwonder.TM1640(4, 9)
mtx.brightness(4)

for i in range(21):
    mtx.tube_display_int(i)
    mtx.refresh()
    time.sleep(0.2)
