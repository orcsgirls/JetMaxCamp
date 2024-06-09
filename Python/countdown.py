import hiwonder
import time

mtx = hiwonder.TM1640(4, 9)
mtx.brightness(4)

count = 10.0   # Countdown 10 seconds
wait  = 0.1    # Wait between number updates
delay = 0.05   # Time the display takes to update (tweak this to bet the timing right)

while count > 0.0:
    mtx.tube_display_float(count)
    mtx.refresh()
    count = count-wait
    time.sleep(wait - delay)

print("Done")
