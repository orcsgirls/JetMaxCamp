import sys
from myjetmax import *

offset_y = 7.5

with myAIBlocks(what="apriltag") as blocks, myJetMax() as jetmax:

  jetmax.go_home()
  time.sleep(1.0)

  found = blocks.get_data_wait
  (block_x, block_y) = (found[0].x, found[0].y)
  print ("Found block at: ", block_x, block_y)

  # Correcting
  block_y = block_y + offset_y

  # Moving there
  jetmax.move_to(block_x, block_y, 150, 1.0)
  jetmax.move_to(block_x, block_y,  30, 1.0)

  # Sucker on and move up and drop
  jetmax.suck()
  jetmax.move_to(block_x, block_y, 50, 2.0)
  jetmax.release()
