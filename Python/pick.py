import sys
from myjetmax import *

with myAIBlocks(what="apriltag") as blocks, myJetMax() as jetmax:

  found = blocks.get_data_wait

  (block_x, block_y) = (found[0]['x'], found[0]['y'])
  print ("Found block at: ", block_x, block_y)

  # Applying offset
  block_y = block_y+7.5

  # Moving there  nd go down
  jetmax.move_to(block_x, block_y, 150, 1.0)
  jetmax.move_to(block_x, block_y,  30, 1.0)

  # Sucker on and move up and drop
  jetmax.suck()
  jetmax.move_to(block_x, block_y, 50, 2.0)
  jetmax.release()
