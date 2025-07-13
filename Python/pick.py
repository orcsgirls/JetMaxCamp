import sys
from myjetmax import *

with myAIBlocks(what="apriltag") as blocks, myJetMax() as jetmax:

  found=[]
  while len(found)<1:
      found = blocks.get_data

  (block_x, block_y) = (found[0]['x'], found[0]['y'])
  print ("Found block at: ", block_x, block_y)

  # Moving there  nd go down
  jetmax.move_to(block_x, block_y, 150, 1.0)
  jetmax.move_to(block_x, block_y, 40, 1.0)

  # Sucker on and move up and drop
  #jetmax.suck()
  #jetmax.move_to(block_x, block_y, 40, 2.0)
  #jetmax.release()

  while True:
      pass
