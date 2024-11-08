import sys
from myjetmax import *

jetmax = myJetMax()
blocks = myColorBlocks()

# Pick up the block and lift and drop :)

(block_x, block_y) = blocks.position
print ("Found block at: ", block_x, block_y)

# Moving there  nd go down
jetmax.move_to(block_x, block_y, 150, 2.0)
jetmax.move_to(block_x, block_y, 20, 2.0)

# Sucker on and move up and drop
jetmax.suck()
jetmax.move_to(block_x, block_y, 40, 2.0)
jetmax.release()

