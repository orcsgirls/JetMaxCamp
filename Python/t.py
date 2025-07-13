import sys
import time
from myjetmax import *

blocks = myColorBlocks()

for i in range(10):
    if(blocks.blocks_detected>0):
        print(blocks.color)
    time.sleep(1)

blocks.exit()
print('Done')
