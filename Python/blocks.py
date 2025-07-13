import sys
import time
from myjetmax import *

with myAIBlocks(what="apriltag") as blocks, myJetMax() as jetmax:
  try:
    while True:
      print(blocks.get_data)
      time.sleep(1)
  except KeyboardInterrupt:
    pass

print('Done')
