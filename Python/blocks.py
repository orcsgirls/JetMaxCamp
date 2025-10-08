import sys
import time
from myjetmax import *

with myAIBlocks(what="color") as blocks, myJetMax() as jetmax:
    try:
        while True:
            found=blocks.get_data_wait(what='red')
            for f in found:
                print(f"Block {f.id:<5} at position {f.x:7.2f}, {f.y:7.2f}")
            time.sleep(1)
    except KeyboardInterrupt:
        pass

print('Done')
