import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')

import cv2

from retico.core.debug.console import DebugModule
from retico.modules.psi.psiAsrReceiver import psiAsrReceiverModule
from retico.interop.zeromq.io import ZeroMQReader

zmPsi = ZeroMQReader(topic='some_topic', ip='localhost', port=12345)
psiAsr = psiAsrReceiverModule()
debug = DebugModule()

zmPsi.subscribe(psiAsr)
psiAsr.subscribe(debug)

zmPsi.run()
psiAsr.run()
debug.run()

print("reading: ")

input()

zmPsi.stop()
psiAsr.stop()
debug.stop()