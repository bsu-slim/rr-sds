import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')

from retico.core.visual.io import WebcamModule
from retico.core.debug.console import DebugModule
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import WriterSingleton
from retico.interop.zeromq.io import ZeroMQReader
from retico.interop.zeromq.io import ReaderSingleton

WriterSingleton(ip='10.255.221.241', port='12347')
# ReaderSingleton(ip='10.29.2.115', port='12346')

# instantiate modules
webcam = WebcamModule()
writer = ZeroMQWriter(topic='object_boxes')
# reader = ZeroMQReader(topic='object_features')
debug = DebugModule()
# hook modules up to each other
webcam.subscribe(writer)
# reader.subscribe(debug)
# initialize modules
webcam.run()
writer.run()
# reader.run()
debug.run()

time.sleep(2)

webcam.stop()
writer.stop()
# reader.stop()
debug.stop()