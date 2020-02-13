import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')

from retico.core.debug.console import DebugModule
from retico.modules.misty.misty_camera import MistyCameraModule


# instantiate modules
camera = MistyCameraModule("10.10.0.7")
debug = DebugModule()

# hook modules up to each other
camera.subscribe(debug)

# initialize modules
camera.run()
debug.run()

input() # keep things running

camera.stop()
debug.stop()