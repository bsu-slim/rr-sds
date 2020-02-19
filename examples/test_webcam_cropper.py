import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')

from retico.core.visual.io import WebcamModule
from retico.core.visual.common import ImageCropperModule


# instantiate modules
webcam = WebcamModule()
# crop the top off by a little bit
cropper = ImageCropperModule(top=200)

# hook modules up to each other
webcam.subscribe(cropper)

# initialize modules
webcam.run()
cropper.run()

time.sleep(1)
# input() # keep things running

webcam.stop()
cropper.stop()