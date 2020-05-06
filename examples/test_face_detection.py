import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')

from retico.core.debug.console import DebugModule
from retico.modules.opencv.face_detect import FaceDetectionModule
from retico.core.visual.io import WebcamModule

faces = FaceDetectionModule()
webcam = WebcamModule()
debug = DebugModule()

webcam.subscribe(faces)
faces.subscribe(debug)

webcam.run()
faces.run()
debug.run()

input()

webcam.stop()
faces.stop()
debug.stop()