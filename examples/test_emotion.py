import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')

from retico.core.debug.console import DebugModule
from retico.modules.azure.emotion_recognition import AzureEmotionDetectionModule
from retico.core.visual.io import WebcamModule


aed_endpoint = "https://westus2.api.cognitive.microsoft.com/"
aed_key = ""

# instantiate modules
emotion = AzureEmotionDetectionModule(aed_key, aed_endpoint)
webcam = WebcamModule()
debug = DebugModule()

# hook modules up to each other
webcam.subscribe(emotion)
emotion.subscribe(debug)

# initialize modules
webcam.run()
emotion.run()
debug.run()

input() # keep things running

webcam.stop()
emotion.stop()
debug.stop()
