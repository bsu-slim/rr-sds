import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')
# export LD_LIBRARY_PATH=/home/casey/anaconda3/envs/py-opendial/lib/python3.6/site-packages/torch/lib/:$LD_LIBRARY_PATH
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'

from retico.core.audio.io import MicrophoneModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.core.debug.console import DebugModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.azure.asr import AzureASRModule
from retico.core.text.asr import IncrementalizeASRModule
from retico.modules.azure.emotion_recognition import AzureEmotionDetectionModule

# how to restart an utterance?
# had to change is_running to _is_running because opendial modules have an is_running() method

# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

aod_endpoint = "https://slimcomputervision.cognitiveservices.azure.com/"
aod_key = "59bfd2dc248a4d08957edf7a6eb6331f"
aed_endpoint = "https://westus2.api.cognitive.microsoft.com/"
aed_key = "1837b9d29e0b4a22843d103a7ca8b3c9"

# instantiate modules
mic = MicrophoneModule(1000, rate=16000)
# mic = RespeakerMicrophoneModule('192.168.20.49:8000')
asr = AzureASRModule("179eaa4b8fc54e0fa5115ba5d14883f2")
emotion = AzureEmotionDetectionModule(aed_key, aed_endpoint)
# asr = GoogleASRModule(rate=16000)
# asr = GoogleASRModule()
iasr = IncrementalizeASRModule()

debug = DebugModule()



# hook modules up to each other
mic.subscribe(asr)
asr.subscribe(iasr)
iasr.subscribe(debug)

# initialize modules
mic.run()
asr.run()
iasr.run()
debug.run()

input() # keep things running

mic.stop()
asr.stop()
iasr.stop()
debug.stop()