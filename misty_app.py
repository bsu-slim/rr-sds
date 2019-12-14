import threading
import time

# set vars before importing modules
import os
import sys
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"

# retico
from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.modules.misty.misty_action import MistyAction

from retico.core.text.common import SpeechRecognitionIU


# had to change is_running to _is_running because opendial modules have an is_running() method
# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/retico/data/cozmo/nlu/models/nlu_20191107-110224' # incr nlu pipeline
domain_dir = '/home/casey/git/retico/data/cozmo/dm/dialogue.xml'

# instantiate modules
mic = MicrophoneModule(5000)
gasr = GoogleASRModule()
iasr = IncrementalizeASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
dm = OpenDialModule(domain_dir=domain_dir)
misty = MistyAction("192.168.0.21")

# hook modules up to each other
mic.subscribe(gasr)
gasr.subscribe(iasr)
iasr.subscribe(nlu)
nlu.subscribe(dm)
dm.subscribe(misty)

# initialize modules
mic.run()
gasr.run()
iasr.run()
nlu.run()
dm.run()
misty.run()
    
input() # start streaming mic to ASR

mic.stop()
gasr.stop()
iasr.stop()
nlu.stop()  
dm.stop()
misty.stop()
