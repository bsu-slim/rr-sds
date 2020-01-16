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
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.core.text.common import SpeechRecognitionIU

# Make sure you're on the same network as Misty!

# had to change is_running to _is_running because opendial modules have an is_running() method
# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/retico/data/misty/nlu/models/nlu_20191217-145005' # incr nlu pipeline
domain_dir = '/home/casey/git/retico/data/misty/dm/dialogue.xml'
misty_ip = "10.10.0.7"
respeaker_ip_port = '10.10.7.45:8000'

# instantiate modules
mic = MicrophoneModule(1000)
gasr = GoogleASRModule()
# mic = RespeakerMicrophoneModule(respeaker_ip_port)
# gasr = GoogleASRModule(rate=16000)
iasr = IncrementalizeASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
dm = OpenDialModule(domain_dir=domain_dir)
misty = MistyAction(misty_ip)
debug = DebugModule()


# hook modules up to each other
mic.subscribe(gasr)
gasr.subscribe(iasr)
iasr.subscribe(nlu)
nlu.subscribe(dm)
dm.subscribe(misty)
dm.subscribe(debug)

# initialize modules
mic.run()
gasr.run()
iasr.run()
nlu.run()
dm.run()
misty.run()
debug.run()
    
input() # start streaming mic to ASR

mic.stop()
gasr.stop()
iasr.stop()
nlu.stop()  
dm.stop()
misty.stop()
debug.stop()
