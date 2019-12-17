import os
import sys
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"

from retico.core.audio.io import MicrophoneModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.azure.asr import AzureASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule


# how to restart an utterance?
# had to change is_running to _is_running because opendial modules have an is_running() method

# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr pipeline
domain_dir = '/home/casey/git/PyOpenDial/domains/augi/augi.xml'

# instantiate modules
mic = MicrophoneModule(1000)
# mic = RespeakerMicrophoneModule('10.29.3.148:8000')
#asr = AzureASRModule("179eaa4b8fc54e0fa5115ba5d14883f2")
#asr = GoogleASRModule(rate=16000)
asr = GoogleASRModule()
# iasr = IncrementalizeASRModule()
# nlu = RasaNLUModule(model_dir=model_dir)
# dm = OpenDialModule(domain_dir=domain_dir)
debug = DebugModule()

# hook modules up to each other
mic.subscribe(asr)
asr.subscribe(debug)
# iasr.subscribe(nlu)
# nlu.subscribe(dm)
# dm.subscribe(debug)

# initialize modules
mic.run()
asr.run()
# iasr.run()
# nlu.run()
# dm.run()
debug.run()

input() # start streaming mic to ASR

mic.stop()
asr.stop()
iasr.stop()
nlu.stop()  
dm.stop()
debug.stop()