
# set vars before importing modules
import os
import sys
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"

# retico
from retico.core.audio.io import MicrophoneModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.interop.zeromq.io import ReaderSingleton
from retico.interop.zeromq.io import WriterSingleton
from retico.interop.zeromq.io import ZeroMQReader
from retico.interop.zeromq.io import ZeroMQWriter
from retico.core.debug.console import DebugModule
from retico.modules.psi.common import PSItoSpeechRecognitionModule

# ReaderSingleton(ip='10.0.2.15', port='12345') # create the zeromq reader: client
WriterSingleton(ip='192.168.21.69', port='12346') # create the zeromq writer: host ip

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr nlu pipeline

# instantiate modules
mic = MicrophoneModule(5000)
asr = GoogleASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
# asr = ZeroMQReader('asr')
psi_asr = PSItoSpeechRecognitionModule()
wr1 = ZeroMQWriter('nlu')

debug = DebugModule()

# hook modules up to each other
mic.subscribe(asr)
asr.subscribe(nlu)
# asr.subscribe(psi_asr)
# psi_asr.subscribe(nlu)
nlu.subscribe(wr1)
asr.subscribe(debug)
# zmqr.subscribe(nlu)

# initialize modules
mic.run()
asr.run()
nlu.run() 
wr1.run()
psi_asr.run()
debug.run()

input() # keep things running

mic.stop()
nlu.stop()
asr.stop()
wr1.stop()
psi_asr.stop()
debug.stop()



