
# set vars before importing modules
import os
import sys
os.environ['RASA'] = "/home/casey/git/rasa_nlu"

# retico
from retico.modules.rasa.nlu import RasaNLUModule
from retico.interop.zeromq.io import ReaderSingleton
from retico.interop.zeromq.io import WriterSingleton
from retico.interop.zeromq.io import ZeroMQReader
from retico.interop.zeromq.io import ZeroMQWriter
from retico.core.debug.console import DebugModule
from retico.modules.psi.common import PSItoSpeechRecognitionModule

ReaderSingleton(ip='192.168.0.31', port='12345') # create the zeromq reader
WriterSingleton(ip='192.168.0.16', port='12346') # create the zeromq reader

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr nlu pipeline

# instantiate modules
# mic = MicrophoneModule(1000)
nlu = RasaNLUModule(model_dir=model_dir)
zmqr = ZeroMQReader(b'asr')
psi_asr = PSItoSpeechRecognitionModule()
wr1 = ZeroMQWriter('nlu')

debug = DebugModule()

# hook modules up to each other
zmqr.subscribe(psi_asr)
psi_asr.subscribe(nlu)
nlu.subscribe(wr1)
nlu.subscribe(debug)
# zmqr.subscribe(nlu)

# initialize modules
zmqr.run()
nlu.run() 
wr1.run()
psi_asr.run()
debug.run()

input() # keep things running

nlu.stop()
zmqr.stop()
wr1.stop()
psi_asr.stop()
debug.stop()



