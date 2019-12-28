import threading
import time

# set vars before importing modules
import os
import sys
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"
os.environ['COZMO'] = "/home/casey/git/cozmo-python-sdk/src"

# retico
from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule
# from retico.modules.cozmo.cozmo_action import CozmoAction
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import WriterSingleton


#cozmo
# sys.path.append(os.environ['COZMO'])
# import cozmo

# had to change is_running to _is_running because opendial modules have an is_running() method
# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

# def init_all(robot : cozmo.robot.Robot):

model_dir = '/home/casey/git/retico/data/cozmo/nlu/models/nlu_20191107-110224' # incr nlu pipeline
domain_dir = '/home/casey/git/retico/data/cozmo/dm/dialogue.xml'


# instantiate modules
mic = MicrophoneModule(5000)
asr = GoogleASRModule()
# use mic and asr below for respeaker
#mic = RespeakerMicrophoneModule('10.29.3.148:8000')
#asr = GoogleASRModule(rate=16000)

# carry on from here
iasr = IncrementalizeASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
dm = OpenDialModule(domain_dir=domain_dir)
debug = DebugModule()

WriterSingleton(ip='192.168.0.16', port='12346') # create the zeromq writer
iasr_psi = ZeroMQWriter('asr')
nlu_psi = ZeroMQWriter('nlu')
dm_psi = ZeroMQWriter('dm')
# cozmo = CozmoAction(debug)

# hook modules up to each other
mic.subscribe(asr)
asr.subscribe(iasr)
iasr.subscribe(nlu)
nlu.subscribe(dm)

dm.subscribe(debug)
nlu.subscribe(debug)

iasr.subscribe(iasr_psi)
nlu.subscribe(nlu_psi)
dm.subscribe(dm_psi)

# dm.subscribe(cozmo)

# initialize modules
mic.run()
asr.run()
iasr.run()
nlu.run()
dm.run()
debug.run()

dm_psi.run()
nlu_psi.run()
iasr_psi.run()
# cozmo.run()
    
input() # start streaming mic to ASR

mic.stop()
asr.stop()
iasr.stop()
iasr_psi.stop()
nlu.stop()  
nlu_psi.stop()
dm.stop()
dm_psi.stop()
# cozmo.stop()

#cozmo.run_program(init_all, use_viewer=False, force_viewer_on_top=False)
# init_all(None)