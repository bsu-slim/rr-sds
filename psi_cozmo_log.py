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
from retico.modules.cozmo.cozmo_action import CozmoActionModule
from retico.modules.cozmo.cozmo_state import CozmoStateModule
from retico.modules.keras.object_features import KerasObjectFeatureExtractorModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.modules.azure.object_detection import AzureObjectDetectionModule
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import WriterSingleton



#cozmo
sys.path.append(os.environ['COZMO'])
import cozmo

# had to change is_running to _is_running because opendial modules have an is_running() method
# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

def init_all(robot : cozmo.robot.Robot):

     

    model_dir = '/home/casey/git/retico/data/cozmo/nlu/models/nlu_20191231-234236' # incr nlu pipeline
    domain_dir = '/home/casey/git/retico/data/cozmo/dm/dialogue.xml'
    aod_endpoint = "https://slimcomputervision.cognitiveservices.azure.com/"
    aod_key = "59bfd2dc248a4d08957edf7a6eb6331f"


    # instantiate modules
    mic = MicrophoneModule(5000)
    asr = GoogleASRModule()
    # use mic and asr below for respeaker
    # mic = RespeakerMicrophoneModule('192.168.0.36:8000')
    # asr = GoogleASRModule(rate=16000)

    iasr = IncrementalizeASRModule()
    nlu = RasaNLUModule(model_dir=model_dir)
    dm = OpenDialModule(domain_dir=domain_dir)
    cozmo_action = CozmoActionModule(robot)
    cozmo_state = CozmoStateModule(robot)
    feat_ext = KerasObjectFeatureExtractorModule()
    obj_det = AzureObjectDetectionModule(key=aod_key,endpoint=aod_endpoint)

    debug = DebugModule()

    # psi related modules
    WriterSingleton(ip='192.168.0.14', port='12346') # create the zeromq writer
    iasr_psi = ZeroMQWriter(topic='asr')
    nlu_psi = ZeroMQWriter(topic='nlu')
    dm_psi = ZeroMQWriter(topic='dm')
   
    # hook modules up to each other
    mic.subscribe(asr)
    asr.subscribe(iasr)
    iasr.subscribe(nlu)
    nlu.subscribe(dm)

    cozmo_state.subscribe(obj_det)
    obj_det.subscribe(feat_ext)
    # feat_ext.subscribe(debug)

    # dm.subscribe(debug)
    # nlu.subscribe(debug)

    iasr.subscribe(iasr_psi)
    nlu.subscribe(nlu_psi)
    dm.subscribe(dm_psi)

    dm.subscribe(cozmo_action)

    # initialize modules
    mic.run()
    asr.run()
    iasr.run()
    nlu.run()
    dm.run()
    cozmo_state.run()
    cozmo_action.run()
    obj_det.run()
    feat_ext.run()
    debug.run()

    dm_psi.run()
    nlu_psi.run()
    iasr_psi.run()

    input() # keep everything running

    mic.stop()
    asr.stop()
    iasr.stop()
    cozmo_state.stop()
    cozmo_action.stop()
    feat_ext.stop()
    obj_det.stop()
    nlu.stop()  
    dm.stop()

    dm_psi.stop()
    iasr_psi.stop()
    nlu_psi.stop()

cozmo.run_program(init_all, use_viewer=True, force_viewer_on_top=False)