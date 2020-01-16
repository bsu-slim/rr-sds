import threading
import time

# set vars before importing modules
import os
import sys
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"
os.environ['COZMO'] = "/home/casey/git/cozmo-python-sdk/src"
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # error level
import warnings
warnings.filterwarnings('ignore')
import logging
logging.getLogger('tensorflow').setLevel(logging.ERROR)
#cozmo
sys.path.append(os.environ['COZMO'])
import cozmo

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
from retico.modules.cozmo.cozmo_camera import CozmoCameraModule
from retico.modules.keras.object_features import KerasObjectFeatureExtractorModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.modules.azure.object_detection import AzureObjectDetectionModule
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import WriterSingleton
from retico.modules.slim.words_as_classifiers import WordsAsClassifiersModule

# had to change is_running to _is_running because opendial modules have an is_running() method
# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

def init_all(robot : cozmo.robot.Robot):

    model_dir = '/home/casey/git/retico/data/cozmo/nlu/models/nlu_20191231-234236' # incr nlu pipeline
    domain_dir = '/home/casey/git/retico/data/cozmo/dm/dialogue.xml'
    aod_endpoint = "https://slimcomputervision.cognitiveservices.azure.com/"
    aod_key = "59bfd2dc248a4d08957edf7a6eb6331f"
    wac_dir = '/home/casey/git/retico/data/wac/subset'

    #
    # INSTANTIATE MODULES
    #
    # mic = RespeakerMicrophoneModule('192.168.20.49:8000')
    # asr = GoogleASRModule(rate=16000)
    mic = MicrophoneModule(1000)
    asr = GoogleASRModule()
    iasr = IncrementalizeASRModule()
    nlu = RasaNLUModule(model_dir=model_dir)
    dm = OpenDialModule(domain_dir=domain_dir)
    cozmo_action = CozmoActionModule(robot)
    cozmo_camera = CozmoCameraModule(robot)
    cozmo_state = CozmoStateModule(robot)
    object_detector = AzureObjectDetectionModule(aod_key, aod_endpoint)
    feature_extractor = KerasObjectFeatureExtractorModule()
    wac = WordsAsClassifiersModule(wac_dir=wac_dir)
    debug = DebugModule()

    # psi related modules
    # WriterSingleton(ip='192.168.20.143', port='12346') # create the zeromq writer
    # iasr_psi = ZeroMQWriter(topic='asr')
    # nlu_psi = ZeroMQWriter(topic='nlu')
    # dm_psi = ZeroMQWriter(topic='dm')
   
    # mic as input
    mic.subscribe(asr)
    asr.subscribe(iasr)
    iasr.subscribe(wac)
    wac.subscribe(dm)
    dm.subscribe(debug)

    # robot state as input
    cozmo_state.subscribe(dm)

    # robot camera as input
    cozmo_camera.subscribe(object_detector)
    object_detector.subscribe(feature_extractor)
    feature_extractor.subscribe(wac)

    # iasr.subscribe(iasr_psi)
    # nlu.subscribe(nlu_psi)
    # dm.subscribe(dm_psi)
    # feat_ext.subscribe(feat_ext_psi)
    # obj_det.subscribe(obj_det_psi)

    #
    # INITIALIZE MODULES
    # 
    mic.run()
    asr.run()
    iasr.run()
    nlu.run()
    dm.run()
    cozmo_action.run()
    cozmo_state.run()
    # cozmo_camera.run()
    # object_detector.run()
    # feature_extractor.run()
    wac.run()
    debug.run()

    # dm_psi.run()
    # nlu_psi.run()
    # iasr_psi.run()

    input() # keep everything running

    mic.stop()
    asr.stop()
    iasr.stop()
    cozmo_action.stop()
    cozmo_state.stop()
    nlu.stop()  
    dm.stop()
    cozmo_camera.stop()
    object_detector.stop()
    feature_extractor.stop()
    wac.stop()

    # dm_psi.stop()
    # iasr_psi.stop()
    # nlu_psi.stop()

cozmo.run_program(init_all, use_viewer=True, force_viewer_on_top=False)