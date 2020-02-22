import threading
import time

# set vars before importing modules
import os
import sys


os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"
os.environ['COZMO'] = "/home/casey/git/cozmo-python-sdk/src"
os.environ['TF_RESEARCH'] = '/home/casey/git/tfmodels/research'
os.environ['TF_SLIM'] = '/home/casey/git/tfmodels/research/slim'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # error level
import warnings
warnings.filterwarnings('ignore')
import logging
logging.getLogger('tensorflow').setLevel(logging.ERROR)
#cozmo
sys.path.append(os.environ['COZMO'])
import cozmo

# retico
import retico
from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule
from retico.modules.cozmo.cozmo_refer import CozmoReferModule
from retico.modules.cozmo.cozmo_state import CozmoStateModule
from retico.modules.cozmo.cozmo_camera import CozmoCameraModule
from retico.modules.keras.object_features import KerasObjectFeatureExtractorModule
from retico.core.audio.io import RespeakerMicrophoneModule
# from retico.modules.azure.object_detection import AzureObjectDetectionModule
from retico.modules.google.od import MaskrRCNNObjectDetection
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import WriterSingleton
from retico.modules.slim.words_as_classifiers import WordsAsClassifiersModule
from retico.core.visual.io import WebcamModule

# had to change is_running to _is_running because opendial modules have an is_running() method

def init_all(robot : cozmo.robot.Robot):
    
    domain_dir = '/home/casey/git/retico/data/cozmo/dm/dialogue.xml'
    aod_endpoint = "https://slimcomputervision.cognitiveservices.azure.com/"
    aod_key = "59bfd2dc248a4d08957edf7a6eb6331f"
    wac_dir = '/home/casey/git/retico/data/wac/subset'
    mask_rcnn_labels = '/home/casey/git/retico/data/maskrcnn/label_map.pbtxt'
    mask_rcnn_model = '/home/casey/git/retico/data/maskrcnn/frozen_inference_graph.pb'

    opendial_variables = ['face_count', # CozmoStateModule
                           'num_objs', # ObjectDetector
                           'near_object', # CozmoRefer
                           'exploring', # CozmoRefer
                           'aligned', # CozmoRefer
                           'word_to_find', # WordsAsClassifiersModule
                           'best_object', # WordsAsClassifiersModule
                           'obj_confidence'] # WordsAsClassifiersModule

    #
    # INSTANTIATE MODULES
    #
    # mic = RespeakerMicrophoneModule('192.168.0.102:8000')
    # asr = GoogleASRModule(rate=16000)
    
    mic = MicrophoneModule(1000)
    asr = GoogleASRModule()
    iasr = IncrementalizeASRModule()
    dm = OpenDialModule(domain_dir=domain_dir, variables=opendial_variables)
    cozmo_refer = CozmoReferModule(robot)
    cozmo_camera = CozmoCameraModule(robot)
    # cozmo_camera = WebcamModule()
    # cozmo_state = CozmoStateModule(robot)
    # object_detector = AzureObjectDetectionModule(aod_key, aod_endpoint)
    object_detector = MaskrRCNNObjectDetection(mask_rcnn_labels, mask_rcnn_model)
    feature_extractor = KerasObjectFeatureExtractorModule()
    wac = WordsAsClassifiersModule(wac_dir=wac_dir)
    debug = DebugModule()

    # psi related modules
    # WriterSingleton should use the *source* ip address (i.e., this machine's)
    WriterSingleton(ip='192.168.0.101', port='12346') # create the zeromq writer
    psi = ZeroMQWriter(topic='retico')
   
    # mic as input
    mic.subscribe(asr)
    asr.subscribe(iasr)
    iasr.subscribe(wac)
    wac.subscribe(dm)
    dm.subscribe(cozmo_refer)

    # robot state as input
    # cozmo_state.subscribe(dm)
    object_detector.subscribe(dm)
    object_detector.subscribe(cozmo_refer)
    cozmo_refer.subscribe(dm)

    # robot camera as input
    cozmo_camera.subscribe(object_detector)
    object_detector.subscribe(feature_extractor)
    feature_extractor.subscribe(wac)
    # feature_extractor.subscribe(debug)

    iasr.subscribe(psi)
    wac.subscribe(psi)
    dm.subscribe(psi)
    feature_extractor.subscribe(psi)
    object_detector.subscribe(psi)

    #
    # INITIALIZE MODULES
    # 
    mic.run()
    asr.run()
    iasr.run()
    dm.run()
    cozmo_refer.run() # IF I MAKE THIS RUN EVERYTHING SLOWS DOWN
    # cozmo_state.run()
    cozmo_camera.run()
    object_detector.run()
    feature_extractor.run()
    wac.run()
    debug.run()
    psi.run()

    input() # keep everything running

    mic.stop()
    asr.stop()
    iasr.stop()
    cozmo_refer.stop()
    cozmo_state.stop()
    dm.stop()
    cozmo_camera.stop()
    object_detector.stop()
    feature_extractor.stop()
    wac.stop()
    debug.stop()

  

cozmo.run_program(init_all, use_viewer=True, force_viewer_on_top=False )