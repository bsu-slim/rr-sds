import threading
import time

# set vars before importing modules
import os
import sys


os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-78e88daf614b.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['TF_RESEARCH'] = '/home/casey/git/tfmodels/research'
os.environ['TF_SLIM'] = '/home/casey/git/tfmodels/research/slim'
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # error level
import warnings
warnings.filterwarnings('ignore')
import logging
logging.getLogger('tensorflow').setLevel(logging.ERROR)

# retico
from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule
from retico.modules.keras.object_features import KerasObjectFeatureExtractorModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.modules.google.od import MaskrRCNNObjectDetection
from retico.interop.zeromq.io import ZeroMQWriter
from retico.interop.zeromq.io import WriterSingleton
from retico.modules.slim.words_as_classifiers import WordsAsClassifiersModule
from retico.core.visual.io import WebcamModule
from retico.core.visual.common import ImageCropperModule
from retico.modules.misty.misty_camera import MistyCameraModule
from retico.modules.misty.misty_refer import MistyReferModule
from retico.modules.misty.misty_state import MistyStateModule
# had to change is_running to _is_running because opendial modules have an is_running() method


domain_dir = '/home/casey/git/retico/data/misty/dm/dialogue.xml'
aod_endpoint = "https://slimcomputervision.cognitiveservices.azure.com/"
aod_key = "59bfd2dc248a4d08957edf7a6eb6331f"
wac_dir = '/home/casey/git/retico/data/wac/subset'
mask_rcnn_labels = '/home/casey/git/retico/data/maskrcnn/label_map.pbtxt'
mask_rcnn_model = '/home/casey/git/retico/data/maskrcnn/frozen_inference_graph.pb'
misty_ip = '192.168.0.15'

opendial_variables = ['face_count', # 
                        'num_objs', # ObjectDetector
                        'exploring', # MistyReferModule
                        'aligned', #  MistyReferModule
                        'word_to_find', # WordsAsClassifiersModule
                        'best_object', # WordsAsClassifiersModule
                        'obj_confidence'] # WordsAsClassifiersModule

#
# INSTANTIATE MODULES
#
# mic = RespeakerMicrophoneModule('192.168.20.49:8000')
# asr = GoogleASRModule(rate=16000)

mic = MicrophoneModule(1000)
asr = GoogleASRModule()
iasr = IncrementalizeASRModule()
dm = OpenDialModule(domain_dir=domain_dir, variables=opendial_variables)
# cozmo_camera = WebcamModule()
misty_camera = MistyCameraModule(misty_ip)
misty_refer = MistyReferModule(misty_ip)
misty_state = MistyStateModule(misty_ip)
cropper = ImageCropperModule(top=200)
# object_detector = AzureObjectDetectionModule(aod_key, aod_endpoint)
object_detector = MaskrRCNNObjectDetection(mask_rcnn_labels, mask_rcnn_model, max_objs=1)
feature_extractor = KerasObjectFeatureExtractorModule()
wac = WordsAsClassifiersModule(wac_dir=wac_dir)
debug = DebugModule()

print('All modules instantiated.')

# psi related modules
# WriterSingleton should use the *source* ip address (i.e., this machine's)
# WriterSingleton(ip='10.255.146.186', port='12346') # create the zeromq writer
# iasr_psi = ZeroMQWriter(topic='asr')
# nlu_psi = ZeroMQWriter(topic='nlu')
# dm_psi = ZeroMQWriter(topic='dm')

# mic as input
mic.subscribe(asr)
asr.subscribe(iasr)
iasr.subscribe(wac)
wac.subscribe(dm)

# robot state as input
wac.subscribe(misty_refer)
misty_state.subscribe(misty_refer)
dm.subscribe(misty_refer)
misty_refer.subscribe(dm)
object_detector.subscribe(misty_refer)

# robot camera as input
misty_camera.subscribe(cropper)
cropper.subscribe(object_detector)
object_detector.subscribe(feature_extractor)
object_detector.subscribe(dm)
feature_extractor.subscribe(wac)

print('All modules subscribed.')


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
dm.run()
object_detector.run()
feature_extractor.run()
misty_state.run()
misty_refer.run()
misty_camera.run()
cropper.run()
wac.run()
# debug.run()
print('All modules running.')
# dm_psi.run()
# nlu_psi.run()
# iasr_psi.run()

input() # keep everything running

mic.stop()
asr.stop()
iasr.stop()
dm.stop()
object_detector.stop()
feature_extractor.stop()
wac.stop()
misty_refer.stop()
misty_camera.stop()
cropper.stop()

# dm_psi.stop()
# iasr_psi.stop()
# nlu_psi.stop()
