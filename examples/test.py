import os
import sys
import time
import warnings
warnings.filterwarnings('ignore')
# export LD_LIBRARY_PATH=/home/casey/anaconda3/envs/py-opendial/lib/python3.6/site-packages/torch/lib/:$LD_LIBRARY_PATH
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"
os.environ['TF_RESEARCH'] = '/home/casey/git/tfmodels/research'
os.environ['TF_SLIM'] = '/home/casey/git/tfmodels/research/slim'

import cv2

from retico.core.audio.io import MicrophoneModule
from retico.core.audio.io import RespeakerMicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.azure.asr import AzureASRModule
from retico.modules.rasa.nlu import RasaNLUModule
# from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule
from retico.modules.keras.object_features import KerasObjectFeatureExtractorModule
from retico.modules.azure.object_detection import AzureObjectDetectionModule
from retico.modules.slim.words_as_classifiers import WordsAsClassifiersModule
from retico.modules.google.od import MaskrRCNNObjectDetection
from retico.core.visual.common import ImageIU

# how to restart an utterance?
# had to change is_running to _is_running because opendial modules have an is_running() method

# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr pipeline
domain_dir = '/home/casey/git/PyOpenDial/domains/augi/augi.xml'
aod_endpoint = "https://slimcomputervision.cognitiveservices.azure.com/"
aod_key = "59bfd2dc248a4d08957edf7a6eb6331f"
wac_dir = '/home/casey/git/retico/data/wac/subset'
mask_rcnn_labels = '/home/casey/git/retico/data/maskrcnn/label_map.pbtxt'
mask_rcnn_model = '/home/casey/git/retico/data/maskrcnn/frozen_inference_graph.pb'

# instantiate modules
mic = MicrophoneModule(1000)
# mic = RespeakerMicrophoneModule('192.168.20.49:8000')
#asr = AzureASRModule("179eaa4b8fc54e0fa5115ba5d14883f2")
# asr = GoogleASRModule(rate=16000)
asr = GoogleASRModule()
iasr = IncrementalizeASRModule()
obj_det = MaskrRCNNObjectDetection(mask_rcnn_labels, mask_rcnn_model)
# nlu = RasaNLUModule(model_dir=model_dir)
# dm = OpenDialModule(domain_dir=domain_dir)
# obj_det = AzureObjectDetectionModule(key=aod_key,endpoint=aod_endpoint)
feat_ext = KerasObjectFeatureExtractorModule()
wac = WordsAsClassifiersModule(wac_dir=wac_dir)

debug = DebugModule()



# hook modules up to each other
mic.subscribe(asr)
asr.subscribe(iasr)
iasr.subscribe(wac)
# nlu.subscribe(dm)
# dm.subscribe(debug)
obj_det.subscribe(feat_ext)
feat_ext.subscribe(wac)

# initialize modules
mic.run()
asr.run()
iasr.run()
# nlu.run()
# dm.run()
obj_det.run()
feat_ext.run()
wac.run()
debug.run()

img = cv2.imread('/home/casey/git/retico/examples/tmp0.13436424411240122.png', cv2.IMREAD_UNCHANGED)
img_iu = ImageIU()
img_iu.set_image(img, 1, 1)

obj_det.append(img_iu)
# feats = feat_ext.process_iu(output_iu)

while wac.word_buffer is None:
    time.sleep(1.0)
    
wac.process_iu(feats)

input() # keep things running

mic.stop()
asr.stop()
# iasr.stop()
# nlu.stop()  
# dm.stop()
obj_det.stop()
feat_ext.stop()
wac.stop()
debug.stop()