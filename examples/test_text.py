import os
import sys
os.environ['GOOGLE_APPLICATION_CREDENTIALS']='/home/casey/substutute-ca5bdacf1d9a.json'
os.environ['PYOD'] = '/home/casey/git/PyOpenDial'
os.environ['RASA'] = "/home/casey/git/rasa_nlu"

from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule
from retico.core.text.common import SpeechRecognitionIU

# how to restart an utterance?
# had to change is_running to _is_running because opendial modules have an is_running() method

# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr pipeline
domain_dir = '/home/casey/git/PyOpenDial/domains/augi/augi.xml'

# instantiate modules
# mic = MicrophoneModule(5000)
# gasr = GoogleASRModule()
# iasr = IncrementalizeASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
dm = OpenDialModule(domain_dir=domain_dir)
debug = DebugModule()

# hook modules up to each other
# mic.subscribe(gasr)
# gasr.subscribe(iasr)
# iasr.subscribe(nlu)
nlu.subscribe(dm)
dm.subscribe(debug)

# initialize modules
# mic.run()
# gasr.run()
#iasr.run()
nlu.run()
dm.run()
debug.run()

trial = [('Smif', 'add'),
        ('Smif', 'revoke'),
        ('Smith', 'add'),
        ('wonks', 'add'),
        ('at', 'add'),
        ('at', 'revoke'),
        ('wonks','revoke'),
        ('works', 'add'),
        ('at','add'),
        ('Banf', 'add'),
        ('Banf', 'revoke'),
        ('Bank', 'add')]

iu_counter = 0
prefix = [None] # growing prefix of same-level links; start with None
for word,edit in trial:
    # revoke!
    if edit == 'revoke':
        iu.revoked = True
        res = nlu.revoke(prefix[-1]) # we can only revoke the most recently created iu
        prefix.pop() # remove it from the prefix
    # add!
    if edit == 'add':
        iu = SpeechRecognitionIU(None, iu_counter, prefix[-1], None)
        iu.set_asr_results([word], word, 0.9, 0.9, False)
        res = nlu.process_iu(iu)
        prefix.append(iu)
        
    iu_counter += 1 # iu_counter should always increment so new IUs have unique IDs from revoked IUs
    # print out NLU payload
    if res is not None:
        print(res.payload)

#input() # start streaming mic to ASR

# mic.stop()
# gasr.stop()
# iasr.stop()
nlu.stop()  
dm.stop()
debug.stop()