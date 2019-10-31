from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule

# how to restart an utterance?
# had to change is_running to _is_running because opendial modules have an is_running() method

# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

# model_dir = '/home/casey/git/defclar/models/nlu/model_20191021-103901' # incr rasa
#model_dir = '/home/casey/git/defclar/models/nlu/model_20191021-180503'
model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr pipeline

domain_dir = '/home/casey/git/PyOpenDial/domains/augi/augi.xml'

mic = MicrophoneModule(5000)
asr = GoogleASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
dm = OpenDialModule(domain_dir=domain_dir)
debug = DebugModule()
#m2 = StreamingSpeakerModule(5000)

mic.subscribe(asr)
asr.subscribe(nlu)
# nlu.subscribe(dm)
dm.subscribe(debug)

mic.run()
asr.run()
nlu.run()
dm.run()
debug.run()

input()

mic.stop()
asr.stop()
nlu.stop()  
dm.stop()
debug.stop()