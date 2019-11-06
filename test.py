from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.modules.opendial.dm import OpenDialModule
from retico.core.text.asr import IncrementalizeASRModule

# how to restart an utterance?
# had to change is_running to _is_running because opendial modules have an is_running() method

# run: export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr pipeline
domain_dir = '/home/casey/git/PyOpenDial/domains/augi/augi.xml'

# instantiate modules
mic = MicrophoneModule(5000)
gasr = GoogleASRModule()
iasr = IncrementalizeASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
dm = OpenDialModule(domain_dir=domain_dir)
debug = DebugModule()

# hook modules up to each other
mic.subscribe(gasr)
gasr.subscribe(iasr)
iasr.subscribe(nlu)
nlu.subscribe(dm)
dm.subscribe(debug)

# initialize modules
mic.run()
gasr.run()
iasr.run()
nlu.run()
dm.run()
debug.run()

input() # start streaming mic to ASR

mic.stop()
gasr.stop()
iasr.stop()
nlu.stop()  
dm.stop()
debug.stop()