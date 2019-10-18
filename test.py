from retico.core.audio.io import MicrophoneModule
from retico.core.debug.console import DebugModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.modules.google.asr import GoogleASRModule
from retico.modules.rasa.nlu import RasaNLUModule

# use incremental rasa insead

# run export GOOGLE_APPLICATION_CREDENTIALS=/home/casey/substutute-ca5bdacf1d9a.json

model_dir = '/home/casey/git/defclar/models/nlu/model_20191018-084937'

mic = MicrophoneModule(5000)
asr = GoogleASRModule()
nlu = RasaNLUModule(model_dir=model_dir)
debug = DebugModule()
#m2 = StreamingSpeakerModule(5000)

mic.subscribe(asr)
asr.subscribe(nlu)
nlu.subscribe(debug)

mic.run()
asr.run()
nlu.run()
debug.run()

input()

mic.stop()
asr.stop()
nlu.run()   
debug.stop()