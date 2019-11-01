from retico.core.debug.console import DebugModule
from retico.modules.rasa.nlu import RasaNLUModule
from retico.core.text.common import SpeechRecognitionIU

model_dir = '/home/casey/git/defclar/models/nlu_20191025-102321' # incr pipeline

nlu = RasaNLUModule(model_dir=model_dir)

nlu.run()

# test utterance that simulates ASR: 'Smith works at Bank' with misrecognitions
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
    

nlu.stop()