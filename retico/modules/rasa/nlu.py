"""A module for Natural Language Understanding provided by rasa_nlu"""

from retico.core import abstract
from retico.core.text.common import SpeechRecognitionIU
from retico.core.dialogue.common import DialogueActIU

import sys
sys.path.append("/home/casey/git/rasa_nlu")
from rasa.nlu.model import IncrementalInterpreter as Interpreter
#from rasa_nlu.model import Interpreter

class RasaNLUModule(abstract.AbstractModule):
    """A standard rasa NLU module.

    Attributes:
        model_dir (str): The path to the directory of the NLU model generated by
            rasa_nlu.train.
        config_file (str): The path to the json file containing the rasa nlu
            configuration.
    """

    @staticmethod
    def name():
        return "Rasa NLU Module"

    @staticmethod
    def description():
        return "A Module providing Natural Language Understanding by rasa_nlu"

    @staticmethod
    def input_ius():
        return [SpeechRecognitionIU]

    @staticmethod
    def output_iu():
        return DialogueActIU

    def __init__(self, model_dir, incremental=False, **kwargs):
        """Initializes the RasaNLUModule.

        Args:
            model_dir (str): The path to the directory of the NLU model
                generated by rasa_nlu.train.
        """
        super().__init__(**kwargs)
        self.model_dir = model_dir
        self.interpreter = None
        self.incremental = incremental
        self.lb_hypotheses = []
        self.cache = None
        self.started_prediction = False

    def process_result(self, result, input_iu):
        concepts = {}
        for entity in result.get("entities"):
            concepts[entity["entity"]] = entity["value"]
        act = result["intent"]["name"]
        confidence = result["intent"]["confidence"]
        #print('nlu', act, concepts, confidence)
        output_iu = self.create_iu(input_iu)
        output_iu.set_act(act, concepts, confidence)
        piu = output_iu.previous_iu
        if piu:
            if piu.act != output_iu.act or piu.concepts != output_iu.concepts:
                piu.revoked = True
        if input_iu.committed:
            output_iu.committed = True
            self.started_prediction = False
        else:
            self.started_prediction = True
        return output_iu

    def process_iu(self, input_iu):

        #print('asr', input_iu.get_text()) # TODO: asr is restart-incremental
        for word in input_iu.get_text().split():
            text_iu = (word, "add") # only handling add for now
            print('nlu add({})'.format(word))
            result = self.interpreter.parse_incremental(text_iu)
        #print(result)

        #result = self.interpreter.parse(input_iu.get_text())
        #print(result)
        return self.process_result(result, input_iu)
        

    def process_revoke(self, revoked_iu):
        for word in reversed(revoked_iu.get_text().split()):
            text_iu = (word, "revoke") 
            print('nlu revoke({})'.format(word))
            result = self.interpreter.parse_incremental(text_iu)
        result = self.process_result(result, revoked_iu)

        if len(self._iu_stack) > 0:
            last_output_iu = self._iu_stack.pop()
            self.revoke(last_output_iu)
        
        return result

    def setup(self):
        self.interpreter = Interpreter.load(self.model_dir)
