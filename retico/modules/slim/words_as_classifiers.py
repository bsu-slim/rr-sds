"""A module for grounded semantics"""

# retico
from retico.core import abstract
from retico.core.visual.common import ObjectFeaturesIU
from retico.core.text.common import SpeechRecognitionIU
from retico.modules.slim.wac import WAC
from retico.modules.slim.common import GroundedFrameIU

import numpy as np
import os
import os.path as osp
from tqdm import tqdm
import sys
import threading


class WordsAsClassifiersModule(abstract.AbstractModule):
    """A model of grounded semantics. 

    Attributes:
        model_dir(str): directory where WAC classifiers are saved
    """

    @staticmethod
    def name():
        return "SLIM Group WAC Model"

    @staticmethod
    def description():
        return "WAC Visually-Grounded Model"

    @staticmethod
    def input_ius():
        return [ObjectFeaturesIU,SpeechRecognitionIU]

    @staticmethod
    def output_iu():
        return GroundedFrameIU

    def __init__(self, wac_dir, **kwargs):
        """Loads the WAC models.

        Args:
            model_dir (str): The path to the directory of WAC classifiers saved as pkl files.
        """
        super().__init__(**kwargs)
        self.wac = WAC(wac_dir)
        self.word_buffer = None
        self.itts = 0
        self.train_mode = False

    def process_iu(self, input_iu):

        frame = {}
        if isinstance(input_iu, SpeechRecognitionIU):
            self.word_buffer = input_iu.get_text().lower().split()[0]
            if not self.train_mode:
                frame['word_to_find'] = self.word_buffer
        
        # when new objects are observed (i.e., not SpeechRecognitionIUs)
        if isinstance(input_iu, ObjectFeaturesIU):
            objects = input_iu.payload
            # WAC wants a list of intents (objectIDs) and their corresponding features in a tuple
            intents = objects.keys()
            features = [np.array(objects[obj_id]) for obj_id in objects]
            
            word,_ = self.wac.best_word((intents, features))
            frame['best_known_word'] = word
            

            # uncomment below for training
            if self.train_mode:
                if self.word_buffer is not None:
                    self.wac.add_observation(self.word_buffer, features[0], 1)
                    self.itts += 1
                    if self.itts % 25 == 0:
                        print('updating negatives')
                        self.wac.create_negatives()
                        print('training')
                        self.wac.train()
                        print('persisting')
                        self.wac.persist_model()

            
            if self.word_buffer is not None:
                target = self.wac.best_object(self.word_buffer, (intents, features))
                if target is None: return None
                frame['best_object'] = target[0] 
                frame['obj_confidence'] = target[1] 

        if len(frame) == 0: return None
        output_iu = self.create_iu(input_iu)
        output_iu.set_frame(frame)
        return output_iu

    def setup(self):
        if not self.train_mode:
            self.wac.load_model()