"""A module for eSpeak provided by azure"""


from collections import deque
import threading
import time
from espeakng import ESpeakNG
import numpy as np
# retico
from retico.core import abstract, text
from retico.core.text.common import TextIU
from retico.core.text.common import SpeechRecognitionIU
from retico.core.text.asr import IncrementalizeASRModule
from retico.core.audio.io import StreamingSpeakerModule
from retico.core.audio.io import AudioIU
# cv2
import cv2
from random import seed
from random import random
seed(2)

# eSpeak

class eSpeakModule(abstract.AbstractModule):
    """An eSpeak module using Microsoft Azure."""

    @staticmethod
    def name():
        return "eSpeak module"

    @staticmethod
    def description():
        return "eSpeak module"

    @staticmethod
    def input_ius():
        return [TextIU]
   

    
   
    def process_iu(self, input_iu):
        self.queue.clear() # drop frames, if still waiting
        self.queue.append(input_iu)
        return None
    
    def __init__(self, **kwargs):
         super().__init__(**kwargs)

         self.queue = deque()
         self.esp = ESpeakNG()
         self.iasr = IncrementalizeASRModule()

    def run_detector(self):
        self.esp.voice = 'en-us'
        while True:
            
            if len(self.queue) == 0:
                time.sleep(0.5)
                continue
            
            input_iu = self.queue.popleft()
            text = self.iasr.get_increment(input_iu)
            
            self.esp.say(text.get_text(),sync=True)
            print(text.get_text())
            time.sleep(0.5)

    def setup(self):
       
        t = threading.Thread(target=self.run_detector)
        t.start()
