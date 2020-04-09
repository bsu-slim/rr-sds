"""A module for processing received asr IUs from psi"""

# retico
from retico.core import abstract
from retico.core.text.common import SpeechRecognitionIU
from retico.interop.zeromq.io import ZeroMQIU

# psiAsrReceiver
import sys
import os
import json

class psiAsrReceiverModule(abstract.AbstractModule):
    """A module to handle json asr results from psi"""

    @staticmethod
    def name():
        return "psi asr receiver module"

    @staticmethod
    def description():
        return "A Module receiving ASR outputs from psi"

    @staticmethod
    def input_ius():
        return [ZeroMQIU]

    @staticmethod
    def output_iu():
        return SpeechRecognitionIU

    def __init__(self, **kwargs):
        """Initializes the psiReceiverModule.
        """
        super().__init__(**kwargs)
        self.all_ius = []
    

    def process_iu(self, input_iu):
        originateTime = input_iu.payload['originatingTime']
        message = input_iu.payload['message']
        messageBroken = json.loads(message)
        totalList = messageBroken['TotalList']
        SignalsList = messageBroken['SignalsList']

        output_iu = None

        for i in range(0, len(totalList)):
            output_iu = self.create_iu(input_iu)


            final = False
            if totalList[i]["EditType"] == "Commit":
                final = True
            output_iu.set_asr_results(
            totalList[i]['EditType'],
            totalList[i]['Payload']['Text'],
            totalList[i]['GroundedInIU'],
            totalList[i]['TimeStamp'],
            final,
            )
            self.all_ius.append(output_iu)
            self.append(output_iu)

        return

    def setup(self):
        pass


