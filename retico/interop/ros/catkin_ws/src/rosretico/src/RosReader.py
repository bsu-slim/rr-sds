#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from retico.core import abstract
from retico.core.dialogue.common import DialogueActIU
import datetime
import json

class RosReaderIU(abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "ZeroMQ Incremental Unit"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 payload=None, **kwargs):
        """Initialize the DialogueActIU with act and concepts.

        Args:
            act (string): A representation of the act.
            concepts (dict): A representation of the concepts as a dictionary.
        """
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=payload)

    def set_payload(self, payload):
        self.payload = payload

class RosReader(abstract.AbstractProducingModule):
    """A ros Reader Module

    Attributes:
        
    """
    @staticmethod
    def name():
        return "ros Reader node"

    @staticmethod
    def description():
        return "A Module providing reading from a ZeroMQ bus"

    @staticmethod
    def output_iu():
        return None

    def process_iu(self, input_iu):
        rospy.loginfo('iu payload: ' + input_iu.msg)
        output_iu = self.create_iu()
        output_iu.set_payload(input_iu.data)
        
    def __init__(self, topic,  **kwargs):
        """Initializes the ros reader node.

        Args: topic(str): the topic/scope where the information will be read.
            
        """
        super().__init__(**kwargs)
        self.topic = topic
        rospy.init_node('RosReader', anonymous=True)
        rospy.loginfo('Initialized rosReader node with topic ' + str(topic))
        self.reader = rospy.Subscriber(topic, String, self.process_iu, 10)

    def setup(self):
        pass