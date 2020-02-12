#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from retico.core import abstract
from retico.interop.ros.catkin_ws.src.rosretico.src.RosReader import process_iu
import datetime
import json

class RosIU(abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "Ros Incremental Unit"

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
        return RosReader

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
        self.reader = rospy.Subscriber(topic, String, process_iu, 10)
        rospy.init_node('RosReader', anonymous=True)
        rospy.loginfo('Initialized rosReader node with topic ' + str(topic))

    def setup(self):
        pass

class RosWriter(abstract.AbstractModule):

    """A ZeroMQ Writer Node

    Note: If you are using this to pass IU payloads to PSI, make sure you're passing JSON-formatable stuff (i.e., dicts not tuples)

    Attributes:
    topic (str): topic/scope that this writes to
        
    """
    @staticmethod
    def name():
        return "Ros Writer node"

    @staticmethod
    def description():
        return "A Node providing writing onto a Ros message"

    @staticmethod
    def output_iu():
        return None 

    @staticmethod
    def input_ius():
        return [abstract.IncrementalUnit] 

    def __init__(self, topic,  **kwargs):
        """Initializes the Ros writer.

        Args: topic(str): the topic/scope where the information will be read.
            
        """
        super().__init__(**kwargs)
        self.topic = topic
        self.publisher = rospy.Publisher(topic, String, queue_size=10)
        rospy.init_node('RosWriter', anonymous=True)
        rospy.loginfo('Initialized rosWriter node with topic ' + str(topic))

    def process_iu(self, input_iu):
        '''
        This assumes that the message is json formatted, then packages it as payload into an IU
        '''
        payload = {}
        payload['message'] = json.dumps(input_iu.payload)
        payload['originatingTime'] = datetime.datetime.now().isoformat()
        self.publisher.publish(payload)
        rospy.loginfo('iu payload: ' + json.dumps(payload).encode('utf-8'))
