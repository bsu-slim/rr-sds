#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from retico.core import abstract
import rospy
import datetime
import json

class RosTopic(abstract.AbstractModule):
    """A ROS Writer Node

    Note: If you are using this to pass IU payloads to PSI, make sure you're passing JSON-formatable stuff (i.e., dicts not tuples)

    Attributes:
    topic (str): topic/scope that this writes to
        
    """
    @staticmethod
    def name():
        return "ROS Writer node"

    @staticmethod
    def description():
        return "A Node providing writing onto a Ros message"

    @staticmethod
    def output_iu():
        return abstract.IncrementalUnit

    @staticmethod
    def input_ius():
        return [abstract.IncrementalUnit] 

    def __init__(self, topic, **kwargs):
        """Initializes the Ros writer.

        Args: topic(str): the topic/scope where the information will be read.
            
        """
        super().__init__(**kwargs)
        self.publisher = rospy.Publisher(str(topic), String, queue_size=10)
        rospy.loginfo('Created publisher on topic ' + str(topic))

    def process_iu(self, input_iu):
        self.publisher.publish(input_iu.payload)

    def setup(self):
        pass