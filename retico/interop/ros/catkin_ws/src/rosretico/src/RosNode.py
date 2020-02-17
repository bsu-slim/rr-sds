#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from retico.core import abstract
import rospy
import datetime
import json

class RosNode(abstract.AbstractModule):
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

    def __init__(self,  **kwargs):
        """Initializes the Ros writer.

        Args: topic(str): the topic/scope where the information will be read.
            
        """
        super().__init__(**kwargs)
        rospy.loginfo('Initialized Robot_Ready_SDS_RosNode')
        rospy.init_node("Robot_Ready_SDS_RosNode", anonymous=True)

    def process_iu(self, input_iu):
        pass

    def setup(self):
        pass