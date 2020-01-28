import functools
import threading
import time
import asyncio
import sys

# retico
from retico.core import abstract
from retico.core.dialogue.common import DialogueDecisionIU
from retico.modules.cozmo.cozmo_behaviors import CozmoBehaviors
from retico.core.visual.common import DetectedObjectsIU

# cozmo
import sys
import os
sys.path.append(os.environ['COZMO'])
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps


class CozmoReferModule(abstract.AbstractModule):

    @staticmethod
    def name():
        return "Cozmo Refer Module"

    @staticmethod
    def description():
        return "A Module that maps from DM decisions to Cozmo Robot actions in a reference task"

    @staticmethod
    def input_ius():
        return [DialogueDecisionIU,DetectedObjectsIU]

    @staticmethod
    def output_iu():
        return None # this is an output module so it doesn't produce any IUs

    def __init__(self, robot, use_viewer=False, force_viewer_on_top=False, **kwargs):
        super().__init__(**kwargs)
        self._use_viewer = use_viewer
        self._force_viewer_on_top = force_viewer_on_top
        self.robot = robot
        self._last_command = None
        self.current_objects = None
        self.cb = CozmoBehaviors(robot)

    def run_clarify(self):
        self.robot.say_text('uh', in_parallel=True)

    def run_command(self, command):
        if self.robot is None: return
        print('got the command:', command)
        while self.robot.is_moving:
            self.robot.stop_all_motors()
        if 'begin_explore' == command:
            for i in range(3):
                self.cb.explore()
        if 'align_to_object' == command:
            self.cb.turn_toward_top_object()
            self.cb.go_to_top_object()


    def process_iu(self, input_iu):
        
        if isinstance(input_iu, DetectedObjectsIU):
            self.cb.set_current_objects(input_iu.payload)
            return

        decision = input_iu.payload['decision']
        concepts = input_iu.payload['concepts']
        
        self.run_command(decision)
        
    def setup(self):
        pass
        