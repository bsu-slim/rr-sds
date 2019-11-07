import functools
import threading
import time
import asyncio
import sys

# retico
from retico.core import abstract
from retico.core.dialogue.common import DialogueDecisionIU

# cozmo
import sys
import os
sys.path.append(os.environ['COZMO'])
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps


class CozmoAction(abstract.AbstractModule):

    @staticmethod
    def name():
        return "Cozmo Action Module"

    @staticmethod
    def description():
        return "A Module that maps from DM decisions to Cozmo Robot actions"

    @staticmethod
    def input_ius():
        return [DialogueDecisionIU]

    @staticmethod
    def output_iu():
        return None # this is an output module so it doesn't produce any IUs

    def __init__(self, robot, use_viewer=False, force_viewer_on_top=False, **kwargs):
        super().__init__(**kwargs)
        self._use_viewer = use_viewer
        self._force_viewer_on_top = force_viewer_on_top
        self.robot = robot
        self._last_commands = None

    def run_clarify(self):
        self.robot.say_text('uh', in_parallel=True)

    def run_command(self, commands):
        for command in commands:
            if 'turn_left' == command:
                self.robot.turn_in_place(degrees(60), in_parallel=True)#.wait_for_completed()
            if 'turn_right' == command:
                self.robot.turn_in_place(degrees(-60), in_parallel=True)#.wait_for_completed()
            if 'forward' == command:
                self.robot.drive_straight(distance_mm(1000), speed_mmps(100), in_parallel=True)#.wait_for_completed()
            if 'backup' == command:
                self.robot.drive_straight(distance_mm(-200), speed_mmps(50), in_parallel=True)#.wait_for_completed()
            if 'lift_up' == command:
                self.robot.set_lift_height(1.0, in_parallel=True)#.wait_for_completed()
            if 'lift_down' == command:
                self.robot.set_lift_height(0.0, in_parallel=True)#.wait_for_completed()
            if 'look_up' == command:
                self.robot.set_head_angle(degrees(44.5), in_parallel=True)#.wait_for_completed()
            if 'look_down' == command:
                self.robot.set_head_angle(degrees(-25), in_parallel=True)#.wait_for_completed()
            if 'look_ahead' == command:
                self.robot.set_head_angle(degrees(0), in_parallel=True)#.wait_for_completed()
            if 'halt' == command:
                self.robot.stop_all_motors()
                while self.robot.is_moving:
                    self.robot.stop_all_motors()
            if 'repeat' == command:
                if self._last_commands is not None:
                    self.run_command(self._last_commands)
                    return # don't want `repeat` to be the last command

        self._last_commands = commands

    def process_iu(self, input_iu):
        
        decision,concepts = input_iu.payload
        if self.robot is None: return
        if 'select' == decision:
            self.run_command(concepts.keys())
        if 'clarify' == decision:
            self.run_clarify()
        

    def setup(self):
        pass
        