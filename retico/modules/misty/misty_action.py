import functools
import threading
import time
import asyncio
import sys

# retico
from retico.core import abstract
from retico.core.dialogue.common import DialogueDecisionIU

# misty
import sys
import os
from retico.modules.misty.mistyPy import Robot

class MistyAction(abstract.AbstractModule):

    @staticmethod
    def name():
        return "Misty II Action Module"

    @staticmethod
    def description():
        return "A Module that maps from DM decisions to Misty II Robot actions"

    @staticmethod
    def input_ius():
        return [DialogueDecisionIU]

    @staticmethod
    def output_iu():
        return None # this is an output module so it doesn't produce any IUs

    def __init__(self, ip, use_viewer=False, force_viewer_on_top=False, **kwargs):
        super().__init__(**kwargs)
        self.robot = Robot(ip)
        self._last_commands = None

    def run_clarify(self):
        pass

    def run_command(self, commands):
        if self.robot is None: return
        for command in commands:
            # if 'turn_left' == command:
            # if 'turn_right' == command:
            if 'forward' == command:
                self.robot.driveTime(80,0,2000)
            if 'backup' == command:
                self.robot.driveTime(-80,0,2000)
            # if 'lift_up' == command:
            # if 'lift_down' == command:
            if 'look_up' == command:
                self.robot.moveHead(0,-40,0, velocity = 100)
            if 'look_down' == command:
                self.robot.moveHead(0,40,0, velocity = 100)
            if 'look_ahead' == command:
                self.robot.moveHead(0,0,0, velocity = 100)
            if 'halt' == command:
                self.robot.stop()
            if 'repeat' == command:
                if self._last_commands is not None:
                    self.run_command(self._last_commands)
                    return # don't want `repeat` to be the last command

        self._last_commands = commands

    def process_iu(self, input_iu):
        
        decision,concepts = input_iu.payload
        
        if 'select' == decision:
            self.run_command(concepts.keys())
        if 'clarify' == decision:
            self.run_clarify()
        

    def setup(self):
        pass        