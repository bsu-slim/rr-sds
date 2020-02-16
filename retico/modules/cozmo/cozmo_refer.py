import functools
import threading
import time
import asyncio
import sys
from collections import deque
import random

# retico
from retico.core import abstract
from retico.core.dialogue.common import DialogueDecisionIU
from retico.core.dialogue.common import GenericDictIU
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
        return [GenericDictIU]

    def __init__(self, robot, use_viewer=False, force_viewer_on_top=False, **kwargs):
        super().__init__(**kwargs)
        self._use_viewer = use_viewer
        self._force_viewer_on_top = force_viewer_on_top
        self.robot = robot
        self._last_command = None
        self._input_iu = None
        self._current_word = None
        self.current_objects = None
        self.cb = CozmoBehaviors(robot)
        self.queue = deque()
        t = threading.Thread(target=self.run_dispatcher)
        t.start()

    def update_dialogue_state(self, signal, value):
        output_iu = GenericDictIU(creator=self, iuid=self.iu_counter)
        self.iu_counter+=1 # inherited
        output_iu.set_payload({signal:value})
        self.append(output_iu)

    def run_command(self, command, input_iu):
        if self.robot is None: return
        if command is None: return

        if '(' in command and ')' in command:
            word = command[command.find('(')+1:command.find(')')]
            self._current_word = word
            self.cb.say(word)
            command = command[:command.find('(')]

        if ':' in command:
            confidence = float(command[command.find(':')+1:])
            command = command[:command.find(':')]

        self._last_command = command
        self._input_iu = input_iu

        try:

            print('running command:', command)
            self.cb.camera_on()
            time.sleep(0.2)
            self.cb.camera_off()
            # while self.robot.is_moving:
            #     self.robot.stop_all_motors()
            if 'begin_explore' == command:
                self.update_dialogue_state('exploring', True)
                self.cb.explore()
            if 'align_to_object' == command:
                for _ in range(3):
                    self.cb.turn_toward_top_object()
                self.update_dialogue_state('aligned', True)
            if 'approach_object' == command:
                self.cb.turn_toward_top_object()
                drive_dist = self.cb.go_to_top_object()
                if drive_dist < 5:
                    print('NEAR OBJECT')
                    self.update_dialogue_state('near_object', True)
            if 'check_confidence' == command:
                print(self._current_word, confidence)

                self.update_dialogue_state('aligned', False)
                self.update_dialogue_state('near_object', False)

                if confidence > 0.6:
                    self.cb.say(self._current_word)
                    self.cb.indicate_object()
                    self.update_dialogue_state('word_to_find', None)
                    self.update_dialogue_state('exploring', False)
                else:
                    self.cb.start_position()
                    w = random.choice(['hmmm', 'uhh', 'that'])
                    self.cb.say("{} not {}".format(w, self._current_word))
                    self.cb.back_up()
                    self._last_command = 'begin_explore'
                    self.update_dialogue_state('begin_explore', True)

        except cozmo.exceptions.RobotBusy:
            print('robot is busy')
            

        # output_iu = self.create_iu(self._input_iu)
        # output_iu.set_payload({'':0})
        # self.append(output_iu)




    def run_dispatcher(self):

         while True:
            if len(self.queue) == 0:
                time.sleep(3.0)
                self.run_command(self._last_command, self._input_iu)
                continue

            input_iu = self.queue.popleft()
            decision = input_iu.payload['decision']
            concepts = input_iu.payload['concepts']
            print('new decision', decision)
            self.run_command(decision, input_iu)

    def process_iu(self, input_iu):
        
        if isinstance(input_iu, DetectedObjectsIU):
            self.cb.set_current_objects(input_iu.payload)
            return None

        self.queue.clear() # drop frames, if still waiting
        self.queue.append(input_iu)

        return None


    def new_utterance(self):
        pass
        
    def setup(self):
        pass
        