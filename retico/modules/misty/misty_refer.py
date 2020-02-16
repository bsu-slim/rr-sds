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
from retico.core.visual.common import DetectedObjectsIU
from retico.core.dialogue.common import GenericDictIU
from retico.core.robot.common import RobotStateIU

# misty
import sys
import os
from retico.modules.misty.mistyPy import Robot

class MistyReferModule(abstract.AbstractModule):

    @staticmethod
    def name():
        return "Misty II Refer Module"

    @staticmethod
    def description():
        return "A Module that maps from DM decisions to Misty II Robot actions for a simple reference task"

    @staticmethod
    def input_ius():
        return [RobotStateIU,DialogueDecisionIU,DetectedObjectsIU]

    @staticmethod
    def output_iu():
        return None # this is an output module so it doesn't produce any IUs

    def __init__(self, ip, use_viewer=False, force_viewer_on_top=False, **kwargs):
        super().__init__(**kwargs)
        self.robot = Robot(ip)
        self._last_command = None
        self._input_iu = None
        self._current_word = None
        self.current_state = {}
        self.current_objects = None
        self.queue = deque()
        t = threading.Thread(target=self.run_dispatcher)
        t.start()

    def run_clarify(self):
        pass


    def update_dialogue_state(self, signal, value):
        output_iu = GenericDictIU(creator=self, iuid=self.iu_counter)
        self.iu_counter+=1
        output_iu.set_payload({signal:value})
        self.append(output_iu)

    def run_command(self, command, input_iu):
        if self.robot is None: return
        if command is None: return

        if '(' in command and ')' in command:
            word = command[command.find('(')+1:command.find(')')]
            self._current_word = word
            # say word
            command = command[:command.find('(')]

        if ':' in command:
            confidence = float(command[command.find(':')+1:])
            command = command[:command.find(':')]

        self._last_command = command
        self._input_iu = input_iu

        print('running command:', command)
        time.sleep(0.2)
        current_pitch = self.current_state['Actuator_HeadPitch'] if 'Actuator_HeadPitch' in self.current_state else 0.0
        current_yaw = self.current_state['Actuator_HeadYaw'] if 'Actuator_HeadYaw' in self.current_state else 0.0
        current_roll = self.current_state['Actuator_HeadRoll'] if 'Actuator_HeadRoll' in self.current_state else 0.0
        print(current_pitch, current_yaw, current_roll)
        # while self.robot.is_moving:
        self.robot.stop()
        if 'begin_explore' == command:
            self.update_dialogue_state('exploring', True)
            new_yaw = random.randint(-70,70) # move head left/right randomly
            new_pitch = random.randint(15,25)
            self.robot.move_head(current_roll, new_pitch, new_yaw)

        if 'align_to_object' == command:
            if 'object0' in self.current_objects:
                top_object = self.current_objects['object0']
            for _ in range(3):
                real_object, turn_angle = self.robot.find_coordinates(top_object)
                if real_object:
                    self.robot.move_head(current_roll, current_pitch, current_yaw + turn_angle)
                    self.update_dialogue_state('aligned', True)
        if 'check_confidence' == command:
            print(self._current_word, confidence)

            self.update_dialogue_state('aligned', False)
            self.update_dialogue_state('near_object', False)
            self.update_dialogue_state('exploring', False)

            if confidence > 0.6:
                # say word
                # indicate object
                self.robot.change_LED(0,255,0)
                time.sleep(2)
                self.robot.change_LED(0,0,0)
                self.update_dialogue_state('word_to_find', None)
                self.update_dialogue_state('exploring', False)
            else:
                self.set_start_position()
                w = random.choice(['hmmm', 'uhh', 'that'])
                # say "not x"
                #self.cb.say("{} not {}".format(w, self._current_word))
                self.robot.change_LED(255,0,0)
                time.sleep(2)
                self.robot.change_LED(0,0,0)
                self._last_command = 'begin_explore'
                self.update_dialogue_state('begin_explore', True)
                

    def process_iu(self, input_iu):
        
        if isinstance(input_iu, DetectedObjectsIU):
            self.current_objects = input_iu.payload
            return None

        if isinstance(input_iu, RobotStateIU):
            state_update = input_iu.payload
            for key in state_update:
                self.current_state[key] = state_update[key]
            return None

        self.queue.clear() # drop frames, if still waiting
        self.queue.append(input_iu)

        return None

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

    def set_start_position(self):
        roll = 0
        pitch = 25
        yaw = 0
        self.robot.move_head(roll, pitch, yaw)

        arm_deg = 30
        self.robot.move_arm('left', arm_deg)
        self.robot.move_arm('right', arm_deg)

        self.robot.change_LED(0,0,0)


    def setup(self):
        self.set_start_position()

    def new_utterance(self):
        pass
     