import functools
import threading
import time
import asyncio
import sys

# retico
from retico.core import abstract
from retico.core.robot.common import RobotStateIU

# cozmo
import sys
import os
sys.path.append(os.environ['COZMO'])
import cozmo

import queue
import numpy as np



class CozmoStateModule(abstract.AbstractProducingModule):
    '''
    use_viewer=True must be set in cozmo.run_program
    '''

    @staticmethod
    def name():
        return "Cozmo State Tracking Module"

    @staticmethod
    def description():
        return "A module that tracks the state of Cozmo, including camera frames."

    @staticmethod
    def output_iu():
        return RobotStateIU

    def __init__(self, robot, exposure=0.5, gain=0.5, max_frames=20, **kwargs):
        super().__init__(**kwargs)
        self.robot = robot
        self.exposure_amount = exposure
        self.gain_amount = gain
        self.state_queue = queue.Queue(maxsize=1)
        self.max_frames = max_frames
        

    def process_iu(self, input_iu):
        if not self.state_queue.empty():
            state = self.state_queue.get()
            output_iu = self.create_iu(input_iu)
            output_iu.set_state(state)
            return output_iu

        return None

    
    def setup(self):

        self.num_frames = 0

        def state_change_update(evt, obj=None, tap_count=None, **kwargs):

            robot = kwargs['robot']
            state = robot.get_robot_state_dict()
    
            state.update({"left_wheel_speed":str(robot.left_wheel_speed)})
            state.update({"right_wheel_speed":str(robot.right_wheel_speed)})
            state.update({"battery_voltage":str(robot.battery_voltage)})
            state.update({"robot_id":str(robot.robot_id)})
            state.update({"time":str(time.time())})
            state.update({'face_count': str(robot.world.visible_face_count())})


            if self.num_frames >= self.max_frames:
                self.num_frames = 0
            else:
                self.num_frames += 1
                return

            try:
                if self.state_queue.empty(): 
                    self.state_queue.put(state)
            except queue.Full:
                pass

        self.robot.add_event_handler(cozmo.robot.EvtRobotStateUpdated, state_change_update)



