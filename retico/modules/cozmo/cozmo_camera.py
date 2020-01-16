import functools
import threading
import time
import asyncio
import sys

# retico
from retico.core import abstract
from retico.core.visual.common import ImageIU

# cozmo
import sys
import os
sys.path.append(os.environ['COZMO'])
import cozmo

import queue
import numpy as np



class CozmoCameraModule(abstract.AbstractProducingModule):
    '''
    use_viewer=True must be set in cozmo.run_program
    '''

    @staticmethod
    def name():
        return "Cozmo Camera Tracking Module"

    @staticmethod
    def description():
        return "A module that tracks cozmo camera frames."

    @staticmethod
    def output_iu():
        return ImageIU

    def __init__(self, robot, exposure=0.5, gain=0.5, max_frames=30, **kwargs):
        super().__init__(**kwargs)
        self.robot = robot
        self.exposure_amount = exposure
        self.gain_amount = gain
        self.img_queue = queue.Queue(maxsize=1)
        self.max_frames = max_frames
        

    def process_iu(self, input_iu):
        if not self.img_queue.empty():
            img = self.img_queue.get()
            img = np.array(img)
            output_iu = self.create_iu(input_iu)
            output_iu.set_image(img, 1, 1)
            return output_iu

        return None

    def configure_camera(self):
        self.robot.camera.color_image_enabled = True
        # Lerp exposure between min and max times
        min_exposure = self.robot.camera.config.min_exposure_time_ms
        max_exposure = self.robot.camera.config.max_exposure_time_ms
        exposure_time = (1 - self.exposure_amount) * min_exposure + self.exposure_amount * max_exposure
        # Lerp gain
        min_gain = self.robot.camera.config.min_gain
        max_gain = self.robot.camera.config.max_gain
        actual_gain = (1-self.gain_amount)*min_gain + self.gain_amount*max_gain
        self.robot.camera.set_manual_exposure(exposure_time, actual_gain)


    def setup(self):

        self.num_frames = 0

        def handle_image(evt, obj=None, tap_count=None,  **kwargs):
            if self.num_frames >= self.max_frames:
                self.num_frames = 0
            else:
                self.num_frames += 1
                return

            try:
                if self.img_queue.empty(): 
                    self.img_queue.put(evt.image)
            except queue.Full:
                pass

        self.configure_camera()
        self.robot.add_event_handler(cozmo.camera.EvtNewRawCameraImage, handle_image)



