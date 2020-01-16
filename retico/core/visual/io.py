"""
A module for handling video related input.
"""

from retico.core import abstract
from retico.core.visual.common import ImageIU
import numpy as np
import cv2


class WebcamModule(abstract.AbstractProducingModule):
    """A module that produces IUs containing images that are captured by
    a web camera."""

    @staticmethod
    def name():
        return "Webcam Module"

    @staticmethod
    def description():
        return "A prodicing module that records images from a web camera."

    @staticmethod
    def output_iu():
        return ImageIU

    def __init__(self, width=None, height=None, rate=None, **kwargs):
        """
        Initialize the Webcam Module.
        Args:
            width (int): Width of the image captured by the webcam; will use camera default if unset
            height (int): Height of the image captured by the webcam; will use camera default if unset
            rate (int): The frame rate of the recording; will use camera default if unset
        """
        super().__init__(**kwargs)
        self.width = width
        self.height = height
        self.rate = rate
        self.cap = cv2.VideoCapture(0)

        self.setup()

        # self.video_buffer = queue.Queue()
        # self.stream = None

    def process_iu(self, input_iu):
        # if not self.video_buffer:
        #     return None
        ret, frame = self.cap.read() # ret should be false if camera is off
        if ret:
            output_iu = self.create_iu()
            # output_iu.set_image(frame, self.width, self.height, self.rate)
            output_iu.set_image(frame, 1, self.rate)
            return output_iu
        else:
            print('camera may not be on')

    def setup(self):
        """Set up the webcam for recording."""
        cap = self.cap
        if self.width != None:
            cap.set(3, self.width)
        else:
            self.width = int(cap.get(3))
        if self.height != None:
            cap.set(4, self.height)
        else:
            self.height = int(cap.get(4))
        if self.rate != None:
            cap.set(5, rate)
        else:
            self.rate = int(cap.get(5))

    def shutdown(self):
        """Close the video stream."""
        self.cap.release()
