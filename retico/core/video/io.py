"""
A module for handling video related input.
"""

from retico.core import abstract
from retico.core.video.common import VideoIU, WebcamIU
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
        return WebcamIU

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

        # self.video_buffer = queue.Queue()
        # self.stream = None

    def process_iu(self, input_iu):
        # if not self.video_buffer:
        #     return None
        ret, frame = self.cap.read() # ret should be false if camera is off
        if ret:
            output_iu = self.create_iu()
            output_iu.set_video(frame, self.width, self.height, self.rate)
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

# class AudioDispatcherModule(abstract.AbstractModule):
#     """An Audio module that takes a raw audio stream of arbitrary size and
#     outputs AudioIUs with a specific chunk size at the rate it would be produced
#     if the audio was being played.

#     This could be espacially useful when an agents' TTS module produces an
#     utterance, but this utterance should not be transmitted as a whole but in
#     an incremental way.

#     Attributes:
#         target_chunk_size (int): The size of each output IU in samples.
#         silence (bytes): A bytes array containing [target_chunk_size] samples
#             of silence that is dispatched when [continuous] is True and no input
#             IU is dispatched.
#         continuous (bool): Whether or not the dispatching should be continuous.
#             If True, AudioIUs with "silence" will be disptached if no input IUs
#             are being dispatched. If False, no IUs will be produced during
#             silence.
#         rate (int): The sample rate of the outout and the input IU.
#         sample_width (int): The sample with of the output and input IU.
#         speed (float): The speed of the dispatching. 1.0 means realtime.
#         dispatching_mutex (threading.Lock): The mutex if an input IU is
#             currently being dispatched.
#         audio_buffer (list): The current audio buffer containing the output IUs
#             that are currently dispatched.
#         run_loop (bool): Whether or not the dispatching loop is running.
#         interrupt (bool): Whether or not incoming IUs interrupt the old
#             dispatching
#     """

#     @staticmethod
#     def name():
#         return "Audio Dispatching Module"

#     @staticmethod
#     def description():
#         return (
#             "A module that transmits audio by splitting it up into" "streamable pakets."
#         )

#     @staticmethod
#     def input_ius():
#         return [SpeechIU]

#     @staticmethod
#     def output_iu():
#         return DispatchedAudioIU

#     def __init__(
#         self,
#         target_chunk_size,
#         rate=44100,
#         sample_width=2,
#         speed=1.0,
#         continuous=True,
#         silence=None,
#         interrupt=True,
#         **kwargs
#     ):
#         """Initialize the AudioDispatcherModule with the given arguments.

#         Args:
#             target_chunk_size (int): The size of each output IU in samples.
#             rate (int): The sample rate of the outout and the input IU.
#             sample_width (int): The sample with of the output and input IU.
#             speed (float): The speed of the dispatching. 1.0 means realtime.
#             continuous (bool): Whether or not the dispatching should be
#                 continuous. If True, AudioIUs with "silence" will be dispatched
#                 if no input IUs are being dispatched. If False, no IUs will be
#                 produced during silence.
#             silence (bytes): A bytes array containing target_chunk_size samples
#                 of silence. If this argument is set to None, a default silence
#                 of all zeros will be set.
#             interrupt (boolean): If this flag is set, a new input IU with audio
#                 to dispatch will stop the current dispatching process. If set to
#                 False, the "old" dispatching will be finished before the new one
#                 is started. If the new input IU has the dispatching flag set to
#                 False, dispatching will always be stopped.
#         """
#         super().__init__(**kwargs)
#         self.target_chunk_size = target_chunk_size
#         if not silence:
#             self.silence = generate_silence(target_chunk_size, sample_width)
#         else:
#             self.silence = silence
#         self.continuous = continuous
#         self.rate = rate
#         self.sample_width = sample_width
#         self._is_dispatching = False
#         self.dispatching_mutex = threading.Lock()
#         self.audio_buffer = []
#         self.run_loop = False
#         self.speed = speed
#         self.interrupt = interrupt

#     def is_dispatching(self):
#         """Return whether or not the audio dispatcher is dispatching a Speech
#         IU.

#         Returns:
#             bool: Whether or not speech is currently dispatched
#         """
#         with self.dispatching_mutex:
#             return self._is_dispatching

#     def set_dispatching(self, value):
#         """Set the dispatching value of this module in a thread safe way.

#         Args:
#             value (bool): The new value of the dispatching flag.
#         """
#         with self.dispatching_mutex:
#             self._is_dispatching = value

#     def process_iu(self, input_iu):
#         cur_width = self.target_chunk_size * self.sample_width
#         # If the AudioDispatcherModule is set to intterupt mode or if the
#         # incoming IU is set to not dispatch, we stop dispatching and clean the
#         # buffer
#         if self.interrupt or not input_iu.dispatch:
#             self.set_dispatching(False)
#             self.audio_buffer = []
#         if input_iu.dispatch:
#             # Loop over all frames (frame-sized chunks of data) in the input IU
#             # and add them to the buffer to be dispatched by the
#             # _dispatch_audio_loop
#             for i in range(0, input_iu.nframes, self.target_chunk_size):
#                 cur_pos = i * self.sample_width
#                 data = input_iu.raw_audio[cur_pos : cur_pos + cur_width]
#                 distance = cur_width - len(data)
#                 data += b"\0" * distance

#                 completion = float((i + self.target_chunk_size) / input_iu.nframes)
#                 if completion > 1:
#                     completion = 1

#                 current_iu = self.create_iu(input_iu)
#                 current_iu.set_dispatching(completion, True)
#                 current_iu.set_audio(
#                     data, self.target_chunk_size, self.rate, self.sample_width
#                 )
#                 self.audio_buffer.append(current_iu)
#             self.set_dispatching(True)
#         return None

#     def _dispatch_audio_loop(self):
#         """A method run in a thread that adds IU to the output queue."""
#         while self.run_loop:
#             with self.dispatching_mutex:
#                 if self._is_dispatching:
#                     if self.audio_buffer:
#                         self.append(self.audio_buffer.pop(0))
#                     else:
#                         self._is_dispatching = False
#                 if not self._is_dispatching:  # no else here! bc line above
#                     if self.continuous:
#                         current_iu = self.create_iu(None)
#                         current_iu.set_audio(
#                             self.silence,
#                             self.target_chunk_size,
#                             self.rate,
#                             self.sample_width,
#                         )
#                         current_iu.set_dispatching(0.0, False)
#                         self.append(current_iu)
#             time.sleep((self.target_chunk_size / self.rate) / self.speed)

#     def setup(self):
#         self.run_loop = True
#         t = threading.Thread(target=self._dispatch_audio_loop)
#         t.start()

#     def shutdown(self):
#         self.run_loop = False
#         self.audio_buffer = []

