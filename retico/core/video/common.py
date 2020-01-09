"""
This module redefines the abstract classes to fit the needs of video processing.
"""

from retico.core import abstract


class VideoIU(abstract.IncrementalUnit):
    """An image incremental unit that receives raw image data from a source.

    The image contained should be numpy.ndarray from cv2.

    Attributes:
        creator (AbstractModule): The module that created this IU
        previous_iu (IncrementalUnit): A link to the IU created before the
            current one.
        grounded_in (IncrementalUnit): A link to the IU this IU is based on.
        created_at (float): The UNIX timestamp of the moment the IU is created.
        raw_video (numpy.ndarray): The raw video of this IU
    """

    @staticmethod
    def type():
        return "Video IU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                raw_image=None, **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=raw_image)
        self.raw_image = raw_image


class WebcamIU(VideoIU):
    """A type of video incremental unit that contains an image extracted from a webcam.
    Includes the width and height of the image.
    """

    @staticmethod
    def type():
        return "Webcam IU"

    def __init__(self, image_width=None, image_height=None, frame_rate=None, **kwargs):
        super().__init__(**kwargs)
        self.image_width = image_width
        self.image_height = image_height
        self.frame_rate = frame_rate

    def set_video(self, raw_image, image_width, image_height, frame_rate):
        """Sets the video content of the IU."""
        self.raw_image = raw_image
        self.payload = raw_image
        self.image_width = image_width
        self.image_height = image_height
        self.frame_rate = frame_rate


