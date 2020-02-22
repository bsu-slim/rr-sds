"""
This module redefines the abstract classes to fit the needs of visual processing.
"""

from retico.core import abstract
import base64
import json
import numpy as np
import psutil
from PIL import Image
import io

class ImageIU(abstract.IncrementalUnit):
    """An image incremental unit that receives raw image data from a source.

    Attributes:
        creator (AbstractModule): The module that created this IU
        previous_iu (IncrementalUnit): A link to the IU created before the
            current one.
        grounded_in (IncrementalUnit): A link to the IU this IU is based on.
        created_at (float): The UNIX timestamp of the moment the IU is created.
        image (bytes[]): The image of this IU
        rate (int): The frame rate of this IU
        nframes (int): The number of frames of this IU
    """

    @staticmethod
    def type():
        return "Image IU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 rate=None, nframes=None, image=None,
                 **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=image)
        self.image = image
        self.rate = rate
        self.nframes = nframes

    def set_image(self, image, nframes, rate):
        """Sets the audio content of the IU."""
        self.image = image
        self.payload = image
        self.nframes = int(nframes)
        self.rate = int(rate)

    def get_json(self):
        payload = {}
        payload['image'] = np.array(self.payload).tolist()
        payload['nframes'] = self.nframes
        payload['rate'] = self.rate
        return payload

    def create_from_json(self, json_dict):
        self.image =  Image.fromarray(np.array(json_dict['image'], dtype='uint8'))
        self.payload = self.image
        self.nframes = json_dict['nframes']
        self.rate = json_dict['rate']


class DetectedObjectsIU(abstract.IncrementalUnit):
    """An image incremental unit that maintains a list of detected objects and their bounding boxes.

    Attributes:
        creator (AbstractModule): The module that created this IU
        previous_iu (IncrementalUnit): A link to the IU created before the
            current one.
        grounded_in (IncrementalUnit): A link to the IU this IU is based on.
        created_at (float): The UNIX timestamp of the moment the IU is created.
    """

    @staticmethod
    def type():
        return "Detected Objects IU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 rate=None, nframes=None, sample_width=None, raw_audio=None,
                 **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=None)
        self.detected_objects = None
        self.num_objects = 0
        self.image = None

    def set_detected_objects(self, image, detected_objects):
        """Sets the audio content of the IU."""
        self.image = image
        self.payload = detected_objects
        self.detected_objects = detected_objects
        self.num_objects = detected_objects['num_objs']

    def get_json(self):
        payload = {}
        payload['image'] = np.array(self.payload).tolist()
        payload['detected_objects'] = self.detected_objects
        payload['num_objects'] = self.num_objects
        return payload

    def create_from_json(self, json_dict):
        self.image =  Image.fromarray(np.array(json_dict['image'], dtype='uint8'))
        self.detected_objects = json_dict['detected_objects']
        self.payload = self.detected_objects
        self.num_objects = json_dict['num_objects']

class ObjectFeaturesIU(abstract.IncrementalUnit):
    """An image incremental unit that maintains a list of feature vectors for detected objects in a scene.

    Attributes:
        creator (AbstractModule): The module that created this IU
        previous_iu (IncrementalUnit): A link to the IU created before the
            current one.
        grounded_in (IncrementalUnit): A link to the IU this IU is based on.
        created_at (float): The UNIX timestamp of the moment the IU is created.
    """

    @staticmethod
    def type():
        return "Object Features IU"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 rate=None, nframes=None, sample_width=None, raw_audio=None,
                 **kwargs):
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=None)
        self.object_features = None
        self.num_objects = 0
        self.image = None

    def set_object_features(self, image, object_features):
        """Sets the audio content of the IU."""
        self.image = image
        self.payload = object_features
        self.object_features = object_features
        self.num_objects = len(object_features)

    def get_json(self):
        payload = {}
        payload['image'] = np.array(self.payload).tolist()
        payload['object_features'] = self.object_features
        payload['num_objects'] = self.num_objects
        return payload

    def create_from_json(self, json_dict):
        self.image =  Image.fromarray(np.array(json_dict['image'], dtype='uint8'))
        self.object_features = json_dict['object_features']
        self.payload = self.detected_objects
        self.num_objects = json_dict['num_objects']

class ImageCropperModule(abstract.AbstractModule):
    """A module that crops images"""

    @staticmethod
    def name():
        return "Image Cropper Module"

    @staticmethod
    def description():
        return "A module that crops images"


    @staticmethod
    def input_ius():
        return [ImageIU]

    @staticmethod
    def output_iu():
        return ImageIU

    def __init__(self, top=-1, bottom=-1, left=-1, right=-1, **kwargs):
        """
        Initialize the Webcam Module.
        Args:
            width (int): Width of the image captured by the webcam; will use camera default if unset
            height (int): Height of the image captured by the webcam; will use camera default if unset
            rate (int): The frame rate of the recording; will use camera default if unset
        """
        super().__init__(**kwargs)
        self.top =  top
        self.bottom = bottom
        self.left = left
        self.right = right

    def process_iu(self, input_iu):
        image = input_iu.image
        width, height = image.size
        left = self.left if self.left != -1 else 0
        top = self.top if self.top != -1 else 0
        right = self.right if self.right != -1 else width
        bottom = self.bottom if self.bottom != -1 else height
        image = image.crop((left, top, right, bottom)) 
        output_iu = self.create_iu(input_iu)
        output_iu.set_image(image, input_iu.nframes, input_iu.rate)
        return output_iu
