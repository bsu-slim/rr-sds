"""A module for facail emotion recognition provided by azure"""


from collections import deque
import threading
import time

# retico
from retico.core import abstract
from retico.core.visual.common import ImageIU
from retico.core.dialogue.common import GenericDictIU

# cv2
import cv2
from random import seed
from random import random
seed(2)

# azure
try:
    from azure.cognitiveservices.vision.face import FaceClient
    from msrest.authentication import CognitiveServicesCredentials
    from azure.cognitiveservices.vision.face.models import FaceAttributeType
except ImportError:
    print("""Importing the Azure Cognitive Services Vision SDK for Python failed.
    use the following pip command: 'pip install --upgrade azure-cognitiveservices-vision-face'
    Make sure you're using at least Python version 3.x or later.""")
    exit(1)


# azure
try:
    from azure.cognitiveservices.vision.computervision import ComputerVisionClient
    from msrest.authentication import CognitiveServicesCredentials
except ImportError:
    print("""Importing the Computer Vision SDK for Python failed.
    use the following pip command: 'pip install --upgrade azure-cognitiveservices-vision-computervision'
    Make sure you're using at least Python version 3.x or later.""")
    exit(1)


class AzureEmotionDetectionModule(abstract.AbstractModule):
    """An object detection module using Microsoft Azure.

    Attributes:
        key (str): key to Microsoft Cognitive Services for Object Detection API
    """

    @staticmethod
    def name():
        return "Microsoft Azure / Cognitive Services Object Detection"

    @staticmethod
    def description():
        return "An object detection module using Microsoft Azure."

    @staticmethod
    def input_ius():
        return [ImageIU]

    @staticmethod
    def output_iu():
        return GenericDictIU

    def authenticate(self):
        # Authenticating the Client
        self.face_client = FaceClient(self.endpoint, CognitiveServicesCredentials(self.key))

    def __init__(self, key, endpoint, **kwargs):
        """Initializes the object detector module, authenticates with Azure/MS Cognitive Services.

        Args:
            model_dir (str): The path to the directory of the NLU model
                generated by rasa_nlu.train.
        """
        super().__init__(**kwargs)
        self.face_client = None
        self.endpoint = endpoint
        self.key = key
        self.queue = deque()
        self.f = 'tmp{}.png'.format(random())


    def process_iu(self, input_iu):
        self.queue.clear() # drop frames, if still waiting
        self.queue.append(input_iu)

        return None

    def run_detector(self):
        while True:
            if len(self.queue) == 0:
                time.sleep(0.3)
                continue
            input_iu = self.queue.popleft()
            
            image = input_iu.payload
            cv2.imwrite(self.f, image)
            # Creating an image stream given an input image file.
            try:
                with open(self.f, "rb") as face_fd:
                    detected_faces = self.face_client.face.detect_with_stream(face_fd, return_face_attributes=FaceAttributeType.emotion)
            except FileNotFoundError:
                print("The file, {}, was not found.".format(input_file_path))
                exit(1)

            # Variables used later on.
            returning_dictionary = {}
            count = 0

            # Case: There are no faces detected in the image.
            if not detected_faces:
                return None
                # raise Exception('No face detected from image {}'.format(input_file_path))

            # Creating a unique dictionary object for each face detected from the given file and adding it 
            # to the returning_dictionary dictionary.
            for face in detected_faces:
                Dict = {}
                Dict["anger"] = face.face_attributes.emotion.anger
                Dict["contempt"] = face.face_attributes.emotion.contempt
                Dict["disgust"] = face.face_attributes.emotion.disgust
                Dict["fear"] = face.face_attributes.emotion.fear
                Dict["happiness"] = face.face_attributes.emotion.happiness
                Dict["neutral"] = face.face_attributes.emotion.neutral
                Dict["sadness"] = face.face_attributes.emotion.sadness
                Dict["surprise"] = face.face_attributes.emotion.surprise
                returning_dictionary["face"+str(count)] = Dict
                count += 1

            output_iu = self.create_iu(input_iu)
            output_iu.set_payload(returning_dictionary)
            self.append(output_iu)
            time.sleep(2)

    def setup(self):
        self.authenticate()
        t = threading.Thread(target=self.run_detector)
        t.start()