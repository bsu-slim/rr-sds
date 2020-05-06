import cv2
import sys
import numpy as np

#retico
from retico.core import abstract
from retico.core.visual.common import ImageIU
from retico.core.dialogue.common import GenericDictIU

class FaceDetectionModule(abstract.AbstractModule):
    """A face detection module using OpenCV"""

    @staticmethod
    def name():
        return "Face Recognizer"
    
    @staticmethod
    def description():
        return "A module that detects faces in a photo"

    @staticmethod
    def input_ius():
        return [ImageIU]

    @staticmethod
    def output_iu():
        return GenericDictIU

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.faceCascade = None
    
    def process_iu(self, input_iu):
        
        # Get user supplied values
        image = np.asarray(input_iu.image)

        # Read the image
        # image = cv2.imread(imagePath)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags = cv2.CASCADE_SCALE_IMAGE
        )
        
        face_count = 0
        returning_dictionary = {}

        # Draw a rectangle around the faces
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (250, 0, 180), 2)

            #cv2.imshow("Faces found", image)
            #cv2.waitKey(0)
            returning_dictionary[face_count] = [(x,y), (x+w, y+h)]
            face_count += 1

        if face_count == 0:
            return None
        else:
            #returning_dictionary['faces'] = face_count
            output_iu = self.create_iu(input_iu)
            output_iu.set_payload(returning_dictionary)
            #print(returning_dictionary)
            return output_iu

    def setup(self):
        # Create the haar cascade
        self.faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")




