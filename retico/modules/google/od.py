"""A module for object detection provided by google"""


import sys
import os
sys.path.append(os.environ['TF_RESEARCH'])
sys.path.append(os.environ['TF_SLIM'])
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

from retico.modules.google.mask_rcnn import MaskRCNN


from collections import deque
import threading
import time
import matplotlib
from matplotlib import pyplot as plt
import numpy as np
import cv2
from PIL import Image

# retico
from retico.core import abstract
from retico.core.visual.common import ImageIU
from retico.core.visual.common import DetectedObjectsIU


class MaskrRCNNObjectDetection(abstract.AbstractModule):
    """An object detection module using Google MaskRCNN.

    Attributes:
        
    """

    @staticmethod
    def name():
        return "Google MaskRCNN Object Detection"

    @staticmethod
    def description():
        return "An object detection module using Google MaskRCNN."

    @staticmethod
    def input_ius():
        return [ImageIU]

    @staticmethod
    def output_iu():
        return DetectedObjectsIU

    def __init__(self,labels_path, model_path, max_objs=1, image_dims=(240, 320, 3),
                 conf_threshold=0, **kwargs):
        """Initializes the object detector module

        Args:

        """
        super().__init__(**kwargs)
        # matplotlib.use('TkAgg') 
        # print(matplotlib.get_backend())
        self.category_index = label_map_util.create_category_index_from_labelmap(labels_path, use_display_name=True)
        self.model = MaskRCNN(model_path, image_dims)
        self.queue = deque(maxlen=2)
        self.max_objs = max_objs
        self.image_dims = image_dims
        self.conf_threshold = conf_threshold

    def process_iu(self, input_iu):
        # self.queue.clear() # drop frames, if still waiting
        self.queue.append(input_iu)

        return None

    def run_detector(self):

        while True:
            if len(self.queue) == 0:
                time.sleep(0.1)
                continue
            
            input_iu = self.queue.popleft()
            image = input_iu.payload # assume PIL image
            image = image.resize((self.image_dims[1], self.image_dims[0])) #resize, if necessary
            output_dict = self.model.detect(np.array(image))
            boxes = output_dict['detection_boxes'][:self.max_objs]
            returning_dictionary = {}
            for ind,obj in enumerate(boxes):
                inner_dict = {}
                conf = output_dict['detection_scores'][ind]
                if conf < self.conf_threshold:
                    continue
                label = output_dict['detection_classes'][ind]
                inner_dict['ymin'] = obj[0]
                inner_dict['xmin'] = obj[1]
                inner_dict['ymax'] = obj[2]
                inner_dict['xmax'] = obj[3]
                inner_dict['label'] = label
                inner_dict['confidence'] = conf
                returning_dictionary["object"+str(ind)] = inner_dict
            if len(returning_dictionary) == 0: return
            returning_dictionary['num_objs'] = len(returning_dictionary)
            output_iu = self.create_iu(input_iu)
            output_iu.set_detected_objects(image, returning_dictionary)
            self.append(output_iu)

    def setup(self):
        t = threading.Thread(target=self.run_detector)
        t.start()
