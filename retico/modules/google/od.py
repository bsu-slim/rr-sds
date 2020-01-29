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

    def __init__(self,labels_path, model_path, image_dims=(240, 320, 3), **kwargs):
        """Initializes the object detector module

        Args:

        """
        super().__init__(**kwargs)
        # matplotlib.use('TkAgg') 
        # print(matplotlib.get_backend())
        self.category_index = label_map_util.create_category_index_from_labelmap(labels_path, use_display_name=True)
        self.model = MaskRCNN(model_path, image_dims)
        self.queue = deque()

    def process_iu(self, input_iu):
        self.queue.clear() # drop frames, if still waiting
        self.queue.append(input_iu)

        return None

    def show_image(self, image_np, output_dict):
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            output_dict['detection_boxes'][:10],
            output_dict['detection_classes'][:10],
            output_dict['detection_scores'][:10],
            self.category_index,
            use_normalized_coordinates=True,
            min_score_thresh=0.01,
            line_thickness=8,
            max_boxes_to_draw=3)
        plt.imshow(image_np)
        plt.pause(0.001)
        plt.clf() 
        plt.show()

    def run_detector(self):

        while True:
            if len(self.queue) == 0:
                time.sleep(0.1)
                continue
            
            input_iu = self.queue.popleft()
            image = input_iu.payload
            output_dict = self.model.detect(image)
            boxes = output_dict['detection_boxes'][:1]

            returning_dictionary = {}
            for ind,obj in enumerate(boxes):
                inner_dict = {}
                inner_dict['y1'] = obj[0]
                inner_dict['x1'] = obj[1]
                inner_dict['y2'] = obj[2]
                inner_dict['x2'] = obj[3]
                inner_dict['label'] = output_dict['detection_classes'][ind]
                returning_dictionary["object"+str(ind)] = inner_dict
            returning_dictionary['num_objs'] = len(returning_dictionary)
            output_iu = self.create_iu(input_iu)
            output_iu.set_detected_objects(image, returning_dictionary)
            self.append(output_iu)
            # self.show_image(image, output_dict)
            

    def setup(self):
        t = threading.Thread(target=self.run_detector)
        t.start()
