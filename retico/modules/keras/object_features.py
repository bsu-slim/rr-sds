"""A module for object feature extraction provided by azure"""

# retico
from retico.core import abstract
from retico.core.visual.common import DetectedObjectsIU
from retico.core.visual.common import ObjectFeaturesIU

# keras
from keras.preprocessing import image
from keras.models import Model
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model
import tensorflow as tf



# other
import numpy as np
from PIL import Image as PImage


class KerasObjectFeatureExtractorModule(abstract.AbstractModule):
    """An object feature extraction module using Keras.

    Attributes:
    """

    @staticmethod
    def name():
        return "Keras Object Feature Extractor"

    @staticmethod
    def description():
        return "A feature extractor that takes in an image and produces a vector"

    @staticmethod
    def input_ius():
        return [DetectedObjectsIU]

    @staticmethod
    def output_iu():
        return ObjectFeaturesIU

    def __init__(self, model_type='efficientnet', layer='probs', weights='imagenet', **kwargs):
        """Initializes the object feature detector module

        Args:

        """
        super().__init__(**kwargs)
        self.weights = weights
        self.layer = layer
        self.session = tf.Session(graph=tf.Graph())
        self.pre_input = None
        with self.session.graph.as_default():
            set_session(self.session)

            if model_type == 'efficientnet':
                from efficientnet.keras import EfficientNetB0
                from efficientnet.keras import preprocess_input
                self.pre_input = preprocess_input
                base_model = EfficientNetB0(weights=self.weights)
                self.xs,self.ys=224,224

            if model_type == 'inveptionv3':
                from keras.applications.inception_v3 import InceptionV3, preprocess_input
                base_model = InceptionV3(weights=self.weights,include_top=True)
                self.pre_input = preprocess_input
                self.xs,self.ys=299,299
                
            # allow for other output layers
            self.model = Model(inputs=base_model.input, outputs=base_model.get_layer(self.layer).output)
        print('Feature extractor module setup complete.')

    def get_bounded_subimage(self, I, img_box):
        '''
        I (image)
        img_box (dict): which is a dictionary with bounding box info in the format:
            {'x1': 9, 'x2': 36, 'y1': 6, 'y2': 45, 'label': 'person'}

        '''
        sub = I[img_box['x1']:img_box['x2'],img_box['y1']:img_box['y2']]
        
        # this can show the sub-image
        # import matplotlib.pyplot as plt
        # plt.figure()
        # ax = plt.gca()
        # ax.imshow(sub)
        # plt.show()
        
        if len(sub) == 0: return None
        pim = PImage.fromarray(sub)
        pim2 = pim.resize((self.xs,self.ys), PImage.ANTIALIAS)
        #img = np.array(pim2, dtype=np.float32)
        img = image.img_to_array(pim2)
        img = np.expand_dims(img, axis=0)
        #if len(img.shape) < 3: return None
        #img = img.reshape((1, img.shape[0], img.shape[1], img.shape[2]))
        return img

    def get_img_features(self, img):
        img = self.pre_input(img)
        with self.session.graph.as_default():
            set_session(self.session)
            yhat = self.model.predict(img)
            return yhat

    def process_iu(self, input_iu):
        image = input_iu.image
        detected_objects = input_iu.detected_objects
        object_features = {}
        for obj in detected_objects:
            obj_info = detected_objects[obj]
            sub_img = self.get_bounded_subimage(image, obj_info)
            feats = self.get_img_features(sub_img).flatten()
            object_features[obj] = feats.tolist()
        
        output_iu = self.create_iu(input_iu)
        output_iu.set_object_features(image, object_features)
        return output_iu

    def setup(self):
        '''
        for some reason we can't setup the keras model here or it gets stuck
        '''
        pass
