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
from matplotlib import pyplot as plt
from PIL import Image as PImage
import cv2
import time


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

    def __init__(self, model_type='vgg19', layer='fc2', weights='imagenet', img_dims=(320,240), **kwargs):
        """Initializes the object feature detector module
        efficientnet/probs
        Args:

        """
        super().__init__(**kwargs)
        self.weights = weights
        self.layer = layer
        self.session = tf.Session(graph=tf.Graph())
        self.pre_input = None
        self.img_dims = img_dims
        with self.session.graph.as_default():
            set_session(self.session)

            if model_type == 'efficientnet':
                from efficientnet.keras import EfficientNetB0
                from efficientnet.keras import preprocess_input
                self.pre_input = preprocess_input
                base_model = EfficientNetB0(weights=self.weights)
                self.xs,self.ys=224,224

            if model_type == 'inveptionv3':
                from keras.applications.inception_v3 import InceptionV3
                from keras.applications.inception_v3 import preprocess_input
                base_model = InceptionV3(weights=self.weights,include_top=True)
                self.pre_input = preprocess_input
                self.xs,self.ys=299,299

            if model_type == 'vgg19':
                from keras.applications.vgg19 import VGG19
                from keras.applications.vgg19 import preprocess_input
                base_model = VGG19(weights=self.weights,include_top=True)
                self.pre_input = preprocess_input
                self.xs,self.ys=224,224
                
            # allow for other output layers
            print(base_model.summary())
            self.model = Model(inputs=base_model.input, outputs=base_model.get_layer(self.layer).output)
        print('Feature extractor module setup complete.')

    # def load_image_into_numpy_array(self, image):
    #     (im_width, im_height) = image.size
    #     return np.array(image.getdata()).reshape(
    #         (im_height, im_width, 3)).astype(np.uint8)

    def get_bounded_subimage(self, I, img_box):
        '''
        '''
        xmin = int(img_box['xmin']*self.img_dims[0])
        xmax = int(img_box['xmax']*self.img_dims[0])
        ymin = int(img_box['ymin']*self.img_dims[1])
        ymax = int(img_box['ymax']*self.img_dims[1])
        sub = I.crop([xmin,ymin,xmax,ymax])
        sub.load()
        sub = sub.resize((self.xs,self.ys), PImage.ANTIALIAS)

        print("FOUND OBJECT")
        img_to_show = cv2.cvtColor(np.asarray(sub), cv2.COLOR_BGR2RGB) 
        cv2.imshow('image',img_to_show) 
        cv2.waitKey(1)

        # sub.show()
        img = image.img_to_array(sub)
        img = np.expand_dims(img, axis=0)
        return img

    def get_img_features(self, img):
        img = self.pre_input(img)
        with self.session.graph.as_default():
            set_session(self.session)
            yhat = self.model.predict(img)
            return yhat

    def process_iu(self, input_iu):
        # if(np.random.rand() < 0.5): # simply ignores half of the input ius for speed
        #     return self.create_iu(input_iu)
        start = time.time()
        image = input_iu.image
        detected_objects = input_iu.detected_objects
        object_features = {}
        for obj in detected_objects:
            if 'object' not in obj: continue # objects are prefixed by 'object'
            obj_info = detected_objects[obj]
            sub_img = self.get_bounded_subimage(image, obj_info)
            feats = self.get_img_features(sub_img).flatten() # TODO UNHACK
            object_features[obj] = feats.tolist()
        
        output_iu = self.create_iu(input_iu)
        output_iu.set_object_features(image, object_features)
        print("OBJECT FEATURES PROCESSING TOOK ", time.time() - start, "SECONDS")
        return output_iu

    def setup(self):
        '''
        for some reason we can't setup the keras model here or it gets stuck
        '''
        pass
