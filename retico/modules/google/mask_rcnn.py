import tensorflow as tf
import sys
import os
sys.path.append(os.environ['TF_RESEARCH'])
sys.path.append(os.environ['TF_SLIM'])
from object_detection.utils import ops as utils_ops
import numpy as np

class MaskRCNN():
    """ A class that provides a simple interface to processing images
    with a tensorflow MaskRCNN implementation.
    """

    def __init__(self, model_file, image_dims):
        """Initializes a tensorflow mask rcnn model from a frozen graph file.
        The image_dimensions are needed to automatically scale mask rcnn instance
        mask images to the scale of the input images. 
        
        This is needed becuase in default mask rcnn implementations the masks are only
        the size of the bounding boxes they are within.
        
        Arguments:
            model_file {str} -- file path to a frozen mask rcnn graph file (download from tensorflow model zoo)
            image_dims {tuple} -- tuple of image dimensions in format (height, width, channels)
        """
        self.image_dims = image_dims
        self.detection_graph = load_model_from_file(model_file)
        self.output_names = ['num_detections', 'detection_boxes', 'detection_scores',
                           'detection_classes', 'detection_masks']
        self.tensor_dict = get_output_tensors_by_name(self.output_names, self.detection_graph)
        if 'detection_masks' in self.tensor_dict:
            reframe_box_mask_to_image_mask(self.tensor_dict, self.image_dims)   
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        
        # allow cudnn to initialize w/memory issues
        config = tf.ConfigProto()
        config.gpu_options.allow_growth=True
        self.sess = tf.compat.v1.Session(graph=self.detection_graph, config=config)
    
    def detect(self, image):
        """Processes image with Mask RCNN to generate instance masks, object bounding box
        detections, class predictions for each bounding box detected, and confidence scores
        for each class prediction.
        
        Arguments:
            image {np.array} -- The image that will be processed by mask rcnn.
        
        Raises:
            ValueError: if image does not have the same shape as this instance was constructed with (constructor parameter image_shape)
        
        Returns:
            dict -- With 4 keys 
                        'detection_boxes' -- Predicted bounding box for each object detected.
                                            Formated as np.array of lists all of length 4.  
                                            Bounding box is in format [YMin, YMax, XMin, XMax]
                                            where all values are normalized to values between
                                            0 and 1 according to the image size.
                                            (Always outputs fixed length np.array 
                                                regardless of number of objects detected.
                                                Extra spots in array are given box of [0,0,0,0])
                        'detection_classes' -- Predicted class ID of each bounding box
                                                and mask.
                                                (Always outputs fixed length np.array 
                                                regardless of number of objects detected.
                                                Extra spots in array are given class of 1)
                        'detection_masks' -- Binary masks for each bounding box detected
                                            masks are size of the input image. Masks are
                                            only generated for objects were detected, so
                                            the length of this np.array is equal to the
                                            number of detected objects.
                        'detection_scores' -- Confidence scores, value between 0 and 1. 
                                                (Always outputs fixed length np.array 
                                                regardless of number of objects detected.
                                                Extra spots in array are given score of 0)
                        'num_detections' -- Number of objects detected
        """
        if image.shape != self.image_dims:
            raise ValueError('image dimensions must be the same as the dimensions passed when constructing the model.')
        image_expanded = np.expand_dims(image, axis=0)
        output_dict = run_inference_for_single_image(self.sess, self.tensor_dict, self.image_tensor, image_expanded)
        return output_dict

def load_model_from_file(file_path):
    """Loads a frozen tensorflow graph from the given file.
    
    Arguments:
        file_path {str} -- File to load frozen model from
    
    Returns:
        tensorflow.Graph -- Frozen graph containing model loaded from file path
    """
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.io.gfile.GFile(file_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
    return detection_graph

def get_output_tensors_by_name(names, graph):
    """Creates a dictionary of output tensors for a given graph
    where the keys to the dictionary are the string names for each
    output tensor.
    
    Arguments:
        names {list} -- list of string names to match to output tensor names
        graph {tf.Graph} -- tensorflow graph with output tensors
    
    Returns:
        [dict] -- output tensor dictionary with mapping from name to output tensor.
    """
    tensor_dict = {}
    ops = graph.get_operations()
    all_tensor_names = {output.name for op in ops for output in op.outputs}
    for key in names:
        tensor_name = key + ':0'
        if tensor_name in all_tensor_names:
            tensor_dict[key] = graph.get_tensor_by_name(tensor_name)
    return tensor_dict

def reframe_box_mask_to_image_mask(tensor_dict, image_dims):
    """Takes in a tensor dictionary with keys 'detection_boxes' and
    'detection_masks' and adds tensor operations to scale the detection
    masks to the size of image_dims, rather than the size of the detection
    box for the mask.
    
    Arguments:
        tensor_dict {[dict]} -- dictionary of output tensors from mask rcnn
        image_dims {tuple} -- dimension of images that this instance of maskrcnn
                                should process. (height, width, channels)
    """
    # The following processing is only for single image
    detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
    detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])

    # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
    real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
    detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
    detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
    detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
        detection_masks, detection_boxes, image_dims[0], image_dims[1])
    detection_masks_reframed = tf.cast(tf.greater(detection_masks_reframed, 0.5), tf.uint8)

    # Follow the convention by adding back the batch dimension
    tensor_dict['detection_masks'] = tf.expand_dims(detection_masks_reframed, 0)

def run_inference_for_single_image(sess, tensor_dict, image_tensor, image):
    """ Runs an image through a tensorflow mask rcnn model graph, and returns
    the output of mask rcnn in a dictionary
    
    Arguments:
        sess {tf.Session} -- Tensorflow session where the graph has been set to a mask rcnn graph
        tensor_dict {dict} -- a dicionary mapping strings to the output tensors of the mask rcnn graph
        image_tensor {tf.Tensor} -- the input image tensor for mask rcnn
        image {np.array} -- the image to process with mask rcnn
    
    Returns:
        [dict] -- mask rcnn output
    """
    output_dict = sess.run(tensor_dict, feed_dict={image_tensor: image})
    # all outputs are float32 numpy arrays, so convert types as appropriate
    output_dict['num_detections'] = int(output_dict['num_detections'][0])
    output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.int64)
    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
    output_dict['detection_scores'] = output_dict['detection_scores'][0]
    if 'detection_masks' in output_dict:
        output_dict['detection_masks'] = output_dict['detection_masks'][0]
    return output_dict
