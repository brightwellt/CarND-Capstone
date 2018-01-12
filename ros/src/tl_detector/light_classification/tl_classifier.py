from styx_msgs.msg import TrafficLight
import rospy

import numpy as np
import os
import sys
import tensorflow as tf


from PIL import Image
import visualization_utils as vis_util

print(tf.__version__ )



def load_image_into_numpy_array(image):
    return np.asarray(image, dtype="uint8")
  #(im_width, im_height) = image.sizenp.reshape(boxes_10,(-1,4))
  #return np.array(image.getdata()).reshape(
      #(im_height, im_width, 3)).astype(np.uint8)


class TLClassifier(object):
    def __init__(self):
        MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        self.path_to_ckpt= MODEL_NAME + '/frozen_inference_graph.pb'

        self.index_traffic=10
        self.num_classes = 90
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                self.sess=sess
                # Definite input and output Tensors for detection_graph
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                
    def get_traffic_light_image(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_np = load_image_into_numpy_array(image)
        image_np_expanded = np.expand_dims(image_np, axis=0)
        (boxes, scores, classes, num) = self.sess.run(
          [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
          feed_dict={self.image_tensor: image_np_expanded})
        category_index={ 10: {'id': 10, 'name': 'traffic light'}}
        #rospy.loginfo("Upcoming box class %s",  classes)
        classes=np.squeeze(classes).astype(np.int32)
        boxes=np.squeeze(boxes)
        scores=np.squeeze(scores)
        boxes_10=[]
        scores_10=[]
        classes_10=[]
        for i in range(len(classes)):
            if classes[i] in category_index.keys():
                boxes_10.append(boxes[i])
                scores_10.append(scores[i])
                classes_10.append(classes[i])
               
        box_to_color_map =vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.reshape(boxes_10,(-1,4)),
          np.array(classes_10),
          np.array(scores_10),
          category_index,
          use_normalized_coordinates=True,
          line_thickness=8)
        #for box, color in box_to_color_map[1].items():
        #    rospy.loginfo("Upcoming BOX %s",  box)

        return image_np    
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #rospy.loginfo("Upcoming get_classification")
        #print("get_classification")
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
