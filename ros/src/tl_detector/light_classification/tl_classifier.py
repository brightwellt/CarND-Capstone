from styx_msgs.msg import TrafficLight
import rospy

import numpy as np
import os
import sys
import tensorflow as tf
import cv2

from PIL import Image
import visualization_utils as vis_util

print(tf.__version__ )

def load_image_into_numpy_array(image):
	return np.asarray(image, dtype="uint8")  
def adjust_gamma(image, gamma=2):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

class TLClassifier(object):
    def __init__(self):        
        MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        self.path_to_ckpt= MODEL_NAME + '/frozen_inference_graph.pb'

        self.index_traffic=10
        self.num_classes = 90
        self.detection_graph = tf.Graph()
        self.traffic_light_color=4
        
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

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_np = load_image_into_numpy_array(image)
        image_np_adj = adjust_gamma(image_np, gamma=2)

        # Expand dimensions since the tensorflow model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np_adj, axis=0)
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
              [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
              feed_dict={self.image_tensor: image_np_expanded})
        
        # In the model data, traffic lights have the index number 10
        category_index={ 10: {'id': 10, 'name': 'traffic light'}}
    
        height, width, _ = image_np.shape  
        classes = np.squeeze(classes).astype(np.int32)
        boxes=np.squeeze(boxes)
        scores=np.squeeze(scores)
        confidence = 0
        box_conf = []  
        classes_10=[]
        box_area=0
        for i in range(len(classes)):
            if classes[i] in category_index.keys():
                if scores[i] > confidence:
                    confidence = scores[i]
                    box_conf = boxes[i]
                    classes_10=classes[i]
                    if len(box_conf) != 0 :
                    	ymin, xmin, ymax, xmax = box_conf
                    	(left, right, top, bottom) = (xmin * width, xmax * width,
                                                 ymin * height, ymax * height)   
                    	box_area=(int(bottom)-int(top))*(int(right)-int(left))
                    else:
                    	box_area=0	

        if not ((box_area<800) or (confidence<0.3)):

            rospy.loginfo("TL: box are  %s confidence is %s",box_area, confidence)                         
        
            img_light = image_np[int(top):int(bottom), int(left):int(right)]


            if 1:
                traffic_light = cv2.resize(img_light, (64,128))       	

                tl_hsv = cv2.cvtColor(traffic_light, cv2.COLOR_BGR2HSV)

                red_low = np.array([0,50,50])
                red_up = np.array([10,255,255])
                red1 = cv2.inRange(tl_hsv, red_low , red_up)

                red_low = np.array([170,50,50])
                red_up = np.array([180,255,255])
                red2 = cv2.inRange(tl_hsv, red_low , red_up)
		
		yel_low = np.array([20,100,100])
                yel_up = np.array([40,255,255])
                yel = cv2.inRange(tl_hsv, yel_low , yel_up)

                weighted_img = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)
                blur = cv2.GaussianBlur(weighted_img,(15,15),0)
		blur_yel = cv2.GaussianBlur(yel,(15,15),0)

                circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,0.5,40, param1=70,param2=20,minRadius=2,maxRadius=20)
		circles_yel = cv2.HoughCircles(blur_yel,cv2.HOUGH_GRADIENT,0.5,40, param1=70,param2=20,minRadius=2,maxRadius=20)

                if circles is None:
		    if circles_yel is None:
			rospy.loginfo("TL: traffic light Green")
                        self.traffic_light_color=TrafficLight.GREEN
		    else:
			rospy.loginfo("TL: traffic light Yellow")
                        self.traffic_light_color=TrafficLight.YELLOW
                else:
                    rospy.loginfo("TL: traffic light RED")
                    self.traffic_light_color=TrafficLight.RED

                category_color={ 10: {'id': 10, 'name': self.traffic_light_color}} 
                box_show=[]
                box_show.append(box_conf)
                conf_show=[]
                conf_show.append(confidence)
                classes_show=[]
                classes_show.append(classes_10)
                box_to_color_map =vis_util.visualize_boxes_and_labels_on_image_array(
                                  image_np,
                                  np.array(box_show),
                                  np.array(classes_show),
                                  np.array(conf_show),
                                  category_color,
                                  use_normalized_coordinates=True,
                                  min_score_thresh=.3,
                                  line_thickness=2)    
        return (self.traffic_light_color, image_np)
        
