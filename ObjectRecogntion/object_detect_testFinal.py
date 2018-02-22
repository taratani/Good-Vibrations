import numpy as np 
import os
import sys
import time
import tensorflow as tf

from collections import defaultdict
from io import StringIO
from picamera import PiCamera

sys.path.append("..")

from utils import label_map_util

import cv2

# initialize raspberry pi camera
camera = PiCamera()

# path to frozen detection graph
PATH_TO_CKPT = 'ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')

NUM_CLASSES = 90

# load frozen model into memory
t0 = time.time()
detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')
t1 = time.time()
print('Frozen model loaded: ' + str(t1-t0) + ' s')

def detectObstacle(image_np):
    # actual detection
    image_np_expanded = np.expand_dims(image_np, axis=0)
    (boxes, scores, classes, num) = sess.run(
	[detection_boxes, detection_scores, detection_classes, num_detections],
	feed_dict={image_tensor: image_np_expanded})
    objects = []
    threshold = 0.5
    #objects = [category_index.get(value) for index,value in enumerate(classes[0]) if scores[0,index] > 0.5]
    #print(objects['name'])
    for index, value in enumerate(classes[0]):
        object_dict = {}
        if scores[0, index] > threshold:
          	object_dict[(category_index.get(value)).get('name').encode('utf8')] = \
                scores[0, index]
          	objects.append(object_dict.copy())
    return objects


# loading label maps
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

with detection_graph.as_default():
	with tf.Session(graph=detection_graph) as sess:
	    # definite input and output tensors
	    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
	    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
	    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
	    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
	    num_detections = detection_graph.get_tensor_by_name('num_detections:0')
	    image_np = cv2.imread('image.jpg')
	    detectObstacle(image_np)
	    print('OK!')

	    while True:
	    	option = raw_input()
	    	if(option==''):
		    # capture image
		    os.remove('image.jpg')
		    camera.start_preview()
		    time.sleep(2)
		    t0 = time.time()
	 	    camera.capture('image.jpg')
		    t1 = time.time()
	 	    camera.stop_preview()
		    print('Image taken and saved ' + str(t1-t0) + ' s')
		    detect = False
		    while(not detect):
		        if(raw_input()==''):
			    detect = True
		    image_np = cv2.imread('image.jpg')

		    t0 = time.time()
		    objects = detectObstacle(image_np)
		    t1 = time.time()
		    if objects:
		        print('Object detected: ' + str(t1-t0) + ' s')
		    print(objects)
		if(option=='q'):
		    break
