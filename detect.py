# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run the object detection routine."""
import argparse
import sys
import time

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision

import numpy as np

#Import xml for save xml
from xml.etree.ElementTree import Element, SubElement, ElementTree


_MARGIN = 10  # pixels
_ROW_SIZE = 10  # pixels
_FONT_SIZE = 1
_FONT_THICKNESS = 1
_TEXT_COLOR = (0, 0, 255)  # red

# Import packages for ros #Added##########
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#draw rect, object name, possibility #added comment
def visualize(
    image: np.ndarray,
    detection_result: processor.DetectionResult,
) -> np.ndarray:
  """Draws bounding boxes on the input image and return it.

  Args:
    image: The input RGB image.
    detection_result: The list of all "Detection" entities to be visualize.

  Returns:
    Image with bounding boxes.
  """

  #xml elements
  nameOfFile = str(time.time())
  #root
  annotation = Element("annotation")

  #depth1 basics
  folder = Element("folder")
  filename = Element("filename")
  path = Element("path")
  source = Element("source")
  size = Element("size")
  segmented = Element("segmented")
  annotation.append(folder)
  annotation.append(filename)
  annotation.append(path)
  annotation.append(source)
  annotation.append(size)
  annotation.append(segmented)

  folder.text = '1'
  filename.text = nameOfFile + '.jpg'
  path.text = '~/realtime_images/' + filename.text

  database = SubElement(source, "database")
  database.text = 'Unknown'

  width = SubElement(size, "width")
  width.text = '640'
  height = SubElement(size, "height")
  height.text = '480'
  depth = SubElement(size, "depth")
  depth.text = '3'

  segmented.text = '0'

  for detection in detection_result.detections:
    # Draw bounding_box
    bbox = detection.bounding_box
    start_point = bbox.origin_x, bbox.origin_y
    end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height

    cv2.rectangle(image, start_point, end_point, _TEXT_COLOR, 3)

    # Draw label and score
    category = detection.classes[0]
    class_name = category.class_name
    probability = round(category.score, 2)
    result_text = class_name + ' (' + str(probability) + ')'
    text_location = (_MARGIN + bbox.origin_x,
                     _MARGIN + _ROW_SIZE + bbox.origin_y)
    cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)

    
    #object add for each objects
    object = Element("object")
    annotation.append(object)

    name = SubElement(object, "name")
    name.text = class_name
    pose = SubElement(object, "pose")
    pose.text = 'Unspecified'
    truncated = SubElement(object, "truncated")
    truncated.text = '0'
    difficult = SubElement(object, "difficult")
    difficult.text = '0'

    bndbox = SubElement(object, "bndbox")
    xmin = SubElement(bndbox, "xmin")
    xmin.text = str(start_point[0])
    ymin = SubElement(bndbox, "ymin")
    ymin.text = str(start_point[1])
    xmax = SubElement(bndbox, "xmax")
    xmax.text = str(end_point[0])
    ymax = SubElement(bndbox, "ymax")
    ymax.text = str(end_point[1])

    # Node for publish object_name #Added##########
    rospy.init_node("tflite_node", anonymous=True)
    pub = rospy.Publisher("tflite_topic", String, queue_size=1)
    msg = String()
    msg.data = class_name + str(probability)
    pub.publish(msg)

  cv2.imwrite('/home/pi/raspberry_pi/realtime_images/'+ nameOfFile +'.jpg', image)
  tree = ElementTree(annotation)

  fileName = "/home/pi/raspberry_pi/realtime_images/" + nameOfFile + ".xml"
  with open(fileName, "wb") as file:
    tree.write(file, encoding='utf-8', xml_declaration=True)

  return image

def run(model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
  """

  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  cap = cv2.VideoCapture(camera_id)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.3)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)

  # Continuously capture images from the camera and run inference
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      sys.exit(
          'ERROR: Unable to read from webcam. Please verify your webcam settings.'
      )

    counter += 1
    image = cv2.flip(image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)

    # Draw keypoints and edges on input image
    image = visualize(image, detection_result)

    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    # Stop the program if the ESC key is pressed.
    if cv2.waitKey(1) == 27:
      break
    cv2.imshow('object_detector', image)

    # Node for publish Image #Added##########
    rospy.init_node("tflite_node", anonymous=True)
    pub_img = rospy.Publisher("tflite_image", Image, queue_size=1)
    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(image,"bgr8")
    pub_img.publish(img_msg)

  cap.release()
  cv2.destroyAllWindows()

#parsing args and run run() #added comment
def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default='efficientdet_lite0.tflite')
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  parser.add_argument(
      '--enableEdgeTPU',
      help='Whether to run the model on EdgeTPU.',
      action='store_true',
      required=False,
      default=False)
  args = parser.parse_args()

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
      int(args.numThreads), bool(args.enableEdgeTPU))

#run main
if __name__ == '__main__':
  main()
