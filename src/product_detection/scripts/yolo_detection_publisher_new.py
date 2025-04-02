#!/usr/bin/env python3.10

import numpy as np
from sensor_msgs.msg import Image
from pathlib import Path
from product_detection.msg import BoundingBox, BoundingBoxArray
# import pyrealsense2 as rs
import rospy
from ultralytics import YOLO
import yaml

color_image = Image()

def load_class_names():
    config_path = Path(__file__).parent.parent / "config/yolo_classes.yaml"
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['class_names']

def camera_color_image_callback(msg):
    """Callback function for the camera color image topic."""
    global color_image
    color_image = msg

def publish_yolo_detections():
    rospy.init_node("yolo_detection_publisher")
    boundingbox_publisher = rospy.Publisher("/yolo_detection_boundingbox", BoundingBoxArray, queue_size=10)
    rospy.Subscriber("/camera/color/image_raw", Image, camera_color_image_callback)

    # Load model and annotators
    model_path = Path(__file__).parent.parent / "weights/best_v2.pt"
    model = YOLO(model_path)

    class_name = load_class_names()
    rospy.loginfo(f"Loaded class names: {class_name}")

    while not rospy.is_shutdown():
        if not color_image.data:
            rospy.logwarn("No data received for color image. Skipping this frame.")
            continue

        try:
            np_arr = np.frombuffer(color_image.data, dtype=np.uint8).reshape(color_image.height, color_image.width, -1)
            results = model.predict(source=np_arr, conf=0.7, show=True, save=False, verbose=False)[0]
            # results = model(np_arr, verbose=False, conf=0.75)[0]
        except ValueError as e:
            rospy.logerr(f"Error processing color image: {e}")
            continue

        detections = BoundingBoxArray()
        for result in results.boxes:
            detection = BoundingBox()
            detection.label = class_name[int(result.cls[0].item())]
            detection.confidence = result.conf[0].item()
            detection.bbox = result.xyxy[0].tolist()
            detections.boxes.append(detection)
        boundingbox_publisher.publish(detections)
        rospy.loginfo(f"Find {len(results.boxes)} detected objects")

        rospy.Rate(30).sleep()

if __name__ == "__main__":
    try:
        publish_yolo_detections()
    except rospy.ROSInterruptException:
        pass
