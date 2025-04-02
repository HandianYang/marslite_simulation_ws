#!/usr/bin/env python3.10

import numpy as np
from pathlib import Path
from product_detection.msg import BoundingBox, BoundingBoxArray
import pyrealsense2 as rs
import rospy
from ultralytics import YOLO
import yaml

def load_class_names():
    config_path = Path(__file__).parent.parent / "config/yolo_classes.yaml"
    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)
    return data['class_names']

def publish_yolo_detections():
    rospy.init_node("yolo_detection_publisher")
    boundingbox_publisher = rospy.Publisher("/yolo_detection_boundingbox", BoundingBoxArray, queue_size=10)

    # Load model and annotators
    model_path = Path(__file__).parent.parent / "weights/best_v2.pt"
    model = YOLO(model_path)

    # Get device product line for setting a supporting resolution
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    class_name = load_class_names()
    rospy.loginfo(f"Loaded class names: {class_name}")

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
                
            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image, verbose=False, conf=0.75)[0]

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

    finally:
        pipeline.stop()

if __name__ == "__main__":
    try:
        publish_yolo_detections()
    except rospy.ROSInterruptException:
        pass
