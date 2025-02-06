#!/usr/bin/env python3.10
import rospy
import pyrealsense2 as rs
import supervision as sv
import numpy as np
import cv2
from pathlib import Path
from ultralytics import YOLO

def main():
    rospy.init_node('object_detection_viewer', anonymous=True)

    # Load model and annotators
    model_path = Path(__file__).parent.parent / "weights/best_v2.pt"
    model = YOLO(model_path)
    box_annotator = sv.BoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    # Get device product line for setting a supporting resolution
    pipeline = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image, verbose=False, conf=0.75)[0]
            detections = sv.Detections.from_ultralytics(results)
            color_image = box_annotator.annotate(color_image, detections=detections)
            color_image = label_annotator.annotate(color_image, detections=detections)

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rospy.Rate(25).sleep()

    finally:
        # Stop streaming
        pipeline.stop()
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        