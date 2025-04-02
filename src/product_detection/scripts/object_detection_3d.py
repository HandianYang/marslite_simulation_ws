#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
from product_detection.msg import BoundingBox, BoundingBoxArray, DetectedObject, DetectedObjectArray

class Object3DDetector:
    def __init__(self):
        rospy.init_node("object_detection_3d")

        # Subscribers
        rospy.Subscriber("/yolo_detection_boundingbox", BoundingBoxArray, self.yolo_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        # Publishers
        self.obj_pub = rospy.Publisher("/detected_objects_3d", DetectedObject, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)

        # TF buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None

    def camera_info_callback(self, msg):
        """Stores camera intrinsic parameters"""
        self.camera_info = msg

    def depth_callback(self, msg):
        """Stores latest depth image"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")  # Depth in mm

    def yolo_callback(self, msg):
        """Processes YOLO detections and retrieves 3D positions"""
        if self.depth_image is None or self.camera_info is None:
            return  # Wait until we have depth and camera info

        fx = self.camera_info.K[0]  # Focal length in x
        fy = self.camera_info.K[4]  # Focal length in y
        cx = self.camera_info.K[2]  # Optical center x
        cy = self.camera_info.K[5]  # Optical center y

        detected_objects_list = []

        for box in msg.boxes:
            x_min, y_min, x_max, y_max = box.bbox
            u = int((x_min + x_max) / 2)
            v = int((y_min + y_max) / 2)

            depth = self.depth_image[v, u] / 1000.0  # Convert to meters

            if depth == 0 or np.isnan(depth):
                rospy.logwarn(f"Skipping {box.label}: Invalid depth")
                continue

            # Project to 3D
            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth

            # Convert to PointStamped
            point_camera = PointStamped()
            point_camera.header.frame_id = "camera_link"
            point_camera.point.x = X
            point_camera.point.y = Y
            point_camera.point.z = Z

            # Transform to map frame
            try:
                transform = self.tf_buffer.lookup_transform("odom", "camera_link", rospy.Time(0), rospy.Duration(1.0))
                point_map = tf2_geometry_msgs.do_transform_point(point_camera, transform)

                # Store the detected object
                detected_obj = DetectedObject()
                detected_obj.label = box.label
                detected_obj.position = point_map.point
                detected_obj.confidence = box.confidence
                detected_objects_list.append(detected_obj)

            except Exception as e:
                rospy.logwarn(f"TF Transform Error: {e}")
        
        # Publish all objects in a single message
        if detected_objects_list:
            obj_array_msg = DetectedObjectArray()
            obj_array_msg.objects = detected_objects_list
            self.obj_pub.publish(obj_array_msg)
            rospy.loginfo(f"Published {len(detected_objects_list)} objects")

    def publish_detected_object(self, detection, point):
        """Publishes detected object as 3D message"""
        obj_msg = DetectedObject()
        obj_msg.label = detection.label
        obj_msg.position = point.point
        obj_msg.confidence = detection.confidence
        self.obj_pub.publish(obj_msg)
        rospy.loginfo(f"Published {detection.label} at {point.point.x}, {point.point.y}, {point.point.z}")

    def publish_marker(self, detection, point):
        """Publishes visualization marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = (1.0, 0.0, 0.0)  # Red
        marker.id = hash(detection.label) % 1000
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    try:
        Object3DDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
