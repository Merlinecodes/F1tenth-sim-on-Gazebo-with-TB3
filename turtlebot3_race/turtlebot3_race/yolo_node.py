import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
#from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

import numpy as np
import math
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import torch
import json
#import time
from filterpy.kalman import KalmanFilter
from ultralytics import YOLO
from pathlib import Path
#classNames = ["Human Obstacle", "Animal Obstacle", "Obstacle"]

class YoloNode(Node):
    def _init_(self):
        super()._init_('yolo_node')

        self.yolo_image_pub = self.create_publisher(Image, '/yolo/detections/image', 10)
        self.yolo_object_info_pub = self.create_publisher(String, '/yolo/detections/object_info', 10)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info("CUDA available: " + str(torch.cuda.is_available()))
        self.get_logger().info("Using device: " + self.device)

        self.model = YOLO("/home/merline/ws_camera/src/obstacle_detection/best.pt")
        self.model.to(self.device)
        self.bridge = CvBridge()

        self.pred_image_msg = Image()
        self.color = (0,0,255)

        #self.Kalman_Filter_Init()
        self.KF = KalmanFilter(dim_x=4, dim_z=2)
        self.KF.transition_matrices = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        self.KF.measurement_matrices = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
        self.KF.process_noise_covariance = 0.03 * np.eye(4, dtype=np.float32)
        self.KF.measurement_noise_covariance = 1e-1 * np.eye(2, dtype=np.float32)

        self.detected_objects = None
        self.object_info = None
        self.object_info_str = None

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.frame_count = 0

        self.get_logger().info('Initialisation done')

    def process_frames(self):
        
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        #self.get_logger().info('RGB_callback is working')

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        results = self.model(color_image, conf=0.75)

        if len(results[0].boxes) == 0:
            self.get_logger().info("No detections.")
            return color_image

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            class_id = int(box.cls[0])
            confidence = box.conf[0].cpu().numpy()
            class_name = self.model.names[class_id]

            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            self.KF.predict()
            self.KF.update(np.array([cx, cy], dtype=np.float32))
            pred_cx, pred_cy = self.KF.x[:2]

            depth_width = depth_frame.get_width()
            depth_height = depth_frame.get_height()
            cx = max(0, min(cx, depth_width - 1))
            cy = max(0, min(cy, depth_height - 1))
            depth_value = depth_frame.get_distance(cx, cy)

            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f"Depth: {depth_value:.2f}m", (cx, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(color_image, f"{class_name}: {confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            detection_info = {
                "Detected class": class_name,
                "Confidence": f"{confidence:.2f}",
                "Centroid": f"({cx}, {cy})",
                "Depth": f"{depth_value}m"
            }

            # Convert dictionary to a JSON string
            self.object_info_str = json.dumps(detection_info)
            #self.get_logger().info(self.object_info_str)

            object_info = String()
            object_info.data = self.object_info_str
            self.yolo_object_info_pub.publish(object_info)

        return color_image
    
    def detection_init(self):

        self.get_logger().info("Press 'q' to quit.")
        try:
            while rclpy.ok():
                self.frame_count += 1
                if self.frame_count%50 == 1:
                    annotated_image = self.process_frames()
                    image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                    self.yolo_image_pub.publish(image_msg)
                    cv2.imshow("YOLOv8 Object Detection with Depth", annotated_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main():
    rclpy.init()
    print("ROS2 Initialized") 
    yolo_node = YoloNode()
    yolo_node.detection_init()
    
if _name=='main_':
    main(
