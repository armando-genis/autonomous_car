#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('~/ros2_ws/src/niagara_vision_processing/yolov8_niagara/scripts/yolov8n.pt')

        self.subscription = self.create_subscription(
            Image,
            '/zed2/image_raw',
            self.camera_callback,
            10)
        self.subscription 

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)
        img_copy = img.copy()
        

        results = self.model(img_copy)

        # Loop through each detection and draw bounding boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                cv2.rectangle(img_copy, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
                cv2.putText(img_copy, class_name, (int(b[0]), int(b[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with bounding boxes
        cv2.imshow("IMAGE", img_copy)
        cv2.waitKey(4)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
