#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from typing import List, Dict
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesis, ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
import random
import torch


from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        
        device = "cpu" # cpu or cuda "cuda:0"
        self.threshold = 0.40
        self.enable = True
        self.device = device
        
        model = '~/ros2_ws/src/autonomous_car/lidar_camara_fusion/yolov8_niagara/scripts/yolov8n.pt'
        self.yolo = YOLO(model)
        self.yolo.fuse()
        self.yolo.to(device)

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10)
        self.subscription 
        
        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        

    def camera_callback(self, msg):

        
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        img_copy = cv_image.copy()
        results = self.yolo.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.threshold,
            device=self.device
        )
        results: Results = results[0].cpu()
        

        # create detections msg
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        
        for b in results.boxes:
            label = self.yolo.names[int(b.cls)]
            score = float(b.conf)

            if score < self.threshold:
                continue

            detection = Detection2D()
            box = b.xywh[0]
            
            detection.bbox.center.position.x = float(box[0]) # detection.bbox.center.position.x
            detection.bbox.center.position.y = float(box[1])
            detection.bbox.size_x = float(box[2])
            detection.bbox.size_y = float(box[3])
            
            hypothesis_with_pose = ObjectHypothesisWithPose()
            obj_hypothesis = ObjectHypothesis()
            obj_hypothesis.class_id = str(label)  # Ensure the label is a string
            obj_hypothesis.score = score
            hypothesis_with_pose.hypothesis = obj_hypothesis
                    
            detection.results.append(hypothesis_with_pose)
            
            # draw boxes for debug
            if label not in self._class_to_color:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                self._class_to_color[label] = (r, g, b)
            color = self._class_to_color[label]
        

            min_pt = (round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0), # round(detection.bbox.center.position.x - detection.bbox.size_x / 2.0
                        round(detection.bbox.center.position.y - detection.bbox.size_y / 2.0))
            max_pt = (round(detection.bbox.center.position.x + detection.bbox.size_x / 2.0),
                        round(detection.bbox.center.position.y + detection.bbox.size_y / 2.0))
            cv2.rectangle(img_copy, min_pt, max_pt, color, 2)

            label = "{} ({}) ({:.3f})".format(label, str(1), score)
            pos = (min_pt[0] + 5, min_pt[1] + 25)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img_copy, label, pos, font,
                        1, color, 1, cv2.LINE_AA)

            # append msg
            detections_msg.detections.append(detection)

        # publish detections and dbg image
        self._pub.publish(detections_msg)


        cv2.imshow('result', cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB))
        cv2.waitKey(10)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
