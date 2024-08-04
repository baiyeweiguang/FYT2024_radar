# Copyright(C) FYT Vision Group. All rights Reserved.

# rclpy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
# others
import os 
import torch
import cv2
import numpy as np
# msg
from sensor_msgs.msg import Image
from radar_interfaces.msg import BoundingBox, Detection, DetectionArray
# project
from radar_detector.config import Config, DetectorConfig, init_params
from radar_detector.modules.detector import Detector, DetectorResult

class DetectorNode(Node):
  '''
  识别器节点
  '''
  def __init__(self) -> None:
    super().__init__("radar_detector_node")
    
    # params
    self.cfg = init_params(self)
    self.get_logger().info("{}".format(self.cfg))
    
    self.detector = Detector(self.cfg.detector_config, self.cfg.device)
    self.cv_bridge = CvBridge()
    
    # self.model_warmup()
   
    self.save_images = False
    self.images_cnt = 0
    
    # pub/sub
    self.detection_pub = self.create_publisher(DetectionArray, "detections", 10)
    self.debug_pub = self.create_publisher(Image, "debug_img", 10) 
    self.image_sub = self.create_subscription(Image, "image_raw", self.image_callback, qos_profile_sensor_data)
    self.get_logger().info("detector_node started")                                     
  
  
  def model_warmup(self) -> None:
    dummy_input = torch.randn(1, 3, 640, 640).to(self.cfg.device)
    self.detector.detect(dummy_input)  
    
  def image_callback(self, msg: Image) -> None:
    '''
    图像回调
    '''    
    start_time = self.get_clock().now()
    ori_img = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
    scaled_img = ori_img
    
    if ori_img.shape[0] > 1024:
      scaled_img = cv2.resize(src=ori_img, dsize=None, fx=0.5, fy=0.5)
    
    img_width = scaled_img.shape[1]
    img_height = scaled_img.shape[0]

    results : list[DetectorResult] = self.detector.detect(ori_img=ori_img, scaled_img=scaled_img)
    
    detection_array_msg = DetectionArray()
    detection_array_msg.header = msg.header
    for result in results:
      
      color : int = result.color
      color_name = self.cfg.color_names[color]
      category :int = result.category
      yolo_score: float = result.yolo_score
      class_score: float = result.class_score
      box = result.box
      xyxy = box.xyxy[0].tolist()
      if xyxy[0] <= 0 or xyxy[0] >= img_width - 1 or xyxy[2] <= 0 or xyxy[2] >= img_width - 1 or xyxy[1] <= 0 or xyxy[1] >= img_height - 1 or xyxy[3] <= 0 or xyxy[3] >= img_height - 1:
        continue
      
      detection_msg = Detection()
      detection_msg.class_id = self.cfg.category_to_id[category]
      detection_msg.color = 0 if color == 1 else 1
      detection_msg.obj_score = yolo_score
      detection_msg.class_score = class_score
      detection_msg.class_name = self.cfg.color_names[color] + self.cfg.class_names[category]
      detection_msg.bbox.top_left.x = xyxy[0]
      detection_msg.bbox.top_left.y = xyxy[1]
      detection_msg.bbox.bottom_right.x = xyxy[2]
      detection_msg.bbox.bottom_right.y = xyxy[3]
      
      if color_name in ["Red", "Blue"]:
        detection_array_msg.detections.append(detection_msg)
      
      if self.cfg.debug:
        if color_name == "Red":
          line_color = (255, 0, 0)
        elif color_name == "Blue":
          line_color = (0, 0, 255)
        else:
          line_color = (255, 255, 255)
        cv2.rectangle(scaled_img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), line_color, 2)
        cv2.putText(scaled_img, "{}, {:.2f}".format(detection_msg.class_name, detection_msg.class_score), (int(xyxy[0]), int(xyxy[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, line_color, 2)
    
    self.detection_pub.publish(detection_array_msg)
    end_time = self.get_clock().now()
    if self.cfg.debug:
      latency = end_time - start_time
      cv2.putText(scaled_img, "latency: {:.2f}ms".format(latency.nanoseconds / 1e6), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
      debug_msg = self.cv_bridge.cv2_to_imgmsg(scaled_img, "rgb8")
      self.debug_pub.publish(debug_msg)
    
def main() -> None:
  '''
  主函数, ros2 run的入口
  '''
  rclpy.init()
  node = DetectorNode()
  rclpy.spin(node)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()