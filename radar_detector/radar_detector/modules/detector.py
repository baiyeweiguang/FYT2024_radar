# Copyright(C) FYT Vision Group. All rights Reserved.


import torch
import torch.nn.functional as F
import numpy as np
import cv2 
import asyncio

from ultralytics import YOLO
from ultralytics.nn.modules import Detect, v10Detect, C2f
from ultralytics.engine.results import Results

from ..config import DetectorConfig

class DetectorResult:
  def __init__(self, category=0, color=0, yolo_score=0, class_score=0, box=None) -> None:
    self.category = category
    self.color = color
    self.yolo_score = yolo_score
    self.class_score = class_score
    self.box = box

class Detector:
  ''' 
  识别器: YOLOv8从图像中识别敌方车辆,利用检测框截取roi,利用分类器对roi进行分类
  '''
  def __init__(self, cfg: DetectorConfig, device: torch.device) -> None:
    self.cfg = cfg
    self.device = device
    self.robot_detector_model = YOLO(cfg.car_model, task='detect')
    self.armor_model = YOLO(cfg.armor_model, task='detect')
    self.tracked = {}
    
    # 因为armor_model要动态输入，不能用onnx模型，所以手动将模型做导出设置
    self.armor_model.fuse()
    for m in self.armor_model.modules():
      if isinstance(m, Detect):  
          m.format = ''
          m.export = True
      elif isinstance(m, C2f):
          m.forward = m.forward_split
      
  def detect(self, aimg: np.ndarray) -> list:
    '''
    识别车辆并分类
    '''    
    img = cv2.cvtColor(aimg, cv2.COLOR_BGR2RGB)
    # 识别车辆
    results : Results = self.robot_detector_model.track(img, persist=True, tracker='bytetrack.yaml')[0]
    
    # results = self.robot_detector_model.track(img, persist=True)[0]

    if len(results.boxes) == 0:
      return []
    
    detector_results = []
    rois = []
    for box in results.boxes:
      x1, y1, x2, y2 = box.xyxy[0]
      
      roi = img[int(y1):int(y2), int(x1):int(x2), :]
      # roi = cv2.resize(roi, (224,224))
      rois.append(roi)

    armors_list  = self.armor_model.predict(rois)
    for i, armors in enumerate(armors_list):
      box = results.boxes[i]
      track_id = int(box.id.item())
      detector_result = DetectorResult()
      detector_result.box = box
      detector_result.yolo_score = float(box.conf)
      # 下面三个是默认值
      detector_result.color = 2
      detector_result.category = 0
      detector_result.class_score = 0.0
      
      if armors.boxes is None or len(armors.boxes) == 0:
        if track_id in self.tracked.keys():
          tracked = self.tracked[track_id]
          detector_result.color = tracked.color
          detector_result.category = tracked.category
          detector_result.class_score = tracked.class_score
      else:
        best_armor = armors.boxes[0]
        for armor in armors.boxes:
          if armor.conf > best_armor.conf:
            best_armor = armor
        category = best_armor.cls.int().item()  

        detector_result.color = int(category >= 6)
        detector_result.category = category % 6
        detector_result.class_score = float(best_armor.conf)
        
      self.tracked[track_id] = detector_result
        
      detector_results.append(detector_result)
        
       
    return detector_results