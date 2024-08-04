# Copyright(C) FYT Vision Group. All rights Reserved.


import torch
import torch.nn.functional as F
import numpy as np
import cv2 
import asyncio

from ultralytics import YOLO
from ultralytics.nn.modules import Detect, v10Detect, C2f
from ultralytics.engine.results import Results
from ultralytics.data.augment import LetterBox
from ultralytics.models.yolo.detect import DetectionPredictor
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
    
    if cfg.car_model.split('.')[-1] == 'pt':
      self.fuse_model(self.robot_detector_model)
    if cfg.armor_model.split('.')[-1] == 'pt':
      self.fuse_model(self.armor_model)

  @staticmethod
  def fuse_model(model: YOLO) -> None:
    model.fuse()
    for m in model.modules():
      if isinstance(m, Detect):  
        m.format = ''
        m.export = True
      elif isinstance(m, C2f):
        m.forward = m.forward_split
      
  def detect(self, ori_img: np.ndarray, scaled_img: np.ndarray) -> list:
    '''
    识别车辆并分类
    '''
    scale = ori_img.shape[0] / scaled_img.shape[0]
    
    # 识别车辆
    results : Results = self.robot_detector_model.track(scaled_img, persist=True, tracker='bytetrack.yaml', conf=0.15)[0]
    #results = self.robot_detector_model.predict(scaled_img, conf=0.15)[0]
    if results.boxes is None or len(results.boxes) == 0:
      return []
    
    detector_results = []
    rois = []
    for box in results.boxes:
      x1, y1, x2, y2 = box.xyxy[0] * scale
      roi = ori_img[int(y1):int(y2), int(x1):int(x2), :]
      # rgb to bgr
      roi = cv2.cvtColor(roi, cv2.COLOR_RGB2BGR)
      rois.append(roi)

    armors_list  = self.armor_model.predict(rois, imgsz=224, conf=0.7)
    for i, armors in enumerate(armors_list):
      box = results.boxes[i]
      
      track_id = -1
      if box.id is not None:
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
      
      if track_id != -1:  
        self.tracked[track_id] = detector_result
        
      detector_results.append(detector_result)
        
       
    return detector_results