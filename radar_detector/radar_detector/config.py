from ament_index_python import get_package_share_directory
import os

class Config:
  '''
  整体配置
  '''
  def __init__(self) -> None:
    self.detector_config = DetectorConfig()
    self.device = "cuda:0"
    self.color_names = ["Blue", "Red", "Unkonw"]
    self.class_names = ["1","2", "3", "4", "5", "Sentry"]
    self.category_to_id = {0:1, 1:2, 2:3, 3:4, 4:5, 5:6}
    self.extract_roi = False
    self.debug = True
    
  def __str__(self) -> str:
    str = "debug: {}  device: {} detector_config\n\t: {}\n color_names: {}, class_names: {}".format(
      self.debug, self.device, self.detector_config, self.color_names, self.class_names
    )
    return str

class DetectorConfig:
  '''
  识别器参数
  '''
  def __init__(self) -> None:
    self.num_category = 12
    self.car_model = 'car.pt'
    self.armor_model = 'armor.pt'
    self.yolo_prob_thresh = 0.2
    self.logger = None
    
  def __str__(self) -> str:
    str = "num_category: {} car_model: {} armor_model: {} yolo_prob_thresh: {}".format(
      self.num_category, self.car_model, self.armor_model, self.yolo_prob_thresh
    )
    return str  
    
def init_params(node) -> Config:
    '''
    从ros2参数服务器中读取参数
    '''
    config = Config()
    
    node.declare_parameter("debug", True)
    debug = node.get_parameter("debug").get_parameter_value().bool_value
    
    node.declare_parameter("car_model", "car.onnx")
    car_model = node.get_parameter("car_model").get_parameter_value().string_value
    car_model = os.path.join(get_package_share_directory("radar_detector"), "model", car_model)
    
    node.declare_parameter("armor_model", "armor.pt")
    armor_model = node.get_parameter("armor_model").get_parameter_value().string_value
    armor_model = os.path.join(get_package_share_directory("radar_detector"), "model", armor_model)
    
    
    node.declare_parameter("device", "cuda:0")
    device = node.get_parameter(
            "device").get_parameter_value().string_value
    
    node.declare_parameter("extract_roi", False)
    extract_roi = node.get_parameter("extract_roi").get_parameter_value().bool_value

    config.detector_config.car_model = car_model  
    config.detector_config.armor_model = armor_model 
    config.detector_config.num_category = len(config.class_names)
    config.detector_config.logger = node.get_logger()
    config.device = device
    config.debug = debug
    config.extract_roi = extract_roi
    return config    