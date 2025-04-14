import rclpy
import time, os
from rclpy.node import Node
from sensor_msgs.msg import Image
from monicar3_interfaces.msg import Detections
from cv_bridge import CvBridge
from ultralytics import YOLO

# Initialize YOLO model
rosPath = os.path.expanduser('~/ros2_ws/src/monicar3/monicar3_yolo/weights/')
yolomodel = rosPath + 'monicar3.pt'

# Initialize Yolo network
model = YOLO(yolomodel,task='detect')
#please run only once to convert from pt to ncnn
model.export(format="ncnn")    # create ncnn model 
