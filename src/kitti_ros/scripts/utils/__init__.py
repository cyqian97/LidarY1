"""Tools for parse tracklet_labels.xml, color generation, etc."""
import os
import sys
# os.chdir('/home/cyqian/autodrive/LidarMore/seg1/src/kitti_ros/scripts/utils')

# sys.path.append('/home/cyqian/autodrive/LidarMore/seg1/src/kitti_ros/scripts/utils')
# # import XmlParser
# print("#################")
# print(os. getcwd())
# print("#################")
from .XmlParser import XmlParser
from .Publisher import Publisher
from .KittiPreprocessor import KittiPreprocessor

__author__ = "Gary"
__email__ = "chenshj35@qq.com"
