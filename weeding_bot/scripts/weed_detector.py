#!/usr/bin/env python

import rospy
import numpy as np
import tf

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud


class WeedDetector:

  def __init__(self, robot_name):
    #get the name prefix
    self.robot_name = robot_name

    self.bridge = CvBridge()

    #initialise subscribers and publishers
    self.image_sub = rospy.Subscriber("{}/kinect2_camera/hd/image_color_rect".format(self.robot_name), Image, self.image_callback)

    self.weed_cloud_pub = rospy.Publisher("{}/weed_points".format(self.robot_name), PointCloud, queue_size=10)

    #get camera info once and set our model
    self.camera_info_data = rospy.wait_for_message('/{}/kinect2_camera/hd/camera_info'.format(self.robot), CameraInfo)
    self.camera_model = image_geometry.PinholeCameraModel()
    self.camera_model.fromCameraInfo(camera_info_data)


  #get image data
  def image_callback(self, data):
    namedWindow("Image window")
    namedWindow("blur")
    namedWindow("canny")
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
    print np.mean(gray_img)
    img2 = blur(gray_img, (3, 3))
    imshow("blur", img2)
    img3 = Canny(gray_img, 10, 200)
    imshow("canny", img3)

    imshow("Image window", cv_image)
    waitKey(1)




def main(args):
  #initialize node
  rospy.init_node('sprayer_node', anonymous=True)

  #get arguments and instantiate object of class
  robot_name = sys.argv[1].decode('string-escape')
  WeedDetector(robot_name)

  #start spinning
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
    
        
if __name__ == '__main__':
  main(sys.argv)
