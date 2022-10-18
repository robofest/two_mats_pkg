#!/usr/bin/env python3

# 
# Got error message: ModuleNotFoundError.
# Cause: cfg file name was same as the package!!!! Not allowed
# 
from numpy import flip
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from two_mats_pkg.cfg import TwoMatsConfig   # packageName.cfg
from dynamic_reconfigure.server import Server
import numpy as np

vel_msg = Twist()
bridge = CvBridge()

def dyn_rcfg_cb(config, level):
  global thresh, drive, flip
  thresh = config.thresh
  drive = config.enable_drive
  flip = False
  return config

def move(x, z):
  vel_msg.linear.x = x
  vel_msg.angular.z = z
  velocity_pub.publish(vel_msg)
  return

def image_callback(ros_image):
  try: #convert ros_image into an opencv-compatible image
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  #from now on, you can work exactly like with opencv
  if flip == True:
    cv_image = cv2.flip(cv_image, 1) # flip to see ourselves in a mirror 
  cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
  (rows,cols,channels) = cv_image.shape

  gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

  ret, bw_image = cv2.threshold(gray_image, # input image
                                    thresh,        # threshol_value,
                                    255,        # max value in image
                                    cv2.THRESH_BINARY) # threshold type
                                    # cv2.THRESH_BINARY_INV) # Inverted

  #num_white_pix = np.sum(bw_image == 255)
  num_white_pix = cv2.countNonZero(bw_image)
  white_pct = (100* num_white_pix) / (rows * cols)
  font = cv2.FONT_HERSHEY_SIMPLEX
  cv2.putText(bw_image,f"% White = {white_pct:.1f}%",(10,rows-10), font, 1,(127,127,127),2,cv2.LINE_AA)
  if drive == True:
    if white_pct > 30:     
      move (0, 0) # stop
    else:
      move (0.2, 0)
  else:
      move (0, 0)

  cv2.imshow("Image window", bw_image)
  cv2.waitKey(3)
  
if __name__ == '__main__':
  rospy.init_node('stop_at_matedge', anonymous=True)
  
  imgtopic = rospy.get_param("~img_topic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback)

  twisttopic = rospy.get_param("~twist_topic_name") # private name
  velocity_pub = rospy.Publisher(twisttopic, Twist, queue_size=1)
  
  srv = Server(TwoMatsConfig, dyn_rcfg_cb)

  r = rospy.Rate(20)
  for _ in range(109):
    move(0.0, 0.3) # Z value is positive. Turn Left
    r.sleep()
  move(0, 0) # Stop spinning

  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
