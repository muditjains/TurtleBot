#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan



def scan_callback(msg):
  #length print
  pub=rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

  l=len(msg.ranges)
  print("The length of the msg array is : "+str(l))
  # entire list print
  print msg.ranges
  # 0 degree print
  range_ahead = msg.ranges[0]
  print("Front "+str(range_ahead))
  # print 180 degree
  range_behind = msg.ranges[179]
  print("Behind "+str(range_behind))
  # print min
  print("Minimum "+str(min(msg.ranges)))
  # print mean distance
  print("Average "+str(sum(msg.ranges)/l))

  left = msg.ranges[60:180]
  right = msg.ranges[180:300]
  front = msg.ranges[0:60]+msg.ranges[300:360]
  min_left = min(left)
  min_right = min(right)
  min_front = min(front)
  mean_left = np.mean(left)
  mean_right = np.mean(right)
  mean_front = np.mean(front)
  print(min_front)
  print(min_left)
  print(min_right)
  print(mean_front)
  print(mean_front)
  print(mean_front)
  upward = Twist()
  backward = Twist()
  leftward = Twist()
  rightward = Twist()
  upward.linear.x = 0.3
  backward.linear.x = -0.3
  leftward.angular.z = 0.3
  rightward.angular.z = -0.3
  if(min_front > 0.3):
  	pub.publish(upward)
  elif(min_front < 0.3 and min_left > 0.3):
  	pub.publish(leftward)
  elif(min_front < 0.3 and min_left < 0.3 and min_right > 0.3):
  	pub.publish(rightward)
  else:
  	pub.publish(backward)
  # END MEASUREMENT

rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()
