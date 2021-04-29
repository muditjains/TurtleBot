#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import radians
def mover():
     pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
     rospy.init_node('moveturtlefwd')
     rate=rospy.Rate(1)
     fwd =Twist()
     turn = Twist()

     fwd.linear.x = 0.5
     fwd.linear.y = 0
     fwd.linear.z = 0
 
     turn.angular.z=radians(180)
     turn.angular.y=0
     turn.angular.x=0

     change_time=rospy.Time.now()
     turning=False
     while not rospy.is_shutdown():
       if turning:
           pub.publish(fwd)

       else:
           pub.publish(turn)


       if rospy.Time.now() > change_time:
          turning = not turning
          change_time=rospy.Time.now() + rospy.Duration(0.5)
        


if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
