#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def mover():
     pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
     rospy.init_node('moveturtlefwd')
     rate=rospy.Rate(10)
     velocity =Twist()
	
     velocity.linear.x = 0.3
     velocity.linear.y = 0
     velocity.linear.z = 0
 
     velocity.angular.z=-0.3
     velocity.angular.y=0
     velocity.angular.x=0

     change_time=rospy.Time.now()
     turning=False
     while not rospy.is_shutdown():
       if turning:
	   velocity.angular.z=0.3
           pub.publish(velocity)

       else:
           pub.publish(velocity)
	   turning = True


       if rospy.Time.now() > change_time:
          turning = not turning
          change_time=rospy.Time.now() + rospy.Duration(10)



if __name__ == '__main__':
    try:
        mover()
    except rospy.ROSInterruptException:
        pass
