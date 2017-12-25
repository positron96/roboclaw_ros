#!/usr/bin/env python

## Author: positron
##
## Formulae taken from
## http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

import roslib
import rospy
from roboclaw.msg import MotorSpeeds
from geometry_msgs.msg import Twist
#from watchdog import Watchdog


WHEEL_RADIUS = None
motor_pub = None 
Lx, Ly = None,None

#watchdog = Watchdog(0.2, lambda : noSpeed()  )
	
def cmd_vel(data):
	global motor_pub

	vx = data.linear.x
	vy = data.linear.y
	wz = data.angular.z

	global WHEEL_RADIUS, Lx, Ly
	R = WHEEL_RADIUS
	Lw = (Lx + Ly)*wz

	w1 = (vx - vy - Lw) / R
	w2 = (vx + vy + Lw) / R
	w3 = (vx + vy - Lw) / R
	w4 = (vx - vy + Lw) / R

	v = MotorSpeeds()
	v.Speeds = [w3,w1,w4,w2]
	motor_pub.publish( v )
	#watchdog.kick();
	
if __name__ == '__main__':
	try:
		rospy.init_node('mecanum_wheel_processor')
		rospy.Subscriber("cmd_vel", Twist, cmd_vel)
		motor_pub = rospy.Publisher('cmd_motors', MotorSpeeds, queue_size=10)	  

		rospy.loginfo("mecanum_wheel_processor: Starting node")			

		rate = int( rospy.get_param('~rate', 10) )
		WHEEL_RADIUS = float( rospy.get_param('~wheel_radius', 0.1) )
		Lx = float( rospy.get_param('~wheelbase', 1) ) / 2.0
		Ly = float( rospy.get_param('~track', 1) ) / 2.0
		r = rospy.Rate(rate)
		while not rospy.is_shutdown():
			#watchdog.check()
			r.sleep()	

		rospy.loginfo("mecanum_wheel_processor: exit")

	except rospy.ROSInterruptException as e: 
		rospy.loginfo( str(e) )
