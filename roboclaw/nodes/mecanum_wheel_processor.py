#!/usr/bin/env python

import roslib
import rospy
from roboclaw.msg import SpeedData
from geometry_msgs.msg import Twist
from watchdog import Watchdog


WHEEL_RADIUS = None
motor_pub = None 
K=0.2
vr,vl=0,0

watchdog = Watchdog(0.2, lambda : noSpeed()  )
	
def cmd_vel(data):
	global motor_pub

	vx = data.linear.x * M2T # mps -> ticks per second
	vth = data.angular.z * width * M2T # rad per second -> ticks per second
	
	global vl,vr
	vr = vr*(1-K) + (vx + vth)*K
	vl = vl*(1-K) + (vx - vth)*K
	#rospy.loginfo( "m,"+ str(int(vr))+","+ str(int(vl))	)	
	v = MotorData()
	v.right = int(vr)
	v.left = int(vl)
	motor_pub.publish( v )
	watchdog.kick();
	
def noSpeed() :
	global vl,vr
	vr = vr*0.1
	vl = vl*0.1
	v = MotorData()
	v.right = int(vr)
	v.left = int(vl)
	#rospy.loginfo("mecanum_wheel_processor: reducing speed: vr:"+str(vr)+", vl:"+ str(vl) )
	motor_pub.publish( v )

if __name__ == '__main__':
	try:
		rospy.init_node('mecanum_wheel_processor')
		rospy.Subscriber("cmd_vel", Twist, cmd_vel)
		motor_pub = rospy.Publisher('cmd_motors', SpeedData, queue_size=10)	  

		rospy.loginfo("mecanum_wheel_processor: Starting node")	
		

		rate = int( rospy.get_param('~rate', 10) )
		WHEEL_RADIUS = float( rospy.get_param('~wheel_radius', 0.1) )
		r = rospy.Rate(rate)
		while not rospy.is_shutdown():
			watchdog.check()
			r.sleep()	

		rospy.loginfo("mecanum_wheel_processor: exit")

	except rospy.ROSInterruptException as e: 
		rospy.loginfo( str(e) )
