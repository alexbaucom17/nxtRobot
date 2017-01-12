#!/usr/bin/env python

import nxt
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import math


WHEEL_DIAM = 0.03 #meters
BASE_DIST = 0.1 #meters


def core_node():

	#configure nxt
	motors = { 'us': nxt.PORT_A, 'left': nxt.PORT_B, 'right': nxt.PORT_C }
	sensors = {'touch': nxt.PORT_1, 'us', nxt.PORT_4}
	bot = robot(motors, sensors)

	#start ros node
	rospy.init_node('nxt_core')
	rospy.Subscriber("/cmd_vel", Twist, bot.set_vel())
	pubUS = rospyPublisher("/scanUS", type??, queue_size = 10)

	#loop and do stuff
	while not rospy.is_shutdown():
		bot.update_motors()
		scanData = bot.get_US()
		pubUS.pub(scanData)
		rospy.sleep(1.0)

class PIDcontroller:
'Simple PID motor controller'

	def __init__(self,Kp,Ki,Kd):
		self.P = Kp
		self.I = Ki
		self.D = Kd
		self.error = 0
		self.prev_time = ??? Best clock to use?

	def update(self,setpoint,measurement, measurement_time):
		#do some math
		pass



class robot:
' Class for a wheeled robot with an actuated ultrasonic sensor for mapping '
	
	#motors - {name: port}, valid names: left, right, us
	#sensors - {name: port}, valid names: touch, us, light
	def __init__(self, motors, sensors):

		#connet to brick
		try:
			self.b = nxt.locator.find_one_brick()
		except:
			rospy.logerror("No connection to NXT brick. Aborting")
			return None

		#set up motors
		self.mU = nxt.Motor(self.b, motors['us'])
		self.mL = nxt.Motor(self.b, motors['left'])
		self.mR = nxt.Motor(self.b, motors['right'])

		#set up sensors
		#TODO maybe make this a bit more flexible?
		self.touch = nxt.Touch(self.b, sensors['touch'])	
		self.us = nxt.Ultrasonic(self.b, sensors['us'])


	def update_motors(self)
		'update current motor speeds'

		#get encoder measurements
	
		#call PID updates
	
		#pass PID output to motor run method

		pass		
			


	def set_vel(msg):
		rospy.loginfo("Received a /cmd_vel message!")
		rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
		rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

		# Do velocity processing here:
		v = msg.linear.x
		w = mxg.angular.z

		#basic 2d cart assumption (v in m/s)
		#requires wheels to be in line with center of rotation
		v_l = v - 0.5*BASE_DIST*w
		v_r = v + 0.5*BASE_DIST*w

		#convert to rad/s and store for update_motors to use
		self.w_l = v_l/(WHEEL_DIAM/2)
		self.w_r = v_r/(WHEEL_DIAM/2)
		




if __name__ == '__main__':
    try:
        core_node()
    except rospy.ROSInterruptException: pass

