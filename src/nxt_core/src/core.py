#!/usr/bin/env python

import nxt
import rospy
from geometry_msgs.msg import Twist
import math


WHEEL_DIAM = 0.03 #meters
BASE_DIST = 0.1 #meters


def core_node():

	#start ros node
	rospy.init_node('nxt_core')

	#configure nxt
	motors = { 'us': nxt.PORT_A, 'left': nxt.PORT_B, 'right': nxt.PORT_C }
	sensors = {'touch': nxt.PORT_1, 'us': nxt.PORT_4}
	bot = robot(motors, sensors)

	#set pub/subs
	rospy.Subscriber("/cmd_vel", Twist, bot.set_vel)
	#pubUS = rospyPublisher("/scanUS", type??, queue_size = 10)

	#loop and do stuff
	r = rospy.Rate(10) #10 Hz
	while not rospy.is_shutdown():
		bot.update_motors()
	#	scanData = bot.get_US()
	#	pubUS.pub(scanData)
		r.sleep()

class PIDcontroller:
	'''Simple PID motor controller'''

	def __init__(self,Kp,Ki,Kd,M):
		self.P = Kp
		self.I = Ki
		self.D = Kd
		self.sum_error = 0
		self.prev_error = 0
		self.prev_time = rospy.get_time()
		self.motor = M #pass motor object for each tachometer data
		self.prev_tacho = self.motor.get_tacho()

	def update(self,setpoint):
		#setpoint is a speed to maintain in rad/s
		
		#get measurements
		cur_tacho = self.motor.get_tacho() #tachometer measurment in degrees +/- 1 deg
		cur_time = rospy.get_time()

	#	print cur_tacho
	#tacho appears to be a 3 tuple of (tacho, block_tacho, rotations)
	#need to test this and grab the right one
	
		#find derivatives
		dt = cur_time - self.prev_time
		#ds = (cur_tacho - self.prev_tacho) * math.pi/180
		ds = 1
		dw = ds/dt

		#error calculations
		error = setpoint - dw
		d_error = error - self.prev_error
		
		#store values
		self.sum_error += error
		self.prev_time = cur_time
		self.prev_tacho = cur_tacho

		#compute and return output
		output = self.P*error + self.I*self.sum_error + self.D*d_error
		return output

class robot:
	''' Class for a wheeled robot with an actuated ultrasonic sensor for mapping '''
	
	#motors - {name: port}, valid names: left, right, us
	#sensors - {name: port}, valid names: touch, us, light
	def __init__(self, motors, sensors):

		#connet to brick
		rospy.loginfo("Connecting to brick...")
		try:
			self.b = nxt.locator.find_one_brick()
		except:
			rospy.logerr("No connection to NXT brick. Aborting")
			return None

		#set up motors
		self.mU = nxt.Motor(self.b, motors['us'])
		self.mL = nxt.Motor(self.b, motors['left'])
		self.mR = nxt.Motor(self.b, motors['right'])

		#set up sensors
		self.touch = nxt.Touch(self.b, sensors['touch'])	
		#self.us = nxt.Ultrasonic(self.b, sensors['us'])

		#angular speeds start at 0
		self.w_U = 0
		self.w_L = 0
		self.w_R = 0

		#set up PID controllers
		self.pidU = PIDcontroller(1,0,0,self.mU)
		self.pidL = PIDcontroller(1,0,0,self.mL)
		self.pidR = PIDcontroller(1,0,0,self.mR)
		

	def update_motors(self):
		'update current motor speeds'
	
		#call PID updates
		powU = self.pidU.update(self.w_U)
		powL = self.pidL.update(self.w_L)
		powR = self.pidR.update(self.w_R)
	
		#pass PID output to motor run method
		self.mU.run(powU)
		self.mL.run(powL)
		self.mR.run(powR)			


	def set_vel(msg):
		rospy.loginfo("Received a /cmd_vel message!")
		rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
		rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

		# Do velocity processing here:
		v = msg.linear.x
		w = msg.angular.z

		#basic 2d cart assumption (v in m/s)
		#requires wheels to be in line with center of rotation
		v_L = v - 0.5*BASE_DIST*w
		v_R = v + 0.5*BASE_DIST*w

		#convert to rad/s and store for update_motors to use
		self.w_L = v_L/(WHEEL_DIAM/2)
		self.w_R = v_R/(WHEEL_DIAM/2)
		


if __name__ == '__main__':
    try:
        core_node()
    except rospy.ROSInterruptException: pass

