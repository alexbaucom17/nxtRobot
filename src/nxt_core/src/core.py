#!/usr/bin/env python

import nxt
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import copy


WHEEL_DIAM = 0.03 #meters
BASE_DIST = 0.1 #meters


def core_node():

	#start ros node
	rospy.init_node('nxt_core')

	#configure nxt
	port_left =  nxt.PORT_B
	port_right =  nxt.PORT_C
	base = wheel_base(port_left,port_right)

	#start ros node
	rospy.init_node('nxt_core')
	rospy.Subscriber("/cmd_vel", Twist, base.set_vel)
	#pubUS = rospyPublisher("/scanUS", LaserScan, queue_size = 10) #TODO: confirm scan type

	#loop and do stuff
	r = rospy.Rate(50) #Hz
	while not rospy.is_shutdown():
		base.update_motors()
		#scanData = bot.get_US()
		#pubUS.pub(scanData)
		r.sleep()

	#clean up at exit
	base.stop_all()

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
		self.prev_tacho = self.motor.get_tacho().tacho_count

	def cap_output(self,output):
		if output > 100:
			output = 100
		elif output < -100:
			output = -100
		else:
			output = int(output)
		return output

	def update(self,setpoint):
		#setpoint is a speed to maintain in rad/s
		
		#get measurements
		cur_tacho = self.motor.get_tacho().tacho_count #tachometer measurment in degrees +/- 1 deg
		cur_time = rospy.get_time()

		#find derivatives
		dt = cur_time - self.prev_time
		ds = (cur_tacho - self.prev_tacho) * math.pi/180
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
		return self.cap_output(output)


class wheel_base:
	' Class for a wheeled robot '
	
	def __init__(self, port_left, port_right):

		#connet to brick
		rospy.loginfo("Connecting to brick...")
		try:
			self.b = nxt.locator.find_one_brick()
		except:
			rospy.logerr("No connection to NXT brick. Aborting")
			return None

		#set up motors
		self.mL = nxt.Motor(self.b, port_left)
		self.mR = nxt.Motor(self.b, port_right)

		#angular speeds start at 0
		self.w_L = 0
		self.w_R = 0

		#set up PID controllers
		self.pidL = PIDcontroller(10,0.1,0,self.mL)
		self.pidR = PIDcontroller(10,0.1,0,self.mR)
		
	def stop_all(self):
		self.mL.idle()
		self.mR.idle()

	def update_motors(self):
		'update current motor speeds'
	
		#call PID updates
		#negative b/c of motor direction
		powL = -self.pidL.update(self.w_L)
		powR = -self.pidR.update(self.w_R)
	
		#pass PID output to motor run method
		self.mL.run(powL)
		self.mR.run(powR)
		rospy.loginfo("Power: [%f, %f]"%(powL,powR))			


	def set_vel(self,msg):
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

		
class Scanner:

	def __init__(self,motor_port, us_port, brick):
		self.motor  = nxt.Motor(brick, motor_port)
		self.US = nxt.Ultrasonic(brick, us_port)
		self.PID = PIDcontroller(1,0,0,self.motor)
		self.motor_speed = 1 #motor speed in rad/s
		self.speed_factor = 4 #motor speed/sensor speed - gear ratio
		self.scan_ready = False
		self.scan_data  = []
		self.old_a = 0
		self.scan_da = 5 #how far apart scans should be in degrees
		self.min_angle = -90
		self.max_angle = 90
	
	def is_ready(self):
		return self.scan_ready

	def get_scan(self):
		"""Get scan if data is ready"""		
		if self.scan_ready:
			
			scan = LaserScan()
			
	
			self.scan_ready = False		
			self.scan_data = []
			return scan_return
		else:
			return None

	def update(self):
		"""Update to keep motors running and sensor scanning"""
		
		#check to see if we should get another ping yet
		cur_a = self.speed_factor * self.motor.get_tacho()
		if abs(cur_a - self.old_a) > scan_da:
			self.scan_data.append(self.US.get_distance())
		
		#check to see if we need to change scan directions
		if cur_a > self.max_angle and self.motor_speed > 0:
			self.motor_speed = -self.motor_speed
			self.scan_ready = True
		if cur_a < self.min_angle and self.motor_speed < 0:
			self.motor_speed = -self.motor_speed
			self.scan_ready = True

		#update old angle
		self.old_a = cur_a

		#compute controlled motor speed and run
		self.motor.run(self.PID.update(self.motor_speed))
		

if __name__ == '__main__':
    try:
        core_node()
    except rospy.ROSInterruptException: pass

