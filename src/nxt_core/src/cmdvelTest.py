#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist


#run main behavior code
def run_node():
	rospy.init_node('cmdvelTest')
	cmdvel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
	while not rospy.is_shutdown():
		msg = Twist()
		msg.linear.x = 0.01 #m/s
		cmdvel_pub.publish(msg)
		rospy.sleep(1)


#execute main code
if __name__=='__main__':
    try:
	run_node()
    except rospy.ROSInterruptException:
	pass
