#!usr/bin/env python
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

#class Def_Val():
#	Desired_Distance = 0.5
#	Error = 0
#	Error_i = 0
#	Error_d = 0
#	Error_Previous = 0
#	Kp = 0.5
#	Ki = 0.15
#	Kd = 0.4
	
class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('forward_demo')
		super().__init__('scan_demo')
		self.scan_sub = self.create_subscription(LaserScan,'scan', self.laser_callback, qos_profile_sensor_data)
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 1.0 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.PID_value = 0
		self.Desired_Distance = 0.5
		self.Error = 0
		self.Error_i_sum = 0
		self.Error_i_Array = []
		self.Error_i = 0
		for i in range(10):
			self.Error_i_Array .append(0)
		self.Error_d = 0
		self.Error_Previous = 0
		self.Kp = 0.60
		self.Ki = 0.20
		self.Kd = 0.80
		self.i = 0
		
		
	
	def PID(self):
#========Reading Sensor Values and Calculating Deviation from expected distance===========	
		self.Error   = self.Desired_Distance - min(self.laser_ranges[1000:1160])
		
		self.Error_i_Array.append(self.Error)
#=================================Taking out inf Readings ================================		
		if self.Error > 10:
			self.Error = 0
		elif self.Error < -10:
			self.Error = 0	
#===========================Saving Past 10 Values========================================			
		self.Error_i_Array.append(self.Error)
		
		if(self.i >9):
			self.Error_i_Array.pop(0)

		self.Error_i_sum = 0	

		for j in range(10):
		
			self.Error_i_sum = self.Error_i_sum + self.Error_i_Array[j]	

		self.Error_i = self.Error_i_sum
#==================Getting deviation of current error from previous error==================	
		
		self.Error_d = self.Error - self.Error_Previous
		
		self.Error_Previous = self.Error
#=====================Calc Final PID value and updating it to Angular_Z=====================	

		
		self.PID_value = self.Kp * self.Error + self.Ki * self.Error_i + self.Kd*self.Error_d
		
		self.cmd.angular.z =  self.PID_value
		self.cmd.linear.x = 0.2

#==================Condition just to avoid robot hitting a wall infront======================	
		if(min(self.laser_ranges[0:80])<0.30):
			self.cmd.linear.x = 0.0
		if(min(self.laser_ranges[1360:1439])<0.30):	
	        	self.cmd.linear.x = 0.0
#======================================================================================			
	
	def timer_callback(self):
		self.PID()
		self.publisher_.publish(self.cmd)
		#self.get_logger().info(string)
	        #string = 'Publishing:' + str(self.cmd.linear.x)
	        	

	def laser_callback(self,msg):
		self.get_logger().info(str(msg.ranges[1080]))
		self.laser_ranges = msg.ranges
		
	

def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()
	rclpy.spin(minimal_publisher)
	
	minimal_publisher.destroy_node()
	rclpy.shutdown()
	
	
if __name__=='__main__':
	main()
