#!usr/bin/env python
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


# Declaring the variables used in the following code  -  new comment added for clarification

Right  = 0
Zero   = 1
Left   = 2
Slow   = 0
Medium = 1
Fast   = 2
Near   = 0
Medium = 1
Far    = 2 


   
	
class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('forward_demo')
		super().__init__('scan_demo')
		self.scan_sub = self.create_subscription(LaserScan,'scan', self.laser_callback, qos_profile_sensor_data)
		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
		self.cmd = Twist()
		timer_period = 1.0 # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)

#=======================================================================Rule Base==============================================================================================

# Declaring Rule base in a 2D array using 'for' loop.  -  new comment added for clarification

		self.RE_Rule_Set_Def = []
		for i in range(3):
			for j in range(3):
				Rule_Set_Row = []
				Rule_Set_Row.append(i)
				Rule_Set_Row.append(j)
				self.RE_Rule_Set_Def.append(Rule_Set_Row)
					
		self.RE_Rule_Set_Def_Out = [[Slow,Left],[Slow,Left],[Slow,Left],[Slow,Right],[Medium,Zero],[Slow,Right],[Slow,Right],[Medium,Right],[Fast,Right]]

#==============================================================================================================================================================================
#=================================================================== Main Function ============================================================================================


	def Fuzzy_RE(self):
		
		# In the below 2D array 0th array is Near, 1st array is medium and 2nd array is far.  -  new comment added for clarification

		Distance_From_Sensor = [[ 0.00,0.00 ,0.20,0.30],[ 0.20,0.30,0.30,0.40],[0.30,0.40,2.00,4.00]]
		
		# In the below 2D array 0th array is Right, 1st array is Zero and 2nd array is Left.  -  new comment added for clarification

		Direction = [[-0.75,-0.50,-0.50,-0.25],[-0.25,0.00,0.00,0.25],[0.25,0.50,0.50,0.75]]
		Direction_Centr_Array = [-0.50,0.00,0.50]		
		
		# In the below 2D array 0th array is Slow, 1st array is Medium and 2nd array is Fast.  -  new comment added for clarification

		Speed     = [[0.00,0.10,0.10,0.20],[0.20,0.30,0.30,0.40],[0.40,0.50,0.50,0.60]]
		Speed_Centr_Array = [0.1,0.30,0.50]
	
#=================================================================== Reading Sensor Values =====================================================================================
#===================================================================   Right Front Sensor   =====================================================================================


		Right_Front_Sensor = min(self.laser_ranges[1080:1360])

		if Right_Front_Sensor >1:
			Right_Front_Sensor =1

#===================================================================   Front Back Sensor   =====================================================================================

		Right_Back_Sensor  = min(self.laser_ranges[ 880:1079])

		if Right_Back_Sensor > 1:
			Right_Back_Sensor = 1

#===================================================================    Debuging Step    =====================================================================================
			
		print("=================Sensor Val====================")
		
		print("RFS")
		print(Right_Front_Sensor)
		print("RBS")
		print(Right_Back_Sensor)
		
		print("==============================================")


#=============================================================================================================================================================================
#============================================================== Checking Sensor Conditions  ==================================================================================
# Here we will find each sensor is Near,Medium or Far and will also determine wether its rising edge,
# falling edge or neither and it will make and array for both  - new comment added for clarification
 
		RFS_Condition = []
		RBS_Condition = []

		RFS_Slop = []
		RBS_Slop = []

		for i in range(len(Distance_From_Sensor)):
			for j in range(len(Distance_From_Sensor[0])-1):
			
				if (Right_Front_Sensor >= Distance_From_Sensor[i][j]) and (Right_Front_Sensor <= Distance_From_Sensor[i][j+1]):
				
					RFS_Condition.append(i)
					RFS_Slop.append(j)
				if (Right_Back_Sensor  >= Distance_From_Sensor[i][j]) and (Right_Back_Sensor <= Distance_From_Sensor[i][j+1]):	

					RBS_Condition.append(i)
					RBS_Slop.append(j)
#======================================================================  Debuging Step  ====================================================================================		
		
		print("============Condition N M F===================")
		print(RFS_Condition)
		print(RBS_Condition)
		
		print("==============================================")
		
		print("=============Slop R FL FALL ==================")
		print(RFS_Slop)
		print(RBS_Slop)
		
		print("==============================================")

#=============================================================================================================================================================================			
#============================================================  Calculating the Membership Values  ===========================================================================
# Using the above array created we will calculate the MF values for each sensor  -  new comment added for clarification
		
		RFS_Mf_Array = []
		RBS_Mf_Array = []

#=============================================================================================================================================================================				

		for i in range(len(RFS_Condition)):
			
			k = RFS_Condition[i]
			
			# Rising Edge
			if RFS_Slop[i]==0:
				
				RFS_Mf =  (Distance_From_Sensor[k][1]  - Right_Front_Sensor)/ (Distance_From_Sensor[k][1]-Distance_From_Sensor[k][0])
				RFS_Mf_Array.append(RFS_Mf)
			
			# Flat
			elif RFS_Slop[i]==1:
				RFS_Mf = 1 * Right_Front_Sensor
				RFS_Mf_Array.append(RFS_Mf)
			
			# Falling Edge
			else:
				RFS_Mf = (Right_Front_Sensor - Distance_From_Sensor[k][2])/ (Distance_From_Sensor[k][3] - Distance_From_Sensor[k][2])
				RFS_Mf_Array.append(RFS_Mf)

#=============================================================================================================================================================================				

		for i in range(len(RBS_Condition)):
			
			q = RBS_Condition[i]
			# Rising Edge
			if RBS_Slop[i]==0:
				RBS_Mf =  (Distance_From_Sensor[q][1]  - Right_Back_Sensor) / (Distance_From_Sensor[q][1] - Distance_From_Sensor[q][0])
				RBS_Mf_Array.append(RBS_Mf)
			# Flat
			elif RBS_Slop[i]==1:
				RBS_Mf = 1 * Right_Back_Sensor
				RBS_Mf_Array.append(RBS_Mf)
			# Falling Edge
			elif RBS_Slop[i]==2:
				RBS_Mf = (Right_Back_Sensor - Distance_From_Sensor[q][2]) / (Distance_From_Sensor[q][3] - Distance_From_Sensor[q][2])
				RBS_Mf_Array.append(RBS_Mf)

#======================================================================= Debugging Step ========================================================================================				
				
		print("====================MF ARRAY===========================")
		
		print(RFS_Mf_Array)
		print(RBS_Mf_Array)
		
		print("=======================================================")

#=============================================================================================================================================================================
#=================================================== MF and sensor condition combination function ============================================================================

#  With the calculated we will create two 2D array with the sensor conditions and its MF values -  new comment added for clarification

		Sensor_Mf_Matrix  = []
		Rule_Set_Matrix   = []
		
		for i in range(len(RFS_Condition)):
			
			for j in range(len(RBS_Condition)):

				Sensor_Mf_Row = []
				Rule_Set_Matrix_Row =[]

				Sensor_Mf_Row.append(RFS_Mf_Array[i])
				Sensor_Mf_Row.append(RBS_Mf_Array[j])

				Rule_Set_Matrix_Row.append(RFS_Condition[i])
				Rule_Set_Matrix_Row.append(RBS_Condition[j])
				
				
				Sensor_Mf_Matrix.append(Sensor_Mf_Row)
				Rule_Set_Matrix.append(Rule_Set_Matrix_Row)

#==================================================================== Debugging Step ===========================================================================================

		print("================CONDITION Matrix=======================")
		
		print(Rule_Set_Matrix)
		print(Sensor_Mf_Matrix)
		
		print("======================================================")	

#================================================================================================================================================================================
#===================================================================== Sensor MF Min. Array =====================================================================================		
# Using the MF value matrix we find the min of each and make MF array  -  new comment added for clarification
		
		Sensor_Mf_Min_Array = []
	
		for i in range(len(Sensor_Mf_Matrix)):
		
			Sensor_Mf_Min_Array.append(min(Sensor_Mf_Matrix[i]))

#=======================================================================  Debugging Step ========================================================================================			
		
		print("================== Min MF Array ======================")

		print(Sensor_Mf_Min_Array)

		print("======================================================")

#================================================================================================================================================================================		
#==========================================================================Sum of all the MF's====================================================================================

		Sum_MF           = 0 
		
		for i in range(len(Sensor_Mf_Min_Array)):

			Sum_MF = Sum_MF + Sensor_Mf_Min_Array[i]

#=======================================================================  Debugging Step ========================================================================================

		print("====================Sum MF Array===========================")

		print(Sum_MF)

		print("===========================================================")

#===============================================================================================================================================================================
#=====================================================================  Calculating Firing Strength =============================================================================
# Checking all 9 conditions and calculating speed and steering for the robot  -  new comment added for clarification

		Firing_Strength_X = 0
		Firing_Strength_Z = 0
		
		for i in range(len(Sensor_Mf_Min_Array)):
			
			for j in range(len(self.RE_Rule_Set_Def)):
				if Rule_Set_Matrix[i] == self.RE_Rule_Set_Def[j]:
					
					print(self.RE_Rule_Set_Def_Out[j])
					
					G = self.RE_Rule_Set_Def_Out[j]
					M = G[0]
					N = G[1]
					Firing_Strength_X = Firing_Strength_X + (Speed_Centr_Array[M] * Sensor_Mf_Min_Array[i])
					Firing_Strength_Z = Firing_Strength_Z + (Direction_Centr_Array[N] * Sensor_Mf_Min_Array[i])
		
		X_Value = Firing_Strength_X/Sum_MF
		Z_Value = Firing_Strength_Z/Sum_MF
		
		self.cmd.linear.x  = X_Value
		self.cmd.angular.z = Z_Value
				
#=======================================================================  Debugging Step ========================================================================================

		print("====================Ouput Values===========================")

		print(X_Value)
		print(Z_Value)
		
		print("===========================================================")

#================================================================================================================================================================================
#================================================================================================================================================================================
	
	def timer_callback(self):
		self.Fuzzy_RE()
		self.publisher_.publish(self.cmd)
		
	        	

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
