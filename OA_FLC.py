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

		self.Rule_Set_Def = []
		for i in range(3):
			for j in range(3):
				for k in range(3):
					Rule_Set_Row = []
					Rule_Set_Row.append(i)
					Rule_Set_Row.append(j)
					Rule_Set_Row.append(k)
					self.Rule_Set_Def.append(Rule_Set_Row)
					
		self.Rule_Set_Def_Out = [[Slow,Left],[Slow,Right],[Medium,Right],[Slow,Left],[Slow,Right],[Slow,Right],[Medium,Zero],[Slow,Right],[Medium,Left],[Slow,Left],[Slow,Left],[Slow,Right],[Slow,Left],[Slow,Right],[Medium,Right],[Medium,Right],[Fast,Zero],[Medium,Right],[Slow,Left],[Slow,Left],[Slow,Right],[Slow,Left],[Medium,Left],[Medium,Right],[Fast,Left],[Fast,Zero],[Fast,Zero]]



#==============================================================================================================================================================================
#=================================================================== Main Function ===========================================================================================================
	
	def Fuzzy_OA(self):
	
		
		# In the below 2D array 0th array is Near, 1st array is medium and 2nd array is far.  -  new comment added for clarification
	
		Distance_From_Sensor  = [[ 0.00,0.00 ,0.35 , 0.50],[ 0.35,0.50,0.50,0.65],[0.50,0.65,4.00,4.00]]

		# In the below 2D array 0th array is Right, 1st array is Zero and 2nd array is Left.  -  new comment added for clarification

		Direction             = [[-0.75,-0.50,-0.50,-0.25],[-0.25,0.00,0.00,0.25],[0.25,0.50,0.50,0.75]]
		Direction_Centr_Array = [-0.75,0.00,0.75]

		# In the below 2D array 0th array is Slow, 1st array is Medium and 2nd array is Fast.  -  new comment added for clarification
		
		Speed                 = [[0.00 ,0.10 ,0.10 ,0.20 ],[0.20 ,0.30,0.30,0.40],[0.40,0.50,0.50,0.60]]
		Speed_Centr_Array     = [0.10,0.20,0.30]

		
#=================================================================== Reading Sensor Values =====================================================================================
#===================================================================  Front Right Sensor   =====================================================================================
		Front_Right_Sensor = min(self.laser_ranges[1200:1380])
		
		if Front_Right_Sensor > 4:
			Front_Right_Sensor =3.5		
#===================================================================   Left Right Sensor   =====================================================================================
		Front_Left_Sensor  = min(self.laser_ranges[ 60:240])
		
		if Front_Left_Sensor > 4:
			Front_Left_Sensor =3.5
#===================================================================     Front Sensor     =====================================================================================		
		Front_Sensor_A	    = [min(self.laser_ranges[ 0:40]),min(self.laser_ranges[ 1400:1439])]
		
		Front_Sensor       = min(Front_Sensor_A)
		
		if Front_Sensor > 4:
			Front_Sensor =3.5
#===================================================================    Debuging Step    =====================================================================================
			
		print("=================Sensor Val====================")
		print("FLS")
		print(Front_Left_Sensor)
		print("FS")
		print(Front_Sensor)
		print("FRS")
		print(Front_Right_Sensor)
		print("==============================================")

#=============================================================================================================================================================================
#============================================================== Checking Sensor Conditions  ==================================================================================

# Here we will find each sensor is Near,Medium or Far and will also determine wether its rising edge,
# falling edge or neither and it will make and array for both  -  new comment added for clarification
 

		FRS_Condition = []
		FLS_Condition = []
		FS_Condition  = []
		
		FRS_Slop = []
		FLS_Slop = []
		FS_Slop  = []
		     		
		for i in range(len(Distance_From_Sensor)):
			for j in range(len(Distance_From_Sensor[0])-1):

				if (Front_Left_Sensor  >= Distance_From_Sensor[i][j]) and (Front_Left_Sensor <= Distance_From_Sensor[i][j+1]):
					FLS_Condition.append(i)
					FLS_Slop.append(j)

				if (Front_Sensor  >= Distance_From_Sensor[i][j]) and (Front_Sensor <= Distance_From_Sensor[i][j+1]):

					FS_Condition.append(i)
					FS_Slop.append(j)
					
				if (Front_Right_Sensor >= Distance_From_Sensor[i][j]) and (Front_Right_Sensor <= Distance_From_Sensor[i][j+1]):

					FRS_Condition.append(i)
					FRS_Slop.append(j)
#======================================================================  Debuging Step  ====================================================================================		
		
		print("============Condition N M F===================")
		print(FLS_Condition)
		print(FS_Condition)
		print(FRS_Condition)
		print("==============================================")
		
		print("=============Slop R FL FALL ==================")
		print(FLS_Slop)
		print(FS_Slop)
		print(FRS_Slop)
		print("==============================================")
		
#=============================================================================================================================================================================			
#============================================================  Calculating the Membership Values  ===========================================================================

# Using the above array created we will calculate the MF values for each sensor  -  new comment added for clarification

		FLS_Mf_Array = []
		FRS_Mf_Array = []
		FS_Mf_Array  = []
		
#=============================================================================================================================================================================		
		for i in range(len(FLS_Condition)):
			
			k = FLS_Condition[i]
			
			if FLS_Slop[i]==0:
				
				
				FLS_Mf =  (Distance_From_Sensor[k][1]  - Front_Left_Sensor)/ (Distance_From_Sensor[k][1]-Distance_From_Sensor[k][0])
				FLS_Mf_Array.append(FLS_Mf)
			

			elif FLS_Slop[i]==1:
				FLS_Mf = 1 * Front_Left_Sensor
				FLS_Mf_Array.append(FLS_Mf)

			elif FLS_Slop[i]==2:
				FLS_Mf = (Front_Left_Sensor - Distance_From_Sensor[k][2])/ (Distance_From_Sensor[k][3] - Distance_From_Sensor[k][2])
				FLS_Mf_Array.append(FLS_Mf)
#=============================================================================================================================================================================
		for i in range(len(FRS_Condition)):
			
			k = FRS_Condition[i]
			
			if FRS_Slop[i]==0:
				
				
				if Distance_From_Sensor[k][1]-Distance_From_Sensor[k][0] != 0 :
					FRS_Mf =  (Distance_From_Sensor[k][1]  - Front_Right_Sensor)/ (Distance_From_Sensor[k][1]-Distance_From_Sensor[k][0])
					FRS_Mf_Array.append(FRS_Mf)
				else:
					FRS_Mf_Array.append(FRS_Mf)
					
			elif FRS_Slop[i]==1:
				FRS_Mf = 1 * Front_Right_Sensor
				FRS_Mf_Array.append(FRS_Mf)

			elif FRS_Slop[i]==2:
			  
				FRS_Mf = (Front_Right_Sensor - Distance_From_Sensor[k][2])/ (Distance_From_Sensor[k][3] - Distance_From_Sensor[k][2])
				FRS_Mf_Array.append(FRS_Mf)

#=============================================================================================================================================================================				
		
		for i in range(len(FS_Condition)):
			
			k = FS_Condition[i]
			
			if FS_Slop[i]==0:
			
				FS_Mf =  (Distance_From_Sensor[k][1] - Front_Sensor)/ (Distance_From_Sensor[k][1]-Distance_From_Sensor[k][0])
				FS_Mf_Array.append(FS_Mf)
			

			elif FS_Slop[i]==1:
				FS_Mf = 1 * Front_Sensor
				FS_Mf_Array.append(FS_Mf)

			elif FS_Slop[i]==2:
				FS_Mf = (Front_Sensor - Distance_From_Sensor[k][2])/ (Distance_From_Sensor[k][3] - Distance_From_Sensor[k][2])
				FS_Mf_Array.append(FS_Mf)
				
#======================================================================= Debugging Step ========================================================================================				
				
		print("====================MF ARRAY===========================")
		print(FLS_Mf_Array)
		print(FS_Mf_Array)
		print(FRS_Mf_Array)
		print("=======================================================")

#=============================================================================================================================================================================
#=================================================== MF and sensor condition combination function ============================================================================

#  With the calculated we will create two 2D array with the sensor conditions and its MF values -  new comment added for clarification

		Sensor_Mf_Matrix  = []
		Rule_Set_Matrix   = []
		
		for i in range(len(FLS_Condition)):
			
			for j in range(len(FS_Condition)):

				for z in range(len(FRS_Condition)):
					Sensor_Mf_Row = []
					Rule_Set_Matrix_Row =[]

					Sensor_Mf_Row.append(FLS_Mf_Array[i])
					Sensor_Mf_Row.append(FS_Mf_Array[j])
					Sensor_Mf_Row.append(FRS_Mf_Array[z])

					Rule_Set_Matrix_Row.append(FLS_Condition[i])
					Rule_Set_Matrix_Row.append(FS_Condition[j])
					Rule_Set_Matrix_Row.append(FRS_Condition[z])
					
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

		Firing_Strength_X = 0
		Firing_Strength_Z = 0

# Checking all 27 conditions and calculating speed and steering for the robot  -  new comment added for clarification

		for i in range(len(Sensor_Mf_Min_Array)):
			
			for j in range(len(self.Rule_Set_Def)):
				if Rule_Set_Matrix[i] == self.Rule_Set_Def[j]:
					G = self.Rule_Set_Def_Out[j]
					M = G[0]
					N = G[1]
					Firing_Strength_X = Firing_Strength_X + (Speed_Centr_Array[M] * Sensor_Mf_Min_Array[i])
					Firing_Strength_Z = Firing_Strength_Z + (Direction_Centr_Array[N] * Sensor_Mf_Min_Array[i])
					
		
		X_Value = Firing_Strength_X/Sum_MF
		Z_Value = Firing_Strength_Z/Sum_MF

# This line was accidently missed in the earlier code thats the reason the robot may have not run I have mentioned this 
# to the GLA during the presentation and did took note of it. 
		
		self.cmd.linear.x  = X_Value
		self.cmd.angular.z = Z_Value		
 
				
#=======================================================================  Debugging Step ========================================================================================

		print("====================Sum MF Array===========================")

		print(X_Value)
		print(Z_Value)
		
		print("===========================================================")

#================================================================================================================================================================================
#================================================================================================================================================================================
	
	def timer_callback(self):
		self.Fuzzy_OA()
		self.publisher_.publish(self.cmd)
		#self.get_logger().info(FLS_Mf_Array)
		#self.get_logger().info(FRS_Mf_Array)
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
