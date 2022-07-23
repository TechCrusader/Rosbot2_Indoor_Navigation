
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

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



		
#==============================================================================================================================================================================
#=======================================================================Rule Base Right Edge ===================================================================================


		self.RE_Rule_Set_Def = []
		for i in range(3):
			for j in range(3):
				Rule_Set_Row = []
				Rule_Set_Row.append(i)
				Rule_Set_Row.append(j)
				self.RE_Rule_Set_Def.append(Rule_Set_Row)
					
		self.RE_Rule_Set_Def_Out = [[Medium,Left],[Medium,Left],[Slow,Left],[Slow,Left],[Fast,Zero],[Slow,Right],[Slow,Right],[Medium,Right],[Fast,Right]]


#==============================================================================================================================================================================
#===============================================================Rule Base  Obstacle Avoidance =================================================================================
		
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











#===============================================================================================================================================================================
#=================================================================== Cbstacle Avoidance=========================================================================================
#===============================================================================================================================================================================


	def Fuzzy_OA(self):
	
	
	
		Distance_From_Sensor  = [[ 0.00,0.00 ,0.25 , 0.50],[ 0.25,0.50,0.50,0.75],[0.50,0.75,4.00,4.00]]
		Direction             = [[-0.75,-0.50,-0.50,-0.25],[-0.25,0.00,0.00,0.25],[0.25,0.50,0.50,0.75]]
		Speed                 = [[0.00 ,0.10 ,0.10 ,0.20 ],[0.20 ,0.30,0.30,0.40],[0.40,0.50,0.50,0.60]]
		Direction_Centr_Array = [-0.50,0.00,0.50]
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
		out = [X_Value,Z_Value]
		return out

#===============================================================================================================================================================================
#===============================================================================================================================================================================
#===============================================================================================================================================================================








#===============================================================================================================================================================================
#=================================================================== Right Edge Following ======================================================================================
#===============================================================================================================================================================================

	def Fuzzy_RE(self):
		Distance_From_Sensor = [[ 0.00,0.00 ,0.25 , 0.50],[ 0.25,0.50,0.50,0.75],[0.50,0.75,4.00,4.00]]
		Direction = [[-0.75,-0.50,-0.50,-0.25],[-0.25,0.00,0.00,0.25],[0.25,0.50,0.50,0.75]]
		Speed = [[0.00,0.10,0.10,0.20],[0.20,0.30,0.30,0.40],[0.40,0.50,0.50,0.60]]
		Direction_Centr_Array = [-0.70,0.00,0.70]
		Speed_Centr_Array = [0.2,0.35,0.50]
	
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
		
		out = [X_Value,Z_Value]
		
		return out
		
				
#=======================================================================  Debugging Step ========================================================================================

		print("====================Ouput Values===========================")

		print(X_Value)
		print(Z_Value)
		
		print("===========================================================")

#================================================================================================================================================================================
#================================================================================================================================================================================


#===============================================================================================================================================================================
#===============================================================================================================================================================================
#===============================================================================================================================================================================









#===============================================================================================================================================================================
#===================================================================Combination RE & Obstacle ==================================================================================
#===============================================================================================================================================================================


	def Fuzzy_OA_RE(self):

		Oba_Avoidance = 0
		Right_Edge    = 1

		Distance_From_Sensor = [[ 0.00,0.00 ,0.25 , 0.50],[ 0.25,0.50,2.00,3.00]]
		
#=================================================================== Reading Sensor Values ===================================================================================
#===================================================================   Front Sensor   ========================================================================================
	
		Front_Sensor_A	    = [min(self.laser_ranges[ 0:180]),min(self.laser_ranges[ 1260:1439])]
		
		Front_Sensor       = min(Front_Sensor_A)
		
		if Front_Sensor > 2:
			Front_Sensor =1.9
#===================================================================    Debuging Step    =====================================================================================
			
		print("=================Sensor Val====================")

		print(Front_Sensor)
		
		print("==============================================")

	
#=============================================================================================================================================================================
#============================================================== Checking Sensor Conditions  ==================================================================================

		OA_RE_Condition = []
		
		OA_RE_Slop = []

		for i in range(len(Distance_From_Sensor)):
			for j in range(len(Distance_From_Sensor[0])-1):
			
				if (Front_Sensor > Distance_From_Sensor[i][j]) and (Front_Sensor < Distance_From_Sensor[i][j+1]):
				
					OA_RE_Condition.append(i)
					OA_RE_Slop.append(j)
				
#======================================================================  Debuging Step  ====================================================================================		
		
		print("============Condition N M F===================")
		
		print(OA_RE_Condition)
				
		print("==============================================")
		
		print("=============Slop R FL FALL ==================")
		
		print(OA_RE_Slop)
		
		print("==============================================")

#=============================================================================================================================================================================
#============================================================  Calculating the Membership Values  ===========================================================================

		
		OA_RE_Mf_Array = []

#=============================================================================================================================================================================				

		for i in range(len(OA_RE_Condition)):
			
			k = OA_RE_Condition[i]
			
			# Rising Edge
			if OA_RE_Slop[i]==0:
				
				OA_RE_Mf =  (Distance_From_Sensor[k][1]  - Front_Sensor)/ (Distance_From_Sensor[k][1]-Distance_From_Sensor[k][0])
				OA_RE_Mf_Array.append(OA_RE_Mf)
			
			# Flat
			elif OA_RE_Slop[i]==1:
				OA_RE_Mf = 1 * Front_Sensor
				OA_RE_Mf_Array.append(OA_RE_Mf)
			
			# Falling Edge
			elif OA_RE_Slop[i]==2:
				OA_RE = (Front_Sensor - Distance_From_Sensor[k][2])/ (Distance_From_Sensor[k][3] - Distance_From_Sensor[k][2])
				OA_RE_Mf_Array.append(OA_RE_Mf)
		


		Sensor_Mf_Array = OA_RE_Mf_Array
		

#======================================================================= Debugging Step ========================================================================================
				
		print("====================MF ARRAY===========================")
		
		print(OA_RE_Mf_Array)
		print(Sensor_Mf_Array)

		print("=======================================================")

#===============================================================================================================================================================================
#=============================================================== Fetching Output from OA and RE ================================================================================

		FR_RE = self.Fuzzy_RE()
		FR_OA = self.Fuzzy_OA()

#======================================================================= Debugging Step ========================================================================================
				
		print("===============Output from OA and RE===================")
		
		print(FR_OA)
		print(FR_RE)
		

		print("=======================================================")

#===============================================================================================================================================================================
#==========================================================================Sum of all the MF's====================================================================================

		Sum_MF           = 0 
		
		for i in range(len(Sensor_Mf_Array)):

			Sum_MF = Sum_MF + Sensor_Mf_Array[i]

#=======================================================================  Debugging Step ========================================================================================

		print("====================Sum MF Array===========================")

		print(Sum_MF)

		print("===========================================================")

#===============================================================================================================================================================================
#=====================================================================  Calculating Firing Strength =============================================================================

		Firing_Strength_X = 0
		Firing_Strength_Z = 0
		
		for i in range(len(Sensor_Mf_Array)):

			if OA_RE_Condition == 0:
				
				Firing_Strength_X = Firing_Strength_X + (FR_OA[0] *	Sensor_Mf_Array)
				Firing_Strength_Z = Firing_Strength_Z + (FR_OA[1] * Sensor_Mf_Array)

			if OA_RE_Condition == 1:
				
				Firing_Strength_X = Firing_Strength_X + (FR_RE[0] *	Sensor_Mf_Array)
				Firing_Strength_Z = Firing_Strength_Z + (FR_RE[1] * Sensor_Mf_Array)


		X_Value = Firing_Strength_X/Sum_MF
		Z_Value = Firing_Strength_Z/Sum_MF
		
		self.cmd.linear.x  = X_Value
		self.cmd.angular.z = Z_Value
#=======================================================================  Debugging Step ========================================================================================

		print("=============Combined Output X and Z=======================")

		print(X_Value)
		print(Z_Value)
		
		print("===========================================================")

#================================================================================================================================================================================
#================================================================================================================================================================================





	def timer_callback(self):
		self.Fuzzy_OA_RE()
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
