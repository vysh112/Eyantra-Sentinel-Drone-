#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
import rospy

from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Edrone():
	"""docstring for Edrone"""
	def callback(self,data):
		br=CvBridge()
		kernel=np.ones((5,5),np.uint8)

		image=br.imgmsg_to_cv2(data)


	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	


		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,23]
		self.setpoint1=[-5,-5,23] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint2=[2,2,23] 
		self.setpoint3=[-2,2,23] 
		self.setpoint4=[-2,-2,23]
		self.setpoint5=[2,-2,23] 
		self.setpoint6=[2,0,23] 

		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [30,30,40]
		self.Ki = [0,0,0.00018]
		self.Kd = [15000,15000,15000]


		#-----------------------Add other required variables for pid here ----------------------------------------------


		self.sample_time = 60
		self.prev_values = [0,0,0]
		self.max_values = 2000
		self.min_values = 1000
		self.error = [0,0,0]
		self.now=0.0000
		self.timechange=0.000
		self.errsum=[0,0,0]
		self.derr=[0,0,0]
		self.last_time=0.0000
		
		self.out_roll=0.000
		self.out_pitch=0.000
		self.out_throttle=0.000

		self.sample_time1 = 60
		self.prev_values1 = [0,0,0]
		self.max_values1 = 2000
		self.min_values1= 1000
		self.error1 = [0,0,0]
		self.now1=0.0000
		self.timechange1=0.000
		self.errsum1=[0,0,0]
		self.derr1=[0,0,0]
		self.last_time1=0.0000
		
		self.out_roll1=0.000
		self.out_pitch1=0.000
		self.out_throttle1=0.000

		self.sample_time2 = 60
		self.prev_values2 = [0,0,0]
		self.max_values2 = 2000
		self.min_values2= 2000
		self.error2 = [0,0,0]
		self.now2=0.0000
		self.timechange2=0.000
		self.errsum2=[0,0,0]
		self.derr2=[0,0,0]
		self.last_time2=0.0000
		
		self.out_roll2=0.000
		self.out_pitch2=0.000
		self.out_throttle2=0.000

		self.sample_time3 = 60
		self.prev_values3 = [0,0,0]
		self.max_values3 = 3000
		self.min_values3= 3000
		self.error3 = [0,0,0]
		self.now3=0.0000
		self.timechange3=0.000
		self.errsum3=[0,0,0]
		self.derr3=[0,0,0]
		self.last_time3=0.0000
		
		self.out_roll3=0.000
		self.out_pitch3=0.000
		self.out_throttle3=0.000
    
		self.sample_time4 = 60
		self.prev_values4 = [0,0,0]
		self.max_values4 = 4000
		self.min_values4= 4000
		self.error4 = [0,0,0]
		self.now4=0.0000
		self.timechange4=0.000
		self.errsum4=[0,0,0]
		self.derr4=[0,0,0]
		self.last_time4=0.0000
		
		self.out_roll4=0.000
		self.out_pitch4=0.000
		self.out_throttle4=0.000

		self.sample_time5 = 60
		self.prev_values5 = [0,0,0]
		self.max_values5 = 5000
		self.min_values5= 5000
		self.error5 = [0,0,0]
		self.now5=0.0000
		self.timechange5=0.000
		self.errsum5=[0,0,0]
		self.derr5=[0,0,0]
		self.last_time5=0.0000
		
		self.out_roll5=0.000
		self.out_pitch5=0.000
		self.out_throttle5=0.000

		self.sample_time6 = 60
		self.prev_values6 = [0,0,0]
		self.max_values6 = 6000
		self.min_values6= 6000
		self.error6 = [0,0,0]
		self.now6=0.0000
		self.timechange6=0.000
		self.errsum6=[0,0,0]
		self.derr6=[0,0,0]
		self.last_time6=0.0000
		
		self.out_roll6=0.000
		self.out_pitch6=0.000
		self.out_throttle6=0.000



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.altError = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitchError = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.rollError = rospy.Publisher('/roll_error', Float64, queue_size=1)




		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber("/e_drone/camera_rgb/image_raw",Image, self.callback)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------


	def pitch_set_pid(self,pitch):
		self.Kp[0] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = pitch.Ki * 0.008
		self.Kd[0] = pitch.Kd * 0.3


	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = roll.Ki * 0.008
		self.Kd[1] = roll.Kd * 0.3


	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum


		self.now = int(round(time.time() * 1000))
		self.timechange=self.now-self.last_time
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if (self.timechange>self.sample_time):
			if (self.last_time!=0):			

				#Getting error of all coordinates		
				self.error[0]=self.drone_position[0] - self.setpoint[0]
				self.error[1]=self.drone_position[1] - self.setpoint[1]
				self.error[2]=self.drone_position[2] - self.setpoint[2]
				

				#Integration for Ki
				#self.errsum[0]=self.errsum[0]+(self.error[0]*self.timechange)
				#self.errsum[1]=self.errsum[1]+(self.error[1]*self.timechange)
				self.errsum[2]=self.errsum[2]+(self.error[2]*self.timechange)


				#Derivation for Kd
				self.derr[0]=(self.error[0]-self.prev_values[0])/self.timechange
				self.derr[1]=(self.error[1]-self.prev_values[1])/self.timechange
				self.derr[2]=(self.error[2]-self.prev_values[2])/self.timechange


				#Calculating output in 1500
				self.cmd.rcRoll=int(1500-(self.Kp[0]*self.error[0])-(self.Kd[0]*self.derr[0]))
				self.cmd.rcPitch=int(1500+(self.Kp[1]*self.error[1])+(self.Kd[1]*self.derr[1]))
				self.cmd.rcThrottle=int(1500+(self.Kp[2]*self.error[2])+(self.Kd[2]*self.derr[2])-(self.errsum[2]*self.Ki[2]))

				
				#Checking min and max threshold and updating on true
				#Throttle Conditions
				if self.cmd.rcThrottle>2000:
					self.cmd.rcThrottle=self.max_values
				if self.cmd.rcThrottle<1000:
					self.cmd.rcThrottle=self.min_values		

				#Pitch Conditions
				if self.cmd.rcPitch>2000:
					self.cmd.rcPitch=self.max_values	
				if self.cmd.rcPitch<1000:
					self.cmd.rcPitch=self.min_values

				#Roll Conditions
				if self.cmd.rcRoll>2000:
					self.cmd.rcRoll=self.max_values
				if self.cmd.rcRoll<1000:
					self.cmd.rcRoll=self.min_values


				#Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)

				
				#Updating prev values for all axis
				self.prev_values[0]=self.error[0]
				self.prev_values[1]=self.error[1]
				self.prev_values[2]=self.error[2]
		

		 		
		 	#Updating last time value	
			self.last_time=self.now
	

			#Getting values for Plotjuggler
			self.rollError.publish(self.error[0])
			self.pitchError.publish(self.error[1])
			self.altError.publish(self.error[2])

			print("0 Workingggggggggggggggggg")




		


	#------------------------------------------------------------------------------------------------------------------------


	def pid1(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum


		self.now1= int(round(time.time() * 1000))
		self.timechange1=self.now1-self.last_time1
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if (self.timechange1>self.sample_time1):
			if (self.last_time1!=0):			

				#Getting error of all coordinates		
				self.error[0]=self.drone_position[0] - self.setpoint1[0]
				self.error[1]=self.drone_position[1] - self.setpoint1[1]
				self.error[2]=self.drone_position[2] - self.setpoint1[2]
				

				#Integration for Ki
				#self.errsum[0]=self.errsum[0]+(self.error[0]*self.timechange)
				#self.errsum[1]=self.errsum[1]+(self.error[1]*self.timechange)
				self.errsum1[2]=self.errsum1[2]+(self.error[2]*self.timechange1)


				#Derivation for Kd
				self.derr1[0]=(self.error[0]-self.prev_values1[0])/self.timechange1
				self.derr1[1]=(self.error[1]-self.prev_values1[1])/self.timechange1
				self.derr1[2]=(self.error[2]-self.prev_values1[2])/self.timechange1


				#Calculating output in 1500
				self.cmd.rcRoll=int(1500-(self.Kp[0]*self.error[0])-(self.Kd[0]*self.derr1[0]))
				self.cmd.rcPitch=int(1500+(self.Kp[1]*self.error[1])+(self.Kd[1]*self.derr1[1]))
				self.cmd.rcThrottle=int(1500+(self.Kp[2]*self.error[2])+(self.Kd[2]*self.derr1[2])-(self.errsum1[2]*self.Ki[2]))

				
				#Checking min and max threshold and updating on true
				#Throttle Conditions
				if self.cmd.rcThrottle>2000:
					self.cmd.rcThrottle=self.max_values
				if self.cmd.rcThrottle<1000:
					self.cmd.rcThrottle=self.min_values		

				#Pitch Conditions
				if self.cmd.rcPitch>2000:
					self.cmd.rcPitch=self.max_values	
				if self.cmd.rcPitch<1000:
					self.cmd.rcPitch=self.min_values

				#Roll Conditions
				if self.cmd.rcRoll>2000:
					self.cmd.rcRoll=self.max_values
				if self.cmd.rcRoll<1000:
					self.cmd.rcRoll=self.min_values


				#Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)

				
				#Updating prev values for all axis
				self.prev_values1[0]=self.error[0]
				self.prev_values1[1]=self.error[1]
				self.prev_values1[2]=self.error[2]
		

		 		
		 	#Updating last time value	
			self.last_time1=self.now1
	

			#Getting values for Plotjuggler
			self.rollError.publish(self.error[0])
			self.pitchError.publish(self.error[1])
			self.altError.publish(self.error[2])
			print("1 Workingggggggggggggggggg")


	#------------------------------------------------------------------------------------------------------------------------

	#------------------------------------------------------------------------------------------------------------------------

	


if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(29) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		t_end = time.time() + 15

		while time.time() < t_end:

			e_drone.pid()
		t_end1 = time.time() + 20
		while time.time() < t_end1:

			e_drone.pid1()
		t_end2 = time.time() + 27


		r.sleep()