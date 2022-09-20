#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math 
import numpy as np


class OdomCalculate():
	def __init__(self):
	
		rospy.init_node('odometry',anonymous = True)
		self.rover_width = 890.0 / 10	 						#width of the rover
		self.rover_length = 838.66 / 10	 						#length of the rover
		self.wheel_radius = 135.00 / 10  						#Radius of the wheels
		self.wheel_speed_const = self.wheel_radius / 60			#A constant value that will help us RPM to Speed.
		self.init()
		self.run()
	
	def init(self):
		#Initially everything is zero.
		self.x = 0 
		self.y = 0 
		self.theta = 0
		self.last_left_wheel_speed = 0
		self.last_right_wheel_speed = 0
		self.time_old = rospy.Time.now()
		self.time_new = rospy.Time.now()
		
		
		self.pub = rospy.Publisher("odom",Odometry,queue_size=10) #Odometry Publisher.
		self.broadcaster = tf.TransformBroadcaster()			  #Broadcaster

		
	
	
	def calculate(self,data):
		#When a new RPM data arrives, getting the time.
		self.time_old = self.time_new
		self.time_new = rospy.Time.now()
		
		
		#The time passed between 2 RPM data's (delta t)
		time = (self.time_new - self.time_old).to_sec()
		
		
		#Getting RPM datas. The average values of the wheels are used for both left and right wheels.
		RPM_data = data.data
		left_wheel_RPM = (RPM_data[0] + RPM_data[1]) / 2 
		right_wheel_RPM = (RPM_data[2] + RPM_data[3]) / 2 
		
		
		
		#Calculating the speeds.
		left_wheel_speed = left_wheel_RPM * self.wheel_speed_const 
		right_wheel_speed = right_wheel_RPM * self.wheel_speed_const 
		
		
		#Finding the average speed. 
		average_left_wheel_speed = ( self.last_left_wheel_speed + left_wheel_speed ) / 2 
		average_right_wheel_speed = ( self.last_right_wheel_speed + right_wheel_speed ) / 2
		
		
		
		
		
		
		

		
		
		
		#If speed of the left wheel and right wheel are equal, then there won't be any changes in the direction of the vehicle.
		if average_left_wheel_speed == average_right_wheel_speed:
			w = 0 													#No angular speed. 
			rover_speed = average_left_wheel_speed							
			rover_speed_x = rover_speed * math.sin(self.theta)		#Calculating the speed about x axis and y axis.
			rover_speed_y = rover_speed * math.cos(self.theta)
			
			
			#Finding the new x and y coordinates
			new_x = self.x + rover_speed_x*time 
			new_y = self.y + rover_speed_y*time
			new_theta = self.theta
			

		else:	
		
			#The definitions of R,w,ICC,rotation matrices are coming from website 
			#'https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf'
			#Please check.
		
		
			R  = (self.rover_width / 2) * ((average_left_wheel_speed + average_right_wheel_speed) / (average_right_wheel_speed - average_left_wheel_speed))
			w  = (average_right_wheel_speed-average_left_wheel_speed) / self.rover_width
			 
			
			#ICC = x-Rsin(theta), y+Rcos(theta)
			ICC = [self.x - R*math.sin(self.theta), self.y+R*math.cos(self.theta)]
			#print(self.x, self.y , R, w , ICC)
			
			
			#Rotation by wdt about Z axis matrix.
			rotation_about_z_axis_matrix = np.array([[math.cos(w*time),-1*math.sin(w*time),0],[math.sin(w*time),math.cos(w*time),0],[0,0,1]])
			
			#translate icc to origin matrix.
			translate_icc2origin_matrix  = np.array([self.x-ICC[0],self.y-ICC[1],self.theta])
			
			#translate icc back to original from origin.
			translate_icc2back_matrix = np.array([ICC[0],ICC[1],w*time])
			
			#new x,y and z(theta) values.
			transformed_xyz = np.matmul(rotation_about_z_axis_matrix,translate_icc2origin_matrix) + translate_icc2back_matrix
			
			#setting them.
			new_x,new_y,new_theta = transformed_xyz[0],transformed_xyz[1],transformed_xyz[2]
			
			
			#Speeds about x and y axis of the complete vehicle.
			rover_speed = (average_left_wheel_speed + average_right_wheel_speed) / 2
			rover_speed_x = rover_speed * math.sin(self.theta)
			rover_speed_y = rover_speed * math.cos(self.theta)
			
			
			
			
			
			
			
			
		#Setting the new values.
		self.x = new_x 
		self.y = new_y 
		self.theta = new_theta
		self.last_left_wheel_speed = left_wheel_speed 
		self.last_right_wheel_speed = right_wheel_speed
		
		
		#Transform
		odom_quat = tf.transformations.quaternion_from_euler(0,0,self.theta)
		self.broadcaster.sendTransform((self.x/100,self.y/100,0.),odom_quat,self.time_new,"base_link","odom")
		
		#Odom msg.
		odom = Odometry()
		odom.header.stamp = self.time_new
		odom.header.frame_id = "odom"
		
		odom.pose.pose = Pose(Point(self.x/100 ,self.y/100,0.), Quaternion(*odom_quat))
		
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(rover_speed_x/100 ,rover_speed_y/100 ,0.),Vector3(0,0,w))

		#Publishing the odonm.
		self.publish_odom(odom)
		
		
		
		
	
		
		
	def publish_odom(self,odom):
		
		self.pub.publish(odom)
		
		
	def run(self):
		rospy.Subscriber("/drive_system_all_motors_feedbacks",Float64MultiArray,self.calculate)
		rospy.spin()

if __name__ == '__main__':
	OdomCalculate()
