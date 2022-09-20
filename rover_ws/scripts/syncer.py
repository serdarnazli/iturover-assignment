#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import message_filters

#This is an message synchronizer class. From two different topics into one topic.


class Msg_synchronizer():
	def __init__(self):
		rospy.init_node('syncer',anonymous=True)
		self.init()
		self.run()
	
	def init(self):
		self.available_to_pub = False	#Available to publish the message or not.
		self.left_count = 0				#how many messages received from the left wheel encoder
		self.right_count = 0			#how many messages received right wheel encoder.
		self.left_feedback = None		#Left wheel encoder message
		self.right_feedback = None		#Right wheel encoder message
		
		
		#Publisher
		self.pub = rospy.Publisher("/drive_system_all_motors_feedbacks",Float64MultiArray,queue_size=10)


		#Subscribers
		rospy.Subscriber("/drive_system_left_motors_feedbacks",Float64MultiArray,self.update_left)
		rospy.Subscriber("/drive_system_right_motors_feedbacks",Float64MultiArray,self.update_right)
		
		
		
	def update_left(self,data):
		self.left_feedback = data	#Get the data.
		self.left_count += 1		#Increase the left count
		self.set_availability()		#Set the availability
		self.try_publish()			#Try to publish.
		
		
	def update_right(self,data):
		self.right_feedback = data
		self.right_count += 1
		self.set_availability()
		self.try_publish()
		
		
		
	def set_availability(self):
		if self.left_count == self.right_count: #If left and right count are equal to each other, it is available to publish.
			self.available_to_pub = True
	
		
	
	def try_publish(self):
		if not self.available_to_pub:
			return
			
		#Setting up the message
		msg = Float64MultiArray()
		msg.data = [self.left_feedback.data[0],self.left_feedback.data[1],self.right_feedback.data[0],self.right_feedback.data[1]]
		
		#Publishing it.
		self.pub.publish(msg)
		print("msg : ", msg , "Published!")
		
		#Not available to publish anymore.
		self.available_to_pub = False
	
	
	def run(self):
		rospy.spin()
		
			


if __name__ == '__main__':
	try:
	    Msg_synchronizer()
	except rospy.ROSInterruptException:
		pass
   
