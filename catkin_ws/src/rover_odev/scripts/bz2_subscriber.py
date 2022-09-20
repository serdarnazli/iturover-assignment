#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Char
import bz2
import binascii
#Author: M.Serdar NAZLI

class msg_class():
    def __init__(self):
        self.init()
        self.run()


    def init(self):
        rospy.init_node('bz2_solution_node',anonymous=True)
        rospy.loginfo("BZ2 Solution node started!")
        
        #Publisher to /solution
        self.pub = rospy.Publisher("/solution",Char,queue_size=10)
        
        #Subscriber to /bz2_message
        rospy.Subscriber("/bz2_message",String,self.callback)


    def publish_frequent(self):
        self.pubm = Char()														#Creating the message to publish.
        self.pubm.data = self.frequent_char										#Setting up the message data
        rospy.loginfo("Frequent char:  %s has send. ",self.frequent_char)		
        self.pub.publish(self.pubm)												#Publishing the message.



	#Callback function for bz2_message topic.
    def callback(self,compressed_ascii):
    	rospy.loginfo("message received!")
        compressed_ascii = compressed_ascii.data									#Getting the data.
        uncompressed_ascii = binascii.unhexlify(compressed_ascii.encode('ascii'))	#Uncompressing.
        self.msg = bz2.decompress(uncompressed_ascii).decode('ascii') 				#Decoding.
        self.frequent_char = self.findFrequent(self.msg)							#Finding the frequent.
        self.publish_frequent()														#Publishing the frequent.

    def findFrequent(self,msg):
        letters = list(msg)															#Creating a list for the letters.
        return str(max(set(letters),key=letters.count))								#Finding the most frequent char.

    def run(self):
        rospy.spin()



if __name__ == '__main__':
    try:
        msg_class()
    
    except rospy.ROSInterruptException:
        pass


    
