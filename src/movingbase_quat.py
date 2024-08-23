#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import tf
import math

class movingbaseNode:
    def __init__(self):
        # Initialize message and publisher
        self.sub_movingbase  = rospy.Subscriber('movingbase_yaw',Imu,self.movingbase_callback) 
        self.movingbase_msg = Imu()
        self.heading_pub = rospy.Publisher('movingbase/quat', Imu, queue_size=10)
        self.movingbase_data = None
        
    def movingbase_callback(self,data):
        self.movingbase_data = data
        
    def movingbase_publish_msg(self):
         if self.movingbase_data is not None:
             roll, pitch = 0, 0
             yaw = self.movingbase_data.orientation.z
             
             q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
             rospy.loginfo(q)        
             self.movingbase_msg.header.stamp = rospy.Time.now()
             self.movingbase_msg.header.frame_id = "imu_link"
             self.movingbase_msg.orientation.x = q[0]
             self.movingbase_msg.orientation.y = q[1]
             self.movingbase_msg.orientation.z = -q[2]
             self.movingbase_msg.orientation.w = q[3]
             self.heading_pub.publish(self.movingbase_msg)
             self.movingbase_data = None
         else :
             rospy.loginfo("movingbase_data non")       
        
if  __name__ == "__main__":
    # init node
    rospy.init_node("movingbase")
    rate = rospy.Rate(3)
    movingbase_node = movingbaseNode()
    while not rospy.is_shutdown():
        movingbase_node.movingbase_publish_msg()
        rate.sleep()
