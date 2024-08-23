#!/usr/bin/env python3
import message_filters
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class CLASMovingBaseCombiner:
    def __init__(self):
        self.sub1 = message_filters.Subscriber("/odom/gps", Odometry)
        self.sub2 = message_filters.Subscriber("movingbase/quat", Imu)#sub3.sub4--- ok
        self.mf = message_filters.ApproximateTimeSynchronizer([self.sub1, self.sub2],10,0.5)#0.5 ok
        self.mf.registerCallback(self.callback)
        
        self.odom_pub = rospy.Publisher("/CLAS_movingbase", Odometry, queue_size=10)
        self.odom_msg = Odometry()
        
        self.CLAS_position = None        
        self.movingbase_yaw = None   
        
    def callback(self,msg1,msg2):
        self.CLAS_position = msg1
        self.movingbase_yaw = msg2
    
    def publish_combined_odom(self):
        if self.CLAS_position is not None and self.movingbase_yaw is not None:
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = self.CLAS_position.pose.pose.position.x
            self.odom_msg.pose.pose.position.y = self.CLAS_position.pose.pose.position.y
            self.odom_msg.pose.pose.position.z = self.CLAS_position.pose.pose.orientation.w##Number of satellites
            self.odom_msg.pose.pose.orientation.x = 0
            self.odom_msg.pose.pose.orientation.y = 0
            self.odom_msg.pose.pose.orientation.z = self.movingbase_yaw.orientation.z
            self.odom_msg.pose.pose.orientation.w = self.movingbase_yaw.orientation.w
            self.odom_msg.pose.covariance[0] = 0.0001
            self.odom_msg.pose.covariance[7] = 0.0001
            self.odom_msg.pose.covariance[14] = 0.0001#error
            self.odom_msg.pose.covariance[21] = 0.0001#error
            self.odom_msg.pose.covariance[28] = 0.0001#error
            self.odom_msg.pose.covariance[35] = 0.0001
            self.odom_pub.publish(self.odom_msg)
        else :
            rospy.logwarn("Data missing: CLAS_position or movingbase_yaw is None")
            
if __name__ == "__main__":
    # init node
    rospy.init_node('clas_moving_base_combiner')
    rate = rospy.Rate(1)
    combiner = CLASMovingBaseCombiner()
    while not rospy.is_shutdown():
        combiner.publish_combined_odom()
        rate.sleep()
