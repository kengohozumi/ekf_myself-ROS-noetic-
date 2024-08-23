#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
#from sensor_msgs.msg import Imu

class GPSDataToodom:
    def __init__(self):
        self.fix = rospy.Subscriber("fix", NavSatFix, self.fix_callback)
#        rospy.Subscriber('/movingbase_yaw', Imu, self.sensor_b_callback)

        self.odom_pub = rospy.Publisher("/odom/gps", Odometry, queue_size=10)
        self.odom_msg = Odometry()

        self.theta = rospy.get_param(
            "~heading", 180
        )  # initial heading azithm nakaniwa 179.169287 tsukuba 291.09504
        
        self.initial_coordinate = None
        self.fix_data = None
        
        self.count = 0

    def fix_callback(self, data):
        self.fix_data = data

#    def sensor_b_callback(self, data):
#        if self.count == 0:
#            self.theta = data.orientation.x
#            self.count = 1

    def conversion(self, coordinate, origin, theta):

        # set up constants
        a = 6378137
        f = 35 / 10439
        e1 = 734 / 8971
        e2 = 127 / 1547
        n = 35 / 20843
        a0 = 1
        a2 = 102 / 40495
        a4 = 1 / 378280
        a6 = 1 / 289634371
        a8 = 1 / 204422462123
        degree_to_radian = math.pi / 180

        # calculation below
        delta_latitude = coordinate[0] - origin[0]
        delta_longitude = coordinate[1] - origin[1]

        r_latitude = coordinate[0] * degree_to_radian
        r_longitude = coordinate[1] * degree_to_radian
        r_latitude_origin = origin[0] * degree_to_radian
        r_longitude_origin = origin[1] * degree_to_radian
        r_delta_latitude = delta_latitude * degree_to_radian
        r_delta_longitude = delta_longitude * degree_to_radian
        W = math.sqrt(1 - (e1 ** 2) * (math.sin(r_latitude) ** 2))
        N = a / W
        t = math.tan(r_latitude)
        ai = e2 * math.cos(r_latitude)

        S = (
            a
            * (
                a0 * r_latitude
                - a2 * math.sin(2 * r_latitude)
                + a4 * math.sin(4 * r_latitude)
                - a6 * math.sin(6 * r_latitude)
                + a8 * math.sin(8 * r_latitude)
            )
            / (1 + n)
        )
        if S != 0:
            S0 = (
                a
                * (
                    a0 * r_latitude_origin
                    - a2 * math.sin(2 * r_latitude_origin)
                    + a4 * math.sin(4 * r_latitude_origin)
                    - a6 * math.sin(6 * r_latitude_origin)
                    + a8 * math.sin(8 * r_latitude_origin)
                )
                / (1 + n)
            )
        m0 = S / S0
        B = S - S0
        y1 = (
            (r_delta_longitude ** 2)
            * N
            * math.sin(r_latitude)
            * math.cos(r_latitude)
            / 2
        )
        y2 = (
            (r_delta_longitude ** 4)
            * N
            * math.sin(r_latitude)
            * (math.cos(r_latitude) ** 3)
            * (5 - (t ** 2) + 9 * (ai ** 2) + 4 * (ai ** 4))
            / 24
        )
        y3 = (
            (r_delta_longitude ** 6)
            * N
            * math.sin(r_latitude)
            * (math.cos(r_latitude) ** 5)
            * (
                61
                - 58 * (t ** 2)
                + (t ** 4)
                + 270 * (ai ** 2)
                - 330 * (ai ** 2) * (t ** 2)
            )
            / 720
        )
        y = m0 * (B + y1 + y2 + y3)
        # rospy.logdebug("y: %f", y)

        x1 = r_delta_longitude * N * math.cos(r_latitude)
        x2 = (
            (r_delta_longitude ** 3)
            * N
            * (math.cos(r_latitude) ** 3)
            * (1 - (t ** 2) + (ai ** 2))
            / 6
        )
        x3 = (
            (r_delta_longitude ** 5)
            * N
            * (math.cos(r_latitude) ** 5)
            * (
                5
                - 18 * (t ** 2)
                + (t ** 4)
                + 14 * (ai ** 2)
                - 58 * (ai ** 2) * (t ** 2)
            )
            / 120
        )
        x = m0 * (x1 + x2 + x3)
        # rospy.logdebug("x: %f", x)

        r_theta = theta * degree_to_radian
        h_x = math.cos(r_theta) * x - math.sin(r_theta) * y
        h_y = math.sin(r_theta) * x + math.cos(r_theta) * y
        point = (h_y, -h_x)
        return point

    def publish_GPSodom(self):
        if self.fix_data is not None:
            if self.initial_coordinate is None:
                self.initial_coordinate = [self.fix_data.latitude, self.fix_data.longitude]
            latlon = [self.fix_data.latitude,self.fix_data.longitude]
            GPSxy = self.conversion(latlon, self.initial_coordinate, self.theta)
            rospy.loginfo(GPSxy)
            rospy.logdebug("GPSxy: (%f, %f)", GPSxy[0], GPSxy[1])
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            self.odom_msg.pose.pose.position.x = GPSxy[0]
            self.odom_msg.pose.pose.position.y = GPSxy[1]
            self.odom_msg.pose.pose.position.z = 0
            self.odom_msg.pose.pose.orientation.x = 0
            self.odom_msg.pose.pose.orientation.y = 0
            self.odom_msg.pose.pose.orientation.z = 0
            self.odom_msg.pose.pose.orientation.w = self.fix_data.position_covariance[0]#Number of satellites
            self.odom_msg.pose.covariance = [0.0001, 0, 0, 0, 0, 0, 
                                             0, 0.0001, 0, 0, 0, 0, 
                                             0, 0, 0.000001, 0, 0, 0, 
                                             0, 0, 0, 0.000001, 0, 0, 
                                             0, 0, 0, 0, 0.000001, 0, 
                                             0, 0, 0, 0, 0, 0.0001]
            # status.status = 2 is an analytical Fix solution based on the reference station. fix 2 nonfix -1
            #if self.fix_data.status.status == 2:
            self.odom_pub.publish(self.odom_msg)
        else :
            rospy.loginfo("fix_data non")
        
if __name__ == "__main__":
    # init node
    rospy.init_node("gps_data_acquisition")
    rate = rospy.Rate(3)#10
    gtodom = GPSDataToodom()
    while not rospy.is_shutdown():
        gtodom.publish_GPSodom()
        rate.sleep()
