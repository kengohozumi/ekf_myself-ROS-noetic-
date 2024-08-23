#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class ExtendedKalmanFilter:
    def __init__(self):       
        self.GTheta = None 
        self.GTheta0 = None           
        self.GPSthetayaw0 = 0
        self.DGPStheta = 0
        self.w = None
        self.Q = None # Process noise covariance
        self.H = None
        self.R = None
        self.R1 = 0  # High frequency sensor noise covariance
        self.R2 = 0  # Low frequency sensor noise covariance
        self.R3 = 0  # High frequency sensor(heading) 
        self.R4 = 0  # Low frequency sensor(heading)
        # Initial covariance
        self.P = None
        self.XX = None
        self.prev_time = None
        self.prev_pos = None
        self.Speed = 0
        self.SmpTime = 0.1
        self.GpsXY = None    
        self.conut = 0   
        self.GPS_conut = 0   
        self.GOffset  = 0
        self.offsetyaw = 0
        self.combineyaw = 0
        self.robot_yaw = 0
        self.combyaw = 0
        self.robot_orientationz = 0
        self.robot_orientationw = 0
        self.Number_of_satellites = 0

        rospy.Subscriber('/combine_dr_measurements/odom_combined', PoseWithCovarianceStamped, self.sensor_a_callback)
        rospy.Subscriber('/CLAS_movingbase', Odometry, self.sensor_b_callback)

        self.fused_pub = rospy.Publisher('/kf_myself', PoseWithCovarianceStamped, queue_size=10)
        self.fused_data = PoseWithCovarianceStamped()


    def orientation_to_yaw(self, z, w):
        yaw = np.arctan2(2.0 * (w * z), 1.0 - 2.0 * (z ** 2))
        return yaw

    def yaw_to_orientation(self, yaw):
        orientation_z = np.sin(yaw / 2.0)
        orientation_w = np.cos(yaw / 2.0)
        return orientation_z, orientation_w

    def sensor_a_callback(self, data):        
        #Calculate SmpTime
        current_time = rospy.Time.now().to_sec()
        if self.prev_time is not None:
            #nano sec
            self.SmpTime = current_time - self.prev_time
        else:
            self.SmpTime = 0.1#tmp
        self.prev_time = current_time
        
        #Calculate ishowSpeed
        current_pos = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y
        ])
        if self.prev_pos is not None:
            # Euclidean distance
            distance = np.linalg.norm(current_pos - self.prev_pos)
            self.Speed = distance / self.SmpTime
        else:
            self.Speed = 0#tmp
        self.prev_pos = current_pos
        
        #Calculate GTheta(yaw)
        self.GTheta = self.orientation_to_yaw(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        

    def sensor_b_callback(self, data):
        self.GpsXY = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        
        #Calculate GTheta(yaw)
        self.GPStheta = self.orientation_to_yaw(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        
        # Calculate the difference in GPS yaw
        self.DGPStheta = self.GPStheta - self.GPSthetayaw0
        
        # Update GPSthetayaw0 after calculating DGPStheta
        self.GPSthetayaw0 = self.GPStheta
        
        self.Number_of_satellites = data.pose.pose.position.z#Number of satellites
        
    def determination_of_R(self):                            
        if 0 <= self.Number_of_satellites < 4:#bad
            self.R1 = 1e-2# 0.01 FAST-LIO    
            self.R2 = 9e-2# 0.09 CLAS-movingbase
            self.R3 = 9 #GTheta
            self.R4 = 1 #GPStheta            

        elif 4 <= self.Number_of_satellites < 8:#soso
            self.R1 = 6e-2#0.06 FAST-LIO    
            self.R2 = 4e-2#0.04 CLAS-movingbase
            self.R3 = 4 #GTheta
            self.R4 = 6 #GPStheta            

        elif self.Number_of_satellites >= 8:#good!!!
            self.R1 = 9e-2# 0.09 FAST-LIO    
            self.R2 = 1e-2# 0.01 CLAS-movingbase
            self.R3 = 2 #GTheta
            self.R4 = 8 #GPStheta
            
        #self.Number_of_satellites = 0
        R = np.array([self.R1,self.R2,self.R3,self.R4])
        return R
    
    def initialize(self, GTheta, SmpTime):
        self.GTheta0 = GTheta
        self.XX = np.array([0 ,0 , np.cos(GTheta), np.sin(GTheta)])
        self.w = np.array([(1.379e-3)**2, (0.03 * np.pi / 180 * SmpTime)**2])###
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.array([[(1.379e-3)**2, 0], [0, (0.03 * np.pi / 180 * SmpTime)**2]])###
        G0 = np.array([[1, 0], [0, 0], [0, 0], [0, 1]])
        self.P = G0 @ self.Q @ G0.T

    def initializeGPS(self, GpsXY, GTheta, SmpTime):
        self.GTheta0 = GTheta
        self.XX = np.array([GpsXY[0], GpsXY[1], np.cos(GTheta), np.sin(GTheta)])
        self.w = np.array([(1.379e-3)**2, (0.03 * np.pi / 180 * SmpTime)**2])###
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.array([[(1.379e-3)**2, 0], [0, (0.03 * np.pi / 180 * SmpTime)**2]])##
        G0 = np.array([[1, 0], [0, 0], [0, 0], [0, 1]])
        self.P = G0 @ self.Q @ G0.T

    #def KalfXY(self, Speed, SmpTime, GTheta, GpsXY, HDOP, DMode, R1, R2):
    #def KalfXY(self, Speed, SmpTime, GTheta, GpsXY, Fixtype, R1, R2):
    def KalfXY(self, Speed, SmpTime, GTheta, R1, R2):
        if self.H is None:
            self.initialize(GTheta, SmpTime)

        self.R = np.array([[R1, 0], [0, R2]])

        DTheta = GTheta - self.GTheta0
        self.GTheta0 = GTheta
        # equation-of-state F G
        F = np.array([
            [1, 0, Speed * SmpTime * np.cos(DTheta), -Speed * SmpTime * np.sin(DTheta)],
            [0, 1, Speed * SmpTime * np.sin(DTheta), Speed * SmpTime * np.cos(DTheta)],
            [0, 0, np.cos(DTheta), -np.sin(DTheta)],
            [0, 0, np.sin(DTheta), np.cos(DTheta)]
        ])

        G = np.array([
            [np.cos(GTheta), -Speed * SmpTime * np.sin(GTheta)],
            [np.sin(GTheta), Speed * SmpTime * np.cos(GTheta)],
            [0, -np.sin(GTheta)],
            [0, np.cos(GTheta)]
        ])
        
        self.XX = F @ self.XX + G @ self.w

        return self.XX[:2]
        
    def KalfGPSXY(self, Speed, SmpTime, GTheta, GpsXY,R1, R2):
        if self.H is None:
            self.initializeGPS(GpsXY, GTheta, SmpTime)

        self.R = np.array([[R1, 0], [0, R2]])

        DTheta = GTheta - self.GTheta0
        self.GTheta0 = GTheta
        # equation-of-state F G
        F = np.array([
            [1, 0, Speed * SmpTime * np.cos(DTheta), -Speed * SmpTime * np.sin(DTheta)],
            [0, 1, Speed * SmpTime * np.sin(DTheta), Speed * SmpTime * np.cos(DTheta)],
            [0, 0, np.cos(DTheta), -np.sin(DTheta)],
            [0, 0, np.sin(DTheta), np.cos(DTheta)]
        ])

        G = np.array([
            [np.cos(GTheta), -Speed * SmpTime * np.sin(GTheta)],
            [np.sin(GTheta), Speed * SmpTime * np.cos(GTheta)],
            [0, -np.sin(GTheta)],
            [0, np.cos(GTheta)]
        ])

        # measurment CLAS movingbase
        #if GpsXY is not None and HDOP is not None and DMode is not None:
        #if GpsXY is not None and Fixtype is not None:
        Y = np.array([GpsXY[0], GpsXY[1]])
        
        Fixtype = 0
        
        if Fixtype == 0:        
                self.XX = F @ self.XX #filter equation
                self.P = F @ self.P @ F.T + G @ self.Q @ G.T #Prior Error Covariance 
                K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)#kalman gain
                self.XX = self.XX + K @ (Y - self.H @ self.XX) #estimated value 
                self.P = self.P - K @ self.H @ self.P#Posterior Error Covariance 
        else:
            self.XX = F @ self.XX + G @ self.w
            
        return self.XX[:2]

    def combine_yaw(self,Dtheta, theta1, theta2, w1, w2):
        #0~2pi
        if abs(Dtheta) < 5 * math.pi/180:        
            if theta1 < 0:
                theta1 += 2 * math.pi        
            if theta2 < 0:
                theta2 += 2 * math.pi
                    
            x1, y1 = w1 * math.cos(theta1), w1 * math.sin(theta1)#GTheta R3
            x2, y2 = w2 * math.cos(theta2), w2 * math.sin(theta2)#GPStheta R4
            x_sum = (x1 + x2) / (w1 + w2)
            y_sum = (y1 + y2) / (w1 + w2)
            theta_sum = math.atan2(y_sum, x_sum)
            #-pi~pi
            if theta_sum > math.pi:
                theta_sum -= 2 * math.pi
            elif theta_sum < -math.pi:
                theta_sum += 2 * math.pi 
        
        else :
            theta_sum = theta1
                    
        return theta_sum
    
    def calculate_offset(self, combyaw, GTheta, GPStheta): 
        deference = abs(GTheta) + abs(GPStheta)
        
        #1
        if GTheta>0 and GPStheta<0 and combyaw>0:
            self.GOffset = -(GTheta - combyaw)
            rospy.loginfo("1")
            
        elif GTheta>0 and GPStheta<0 and combyaw<0:
            self.GOffset = -(GTheta + abs(combyaw))
            rospy.loginfo("2")
            
        elif GTheta>0 and GPStheta>0 and combyaw>0 and GTheta > GPStheta:
            self.GOffset = -(abs(combyaw) - abs(GTheta))
            rospy.loginfo("3")
            
        elif GTheta<0 and GPStheta<0 and combyaw<0 and GTheta > GPStheta:
            self.GOffset = -(GTheta + abs(combyaw))
            rospy.loginfo("4")        
            #2
        elif GTheta<0 and GPStheta>0 and combyaw>0:
            self.GOffset = abs(GTheta) + combyaw
            rospy.loginfo("5")
        
        elif GTheta<0 and GPStheta>0 and combyaw<0:
            self.GOffset = abs(GTheta) - abs(combyaw)
            rospy.loginfo("6")
        
        elif GTheta>0 and GPStheta>0 and combyaw>0 and GTheta < GPStheta:
            self.GOffset = combyaw - GTheta
            rospy.loginfo("7")
        
        elif GTheta<0 and GPStheta<0 and combyaw<0 and GTheta < GPStheta:
            self.GOffset = abs(GTheta) - abs(combyaw)
            rospy.loginfo("8")        
            #3
        elif GTheta>0 and GPStheta<0 and combyaw>0 and deference > math.pi:
            self.GOffset = combyaw - GTheta
            rospy.loginfo("9")
        
        elif GTheta>0 and GPStheta<0 and combyaw<0 and deference > math.pi:
            self.GOffset = math.pi - GTheta + math.pi - abs(combyaw) 
            rospy.loginfo("10")
        
        elif GTheta<0 and GPStheta>0 and combyaw>0 and deference > math.pi:
            self.GOffset = -((math.pi - combyaw) + (math.pi - abs(GTheta)))
            rospy.loginfo("11")
        
        elif GTheta<0 and GPStheta>0 and combyaw<0 and deference > math.pi:
            self.GOffset = -(abs(combyaw) - abs(GTheta))
            rospy.loginfo("12")       
        
        if abs(self.GOffset) > 5 * math.pi/180:#-0.0872~0.0872
            self.GOffset = 0
            rospy.logwarn("GOffset warn") 
        
        return self.GOffset

    def publish_fused_value(self):
        if self.Speed is not None and self.SmpTime is not None and self.GTheta is not None:
            rospy.loginfo("data ok")
            rospy.loginfo(self.offsetyaw)
            
            R = self.determination_of_R()
            self.R1 = R[0]
            self.R2 = R[1]
            self.R3 = R[2]
            self.R4 = R[3]
            
            if self.GpsXY is not None:
                fused_value = self.KalfGPSXY(self.Speed, self.SmpTime, self.GTheta, self.GpsXY, self.R1, self.R2)#EKF CLAS(x,y) FASTLIO(x,y)
                self.GPS_conut += 1               
                if self.GPS_conut % 10 == 0:#ok
                    self.combyaw = self.combine_yaw(self.DGPStheta,self.GTheta,self.GPStheta,self.R3,self.R4) 
                    self.offsetyaw = self.calculate_offset(self.combyaw,self.GTheta,self.GPStheta)#yaw offset update
                
                self.robot_yaw = self.GTheta + self.offsetyaw
                if self.robot_yaw < -np.pi:
                    self.robot_yaw += 2 * np.pi
                elif self.robot_yaw < np.pi:
                    self.robot_yaw -= 2 * np.pi
                    
                robot_orientation = self.yaw_to_orientation(self.robot_yaw)
                self.robot_orientationz = robot_orientation[0]#yaw offset update
                self.robot_orientationw = robot_orientation[1]#
                
                self.fused_data.header.stamp = rospy.Time.now()
                self.fused_data.header.frame_id = "odom"
                self.fused_data.pose.pose.position.x = fused_value[0]
                self.fused_data.pose.pose.position.y = fused_value[1]
                self.fused_data.pose.pose.orientation.z = self.robot_orientationz
                self.fused_data.pose.pose.orientation.w = self.robot_orientationw  
            else:
                fused_value = self.KalfXY(self.Speed, self.SmpTime, self.GTheta,self.R1, self.R2)
                self.robot_yaw = self.GTheta + self.offsetyaw
                if self.robot_yaw < -np.pi:
                    self.robot_yaw += 2 * np.pi
                elif self.robot_yaw < np.pi:
                    self.robot_yaw -= 2 * np.pi
                robot_orientation = self.yaw_to_orientation(self.robot_yaw)
                self.robot_orientationz = robot_orientation[0]#
                self.robot_orientationw = robot_orientation[1]#
                
                self.fused_data.header.stamp = rospy.Time.now()
                self.fused_data.header.frame_id = "odom"
                self.fused_data.pose.pose.position.x = fused_value[0]
                self.fused_data.pose.pose.position.y = fused_value[1]
                self.fused_data.pose.pose.orientation.z = self.robot_orientationz
                self.fused_data.pose.pose.orientation.w = self.robot_orientationw

            self.fused_pub.publish(self.fused_data)
            #rospy.loginfo(f"Fused Value: {fused_value}")
        else:
            rospy.loginfo("data non")

if __name__ == "__main__":
    # init node
    rospy.init_node('sensor_fusion') 
    rate = rospy.Rate(20)#10
    EKF = ExtendedKalmanFilter()
    while not rospy.is_shutdown():
        EKF.publish_fused_value()
        rate.sleep()
