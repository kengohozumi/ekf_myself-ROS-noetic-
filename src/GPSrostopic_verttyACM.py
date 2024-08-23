#!/usr/bin/env python3
import rospy
import serial
from sensor_msgs.msg import NavSatFix

class GPSDataToxyz:
    def __init__(self):
        # name of the port that connects to gps device
        self.dev_name = "/dev/ttyACM1" #"/dev/ttyACM1" "/dev/ttyUSB0"

        self.fix_pub = rospy.Publisher("fix", NavSatFix, queue_size=10)
        self.fix_msg = NavSatFix()

        #self.dev_name = rospy.get_param("~port", "/dev/sensors/GNSSbase")
        self.serial_baud = rospy.get_param("~baud", 9600)#UM928 115200 F9P 9600

        self.country_id = rospy.get_param("~country_id", 0)
        # country_id == 0, then Japan
        # country_id == 1, then USA

    def get_gps(self, dev_name, country_id):
        # interface with sensor device(as a serial port)
        try:
            serial_port = serial.Serial(dev_name, self.serial_baud)
        except serial.SerialException as serialerror:
            print(serialerror)
            rospy.logerr("%s", serialerror)
            return None
        
        # country info 
        if country_id == 0:   # Japan
            initial_letters = "$GNGGA"#f9p can not use
        elif country_id == 1: # USA
            initial_letters = "$GPGGA"
        else:                 # not certain
            initial_letters = None

#    gps_data = ["$G?GGA", 
#                "UTC time", 
#                "Latitude (ddmm.mmmmm)", 
#                "latitude type (south/north)", 
#                "Longitude (ddmm.mmmmm)", 
#                "longitude type (east longitude/west longitude)", 
#                "Fixtype", 
#                "Number of satellites used for positioning", 
#                "HDOP", 
#                "Altitude", 
#                "M(meter)", 
#                "Elevation", 
#                "M(meter)", 
#                "", 
#                "checksum"]        
        while 1:
            line = serial_port.readline().decode('latin-1')        
            gps_data = line.split(',')       
            if gps_data[0] == initial_letters:
                break
                
        rospy.loginfo(gps_data)
        Fixtype_data = int(gps_data[6])
        rospy.loginfo(Fixtype_data)
        if Fixtype_data != 0:
            satelitecount_data = int(gps_data[7])#Number of satellites used for positioning
            rospy.loginfo(satelitecount_data)
            if Fixtype_data != 0:
                latitude_data = float(gps_data[2]) / 100.0  # ddmm.mmmmm to dd.ddddd
                if gps_data[3] == 'S':#south
                    latitude_data *= -1
                longitude_data = float(gps_data[4]) / 100.0  # ddmm.mmmmm to dd.ddddd
                if gps_data[5] == 'W':#west
                    longitude_data *= -1
                altitude_data = float(gps_data[9])
            else :
                #not fix data
                latitude_data = 0
                longitude_data = 0
                altitude_data = 0
                satelitecount_data = 0
        else :
        #no GPS data
            latitude_data = 0
            longitude_data = 0
            altitude_data = 0
            satelitecount_data = 0

        serial_port.close()

        gnggadata = (Fixtype_data,latitude_data,longitude_data,altitude_data,satelitecount_data)
        rospy.loginfo("current latitude and longitude (Fixtype,latitude, longitude,altitude):")
        rospy.loginfo(gnggadata)
    
        return gnggadata

    def pub_fix(self):
        lonlat = self.get_gps(self.dev_name, self.country_id)
        if lonlat is not None:
            self.fix_msg.header.stamp = rospy.Time.now()
            self.fix_msg.header.frame_id = "gps"
            # status.status = 2 is an analytical Fix solution based on the reference station. fix 2 nonfix -1
            self.fix_msg.status.status = lonlat[0]
            self.fix_msg.latitude = lonlat[1]
            self.fix_msg.longitude = lonlat[2]
            self.fix_msg.altitude = lonlat[3]
            self.fix_msg.position_covariance[0] = lonlat[4]#Number of satellites
            self.fix_pub.publish(self.fix_msg)
        else :
            rospy.loginfo("non GPS data")    

if  __name__ == "__main__":
    # init node
    rospy.init_node("gps_data_acquisition")
    rate = rospy.Rate(3)
    gtxyz = GPSDataToxyz()
    while not rospy.is_shutdown():
        gtxyz.pub_fix()
        rate.sleep()
