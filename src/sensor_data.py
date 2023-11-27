#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu, JointState  
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from datetime import datetime
import os

class SensorDataHandler():

    def __init__(self):
        # Get parameters
        self._parse_parameters()
        
        # Init ros node 
        rospy.init_node(self._node_name, anonymous=True)
        
        # Setup pubs & subs
        self._init_pubs_and_subs()

        # Init class vars
        self.data = {}        
        self.sensor_data = None
        self.last_update = None

        # Load configs 
        if self._use_sim:
            rospy.loginfo("RUNNING IN SIMULATION MODE")
            self._load_simulator_configuration()

        rospy.loginfo("Sensor data handler initialized")
        

    def _parse_parameters(self):
        # Get parameters from param server

        self._sensor_config_path = rospy.get_param('/config_path', '') 
        self._sensor_frame_rate = rospy.get_param('/sensor_frame_rate', 50)  
        self._node_name = rospy.get_param('/node_name', 'sensor_data_handler')
        self._use_sim = rospy.get_param('/use_sim', False) 


    def _init_pubs_and_subs(self):
        # Pub/sub handlers
        self._imu_sub = rospy.Subscriber("/robot/sensors/imu", Imu, self._imu_callback)  
        self._joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self._joint_state_callback)

    
    def _imu_callback(self, data):
        # Handler for imu data  
        self.data[data.header.frame_id] = (data, datetime.now()) 


    def _joint_state_callback(self, data):
        # Handle joint state data
        self.data[data.header.frame_id] = (data, datetime.now())


    def get_latest_sensor_data(self):
        
        now = datetime.now()        
        while any([x is None for x in self.data.values()]) and (now - self.last_update).seconds < (1.0/self._sensor_frame_rate):
            rospy.sleep(0.1) 

        if self.data:
            latest_stamp = max([x[1] for x in self.data.values()])
            for k in self.data:
                t = self.data[k]
                if abs(latest_stamp-t[1]) < rospy.Duration(1.0/self._sensor_frame_rate):  
                    self.sensor_data = self._merge_sensor_data(self.sensor_data, self.data[k][0] )

            self.last_update = datetime.now()
        
        return self.sensor_data


    def _load_simulator_configuration(self):
        # If running in simulator mode, configure simulator here 
        pass


    def _merge_sensor_data(self, cur_data, new_data):
        # Merge new sensor data appropriately
        if not cur_data:
            return new_data 
        else:
            # TODO - add sensor data merge handling/calibration 
            return cur_data


if __name__ == "__main__":
    node = SensorDataHandler()
    rospy.spin()
