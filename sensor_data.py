import rospy
import numpy as np
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist

class SensorDataHandler:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('sensor_data_handler', anonymous=True)

        # Subscribe to sensor topics
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # Initialize variables to store sensor data
        self.imu_data = None
        self.joint_states = None

    def imu_callback(self, data):
        # Callback function for IMU data
        self.imu_data = data

    def joint_state_callback(self, data):
        # Callback function for joint state data
        self.joint_states = data

    def get_sensor_data(self):
        # Wait for the first set of data to be available
        while self.imu_data is None or self.joint_states is None:
            rospy.sleep(0.1)

        # Process and return sensor data
        # You can modify this part based on what data you need
        imu_array = np.array([self.imu_data.linear_acceleration.x,
                              self.imu_data.linear_acceleration.y,
                              self.imu_data.linear_acceleration.z,
                              self.imu_data.angular_velocity.x,
                              self.imu_data.angular_velocity.y,
                              self.imu_data.angular_velocity.z])

        joint_angles = np.array(self.joint_states.position)
        sensor_data = np.concatenate((imu_array, joint_angles))
        return sensor_data

def process_sensor_data(sensor_data):
    # Process sensor data
    # Modify this function as per your requirements
    processed_data = np.tanh(sensor_data)
    return processed_data

if __name__ == "__main__":
    sensor_handler = SensorDataHandler()
    while not rospy.is_shutdown():
        sensor_data = sensor_handler.get_sensor_data()
        processed_data = process_sensor_data(sensor_data)
        # Do something with the processed data
