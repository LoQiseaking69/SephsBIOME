import rospy
import numpy as np
import yaml
from sensor_msgs.msg import Imu, JointState
from datetime import datetime

class SensorDataHandler():
    def __init__(self):
        self._parse_parameters()
        rospy.init_node(self._node_name, anonymous=True)
        self._init_pubs_and_subs()
        self.data = {}
        self.sensor_data = None
        self.last_update = None

        if self._use_sim:
            rospy.loginfo("RUNNING IN SIMULATION MODE")
            self._load_simulator_configuration()

        rospy.loginfo("Sensor data handler initialized")

    def _parse_parameters(self):
        self._sensor_config_path = rospy.get_param('/config_path', '')
        self._sensor_frame_rate = rospy.get_param('/sensor_frame_rate', 50)
        self._node_name = rospy.get_param('/node_name', 'sensor_data_handler')
        self._use_sim = rospy.get_param('/use_sim', False)

    def _init_pubs_and_subs(self):
        self._imu_sub = rospy.Subscriber("/robot/sensors/imu", Imu, self._imu_callback)
        self._joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self._joint_state_callback)

    def _imu_callback(self, data):
        self.data['imu'] = data

    def _joint_state_callback(self, data):
        self.data['joint_states'] = data

    def get_latest_sensor_data(self):
        now = datetime.now()
        while 'imu' not in self.data or 'joint_states' not in self.data:
            rospy.sleep(0.1)
            if (now - self.last_update).seconds >= (1.0 / self._sensor_frame_rate):
                break

        if 'imu' in self.data and 'joint_states' in self.data:
            self.sensor_data = self._merge_sensor_data(self.sensor_data, self.data)
            self.last_update = datetime.now()

        return self.sensor_data

    def _load_simulator_configuration(self):
        if self._sensor_config_path:
            try:
                with open(self._sensor_config_path, 'r') as file:
                    config = yaml.safe_load(file)
                rospy.loginfo(f"Simulator configuration loaded from {self._sensor_config_path}")
            except IOError as e:
                rospy.logerr(f"Could not read simulator configuration file: {e}")
                config = {}

        # Set default configuration if file is not found or empty
        self.simulator_config = config.get('simulator', {'environment': 'default', 'physics': 'basic'})

    def _merge_sensor_data(self, cur_data, new_data):
        if not cur_data:
            return new_data
        else:
            return {
                'imu': self._merge_imu_data(cur_data.get('imu'), new_data['imu']),
                'joint_states': self._merge_joint_state_data(cur_data.get('joint_states'), new_data['joint_states'])
            }

    def _merge_imu_data(self, cur_imu, new_imu):
        if not cur_imu:
            return new_imu
        merged_imu = Imu()
        merged_imu.header.stamp = rospy.Time.now()
        merged_imu.linear_acceleration.x = np.mean([cur_imu.linear_acceleration.x, new_imu.linear_acceleration.x])
        merged_imu.linear_acceleration.y = np.mean([cur_imu.linear_acceleration.y, new_imu.linear_acceleration.y])
        merged_imu.linear_acceleration.z = np.mean([cur_imu.linear_acceleration.z, new_imu.linear_acceleration.z])
        merged_imu.angular_velocity.x = np.mean([cur_imu.angular_velocity.x, new_imu.angular_velocity.x])
        merged_imu.angular_velocity.y = np.mean([cur_imu.angular_velocity.y, new_imu.angular_velocity.y])
        merged_imu.angular_velocity.z = np.mean([cur_imu.angular_velocity.z, new_imu.angular_velocity.z])
        return merged_imu

    def _merge_joint_state_data(self, cur_joint_states, new_joint_states):
        if not cur_joint_states:
            return new_joint_states
        merged_joint_states = JointState()
        merged_joint_states.header.stamp = rospy.Time.now()
        merged_joint_states.position = np.mean([cur_joint_states.position, new_joint_states.position], axis=0)
        merged_joint_states.velocity = np.mean([cur_joint_states.velocity, new_joint_states.velocity], axis=0)
        merged_joint_states.effort = np.mean([cur_joint_states.effort, new_joint_states.effort], axis=0)
        return merged_joint_states

if __name__ == "__main__":
    node = SensorDataHandler()
    rospy.spin()
