#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from robot_msgs.msg import RobotStatus
class BipedalRobot():

    def __init__(self):
        rospy.init_node('bipedal_robot')
        self.computation_power = 100
        self.mobility = 100
        self.learning_ability = 100
        self.energy_level = 100
        self.max_capacity = 200
        self._pub_status = rospy.Publisher('/robot/status', RobotStatus, queue_size=10)

        self._sub_energy = rospy.Subscriber('/robot/energy', Float64, self.update_energy)

    def update_energy(self, data):
        if data.data <= self.max_capacity:
            self.energy_level = data.data
        else:
            rospy.logwarn_throttle(10, "Invalid energy level input")

        self.publish_status()

    def receive_assistance(self, assistance_request):
        if self.energy_level == 0:
            rospy.logwarn("Cannot receive assistance without energy!")
            return
        for skill in assistance_request:
            self.apply_assistance(skill, assistance_request[skill])
        self.publish_status()

    def apply_assistance(self, skill, assist_value):
        curr_capacity = getattr(self, skill + '_ability')
        boost = self.get_boost_value(assist_value, curr_capacity)
        setattr(self, skill + '_ability', curr_capacity + boost)

    def get_boost_value(self, assist_val, curr):
        return assist_val * (1 - curr / self.max_capacity)

    def publish_status(self):
        status = RobotStatus()
        status.computation = self.computation_power
        status.mobility = self.mobility
        status.learning = self.learning_ability
        status.energy = self.energy_level
        self._pub_status.publish(status)


if __name__ == "__main__":
    robot = BipedalRobot()
    rospy.spin()
