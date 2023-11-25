from std_msgs.msg import String, Float64
import rospy

class BipedalRobot:
    def __init__(self):
        try:
            # Initialize the ROS node
            rospy.init_node('bipedal_robot_node', anonymous=True)
        except rospy.ROSInitException as e:
            rospy.logerr(f"ROS Init Exception: {e}")

        # Robot attributes
        self.computation_power = 100  # Initial computation power
        self.mobility = 100  # Initial mobility
        self.learning_ability = 100  # Initial learning ability
        self.energy_level = 100  # Energy level, affecting performance
        self.max_capacity = 200  # Maximum capacity for attributes

        # ROS publishers and subscribers
        try:
            self.status_publisher = rospy.Publisher('/robot_status', String, queue_size=10)
            self.energy_subscriber = rospy.Subscriber('/energy_update', Float64, self.energy_update_callback)
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Communication Exception: {e}")

    def energy_update_callback(self, data):
        # Update the energy level based on data received from a ROS topic
        try:
            new_energy_level = float(data.data)
            if 0 <= new_energy_level <= self.max_capacity:
                self.energy_level = new_energy_level
                rospy.loginfo(f"Updated energy level: {self.energy_level}")
            else:
                rospy.logwarn("Received invalid energy level update")
        except (ValueError, TypeError) as e:
            rospy.logerr(f"Invalid data type in energy update: {e}")

    def receive_assistance(self, individual):
        if self.energy_level > 0:
            self.apply_assistance(individual)
            self.publish_status()
        else:
            rospy.loginfo("Energy too low for assistance")

    def apply_assistance(self, individual):
        # Apply assistance based on individual's capabilities and robot's current state
        self.computation_power = min(self.computation_power + self.calculate_boost(individual.assist_computation(), self.computation_power), self.max_capacity)
        self.mobility = min(self.mobility + self.calculate_boost(individual.assist_mobility(), self.mobility), self.max_capacity)
        self.learning_ability = min(self.learning_ability + self.calculate_boost(individual.assist_learning(), self.learning_ability), self.max_capacity)

    def calculate_boost(self, assistance_value, current_value):
        # Calculate the boost value based on current capability and assistance
        boost = assistance_value * (1 - current_value / self.max_capacity)
        return boost

    def publish_status(self):
        # Publish the robot's status to a ROS topic
        status = f"Computation: {self.computation_power}, Mobility: {self.mobility}, Learning: {self.learning_ability}, Energy: {self.energy_level}"
        self.status_publisher.publish(status)

    def __repr__(self):
        return (f"Bipedal Robot - Computation: {self.computation_power}, "
                f"Mobility: {self.mobility}, Learning: {self.learning_ability}, "
                f"Energy: {self.energy_level}")

# Example usage (uncomment when running in a ROS environment)
# robot = BipedalRobot()
# rospy.spin()  # Keep the node running