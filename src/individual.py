import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from neural_network import create_neural_network_model
from genome import Genome

class Individual:
    def __init__(self, genome, seq_length, d_model):
        rospy.init_node('individual_node', anonymous=True)

        self.genome = genome
        self.fitness = 0
        self.energy = 100
        self.health = 100
        self.neural_network_model = create_neural_network_model(seq_length, d_model)
        self.decision_log = []
        self.reward_log = []

        # ROS subscribers and publishers
        self.sensor_data_subscriber = rospy.Subscriber("/sensor_data", Image, self.sensor_data_callback)
        self.control_publisher = rospy.Publisher("/control_command", String, queue_size=10)

    def sensor_data_callback(self, data):
        # Convert the received data and process
        sensor_data = self.convert_ros_to_numpy(data)
        processed_data = self.process_sensor_data(sensor_data)
        decision, reward = self.make_decision(processed_data)
        self.execute_decision(decision)
        self.evaluate_fitness(reward)

    def convert_ros_to_numpy(self, data):
        # Convert ROS Image data to numpy array
        image_data = np.frombuffer(data.data, dtype=np.uint8)
        image_data = image_data.reshape((data.height, data.width, -1))
        return image_data

    def process_sensor_data(self, sensor_data):
        # Incorporate genome influence in sensor data processing
        genome_influence = self.process_genes_for_sensor_data()
        processed_data = np.tanh(sensor_data.flatten() + genome_influence)
        processed_data = np.expand_dims(processed_data, axis=0)
        return processed_data

    def process_genes_for_sensor_data(self):
        # Calculate influence from genome on sensor data processing
        sensor_processing_value = sum(gene.weight for gene in self.genome.genes if gene.source_type == 'sensor')
        kinematic_influence = sum(gene.kinematic_trait for gene in self.genome.genes)
        return sensor_processing_value + kinematic_influence

    def make_decision(self, processed_sensor_data):
        # Use neural network model to make a decision
        prediction = self.neural_network_model.predict(processed_sensor_data)
        decision = prediction[0][0]
        reward = prediction[1][0]
        self.decision_log.append(decision)
        self.reward_log.append(reward)
        return decision, reward

    def execute_decision(self, decision):
        # Translate decision to a robot control command
        command = self.translate_decision_to_command(decision)
        self.control_publisher.publish(String(command))

    def translate_decision_to_command(self, decision):
        # Map decision to specific robot control commands
        if decision < 0.2:
            return "INCREASE_SPEED"
        elif decision < 0.4:
            return "DECREASE_SPEED"
        elif decision < 0.6:
            return "TURN_LEFT"
        elif decision < 0.8:
            return "TURN_RIGHT"
        else:
            return "STOP"

    def evaluate_fitness(self, reward):
        # Evaluate fitness based on multiple factors
        stability_score = self.calculate_stability()
        agility_score = self.calculate_agility()
        kinematic_efficiency = self.calculate_kinematic_efficiency()
        self.fitness = self.energy + self.health + reward + stability_score + agility_score + kinematic_efficiency

    def calculate_stability(self):
        # Calculate stability score from genome
        stability_genes = [gene for gene in self.genome.genes if gene.sink_type == 'stability']
        return np.mean([gene.weight for gene in stability_genes]) * 10 if stability_genes else 0

    def calculate_agility(self):
        # Calculate agility score from genome
        agility_genes = [gene for gene in self.genome.genes if gene.sink_type == 'agility']
        return np.mean([gene.weight for gene in agility_genes]) * 10 if agility_genes else 0

    def calculate_kinematic_efficiency(self):
        # Calculate kinematic efficiency from genome
        efficiency_genes = [gene for gene in self.genome.genes if gene.source_type == 'efficiency']
        return np.mean([gene.kinematic_trait for gene in efficiency_genes]) * 20 if efficiency_genes else 0

# Example usage (within a ROS environment):
 genome = Genome()  # Assuming Genome() is properly defined
 individual = Individual(genome, seq_length=10, d_model=32)
 rospy.spin()  # Keep the ROS node running
