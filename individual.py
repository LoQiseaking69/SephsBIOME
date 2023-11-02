from genome import Gene, Genome
import numpy as np
from neural_network import create_neural_network_model

class Individual:
    def __init__(self, genome):
        self.genome = genome
        self.fitness = 0
        self.energy = 100  # Initial energy
        self.health = 100  # Initial health
        self.neural_network_model = create_neural_network_model()
    
    def process_sensor_data(self, sensor_data):
        # Simulate processing of sensor data based on individual's genome
        processed_data = np.tanh(sensor_data + self.process_genes_for_sensor_data())
        # Ensure the processed data is in the correct format for the neural network model input
        processed_data = np.expand_dims(processed_data, axis=0)
        return processed_data
    
    
    def process_genes_for_sensor_data(self):
        # Example implementation: sum the weights of all genes connected to sensors for sensor data processing
        sensor_processing_value = sum(gene.weight for gene in self.genome.genes if gene.source_type == 1)
        return sensor_processing_value

    def make_decision(self, processed_sensor_data):
        # Utilize the neural network model for decision-making
        prediction = self.neural_network_model.predict(processed_sensor_data)
        decision = prediction[0][0]  # Assuming the first output is the decision
        reward = prediction[1][0]  # Assuming the second output is the reward
        return decision, reward
    
    def interact_with_robot(self, decision):
        # Simulate interaction with the bipedal robot
        if decision > 0.5:
            print("Moving Forward")
        elif decision < -0.5:
            print("Moving Backward")
        else:
            print("Standing Still")
    
    def evaluate_fitness(self, reward):
        # Simulate fitness evaluation based on individual's state, decisions, and received reward
        self.fitness = self.energy + self.health + reward  # Simplified fitness function
