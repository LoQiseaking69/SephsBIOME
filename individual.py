import numpy as np
from neural_network import create_neural_network_model

class Individual:
    def __init__(self, genome, seq_length, d_model):
        self.genome = genome
        self.fitness = 0
        self.energy = 100  # Initial energy
        self.health = 100  # Initial health
        self.neural_network_model = create_neural_network_model(seq_length, d_model)
    
    def process_sensor_data(self, sensor_data):
        # Process sensor data based on individual's genome
        processed_data = np.tanh(sensor_data + self.process_genes_for_sensor_data())
        # Format for neural network input
        processed_data = np.expand_dims(processed_data, axis=0)
        return processed_data
    
    def process_genes_for_sensor_data(self):
        # Sum the weights of genes connected to sensors
        sensor_processing_value = sum(gene.weight for gene in self.genome.genes if gene.source_type == 1)
        return sensor_processing_value

    def make_decision(self, processed_sensor_data):
        # Decision-making with neural network
        prediction = self.neural_network_model.predict(processed_sensor_data)
        decision = prediction[0][0]  # Assuming first output is decision
        reward = prediction[1][0]  # Assuming second output is reward
        return decision, reward
    
    def interact_with_robot(self, decision):
        # Interaction with bipedal robot
        if decision > 0.5:
            print("Moving Forward")
        elif decision < -0.5:
            print("Moving Backward")
        else:
            print("Standing Still")
    
    def evaluate_fitness(self, reward):
        # Fitness evaluation
        self.fitness = self.energy + self.health + reward

