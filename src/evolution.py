import rospy
from sensor_msgs.msg import Image
from individual import Individual
from genome import Genome
import random

class ROSInterface:
    def __init__(self):
        rospy.init_node('evolution_node', anonymous=True)
        self.sensor_data_subscriber = rospy.Subscriber("/sensor_data", Image, self.sensor_data_callback)
        self.control_publisher = rospy.Publisher("/control_command", String, queue_size=10)
        self.latest_sensor_data = None

    def sensor_data_callback(self, data):
        self.latest_sensor_data = data

    def get_sensor_data(self):
        return self.latest_sensor_data

    def send_control_command(self, command):
        self.control_publisher.publish(command)

class Evolution:
    def __init__(self, population_size, seq_length, d_model):
        self.population_size = population_size
        self.seq_length = seq_length
        self.d_model = d_model
        self.ros_interface = ROSInterface()
        self.population = [Individual(Genome(), seq_length, d_model) for _ in range(population_size)]

    def run_generation(self):
        for individual in self.population:
            self.simulate_individual(individual)

        self.population.sort(key=lambda x: x.fitness, reverse=True)
        self.population = self.population[:self.population_size // 2]
        next_generation = self.create_next_generation()
        self.population = next_generation

    def simulate_individual(self, individual):
        sensor_data = self.ros_interface.get_sensor_data()
        if sensor_data:
            processed_data = individual.process_sensor_data(sensor_data)
            decision, reward = individual.make_decision(processed_data)
            control_command = individual.determine_control_command(decision)
            self.ros_interface.send_control_command(control_command)
            individual.evaluate_fitness(reward)

    def create_next_generation(self):
        next_generation = []
        while len(next_generation) < self.population_size:
            parent1, parent2 = random.sample(self.population, 2)
            child_genome = parent1.genome.crossover(parent2.genome)
            child_genome.mutate(mutation_rate=0.1)
            child = Individual(child_genome, self.seq_length, self.d_model)
            next_generation.append(child)
        return next_generation

    def run_evolution(self, generations):
        for _ in range(generations):
            self.run_generation()

# Example usage
evolution = Evolution(population_size=100, seq_length=10, d_model=32)
evolution.run_evolution(generations=50)

