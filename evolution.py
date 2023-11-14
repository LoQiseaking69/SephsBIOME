import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Adjust the message type based on your sensor data
from individual import Individual
from genome import Genome

class ROSInterface:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('evolution_node', anonymous=True)

        # Initialize ROS subscribers and publishers
        self.sensor_data_subscriber = rospy.Subscriber("/sensor_data", Image, self.sensor_data_callback)
        self.control_publisher = rospy.Publisher("/control_command", String, queue_size=10)

        # Store the latest sensor data
        self.latest_sensor_data = None

    def sensor_data_callback(self, data):
        # Update the latest sensor data on receiving new data
        self.latest_sensor_data = data

    def get_sensor_data(self):
        # Provide the latest sensor data
        return self.latest_sensor_data

    def send_control_command(self, command):
        # Send control commands to the robot
        self.control_publisher.publish(command)

class Evolution:
    def __init__(self, population_size, seq_length, d_model):
        self.population_size = population_size
        self.seq_length = seq_length
        self.d_model = d_model
        self.ros_interface = ROSInterface()  # Interface with ROS
        self.population = [Individual(Genome(), seq_length, d_model) for _ in range(population_size)]

    def run_generation(self):
        # Simulate a generation of individuals
        for individual in self.population:
            self.simulate_individual(individual)

        # Sort and select the top-performing individuals
        self.population.sort(key=lambda x: x.fitness, reverse=True)
        self.population = self.population[:self.population_size // 2]

        # Create the next generation
        next_generation = self.create_next_generation()
        self.population = next_generation

    def simulate_individual(self, individual):
        # Get sensor data from ROS and simulate individual's performance
        sensor_data = self.ros_interface.get_sensor_data()
        if sensor_data:
            processed_data = individual.process_sensor_data(sensor_data)
            decision, reward = individual.make_decision(processed_data)
            control_command = individual.determine_control_command(decision)
            self.ros_interface.send_control_command(control_command)
            individual.evaluate_fitness(reward)

    def create_next_generation(self):
        # Generate the next generation of individuals
        next_generation = []
        while len(next_generation) < self.population_size:
            parent1, parent2 = random.sample(self.population, 2)
            child_genome = parent1.genome.crossover(parent2.genome)
            child_genome.mutate(mutation_rate=0.1)
            child = Individual(child_genome, self.seq_length, self.d_model)
            next_generation.append(child)
        return next_generation

    def run_evolution(self, generations):
        # Run the evolutionary process for a set number of generations
        for _ in range(generations):
            self.run_generation()

# Example usage
evolution = Evolution(population_size=100, seq_length=10, d_model=32)
evolution.run_evolution(generations=50)
