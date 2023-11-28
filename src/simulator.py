import numpy as np
from individual import Individual
from genome import Genome
from sensor_data import SensorDataHandler

class Simulator:
    def __init__(self, population_size, seq_length, d_model):
        self.population_size = population_size
        self.seq_length = seq_length
        self.d_model = d_model
        self.sensor_handler = SensorDataHandler()
        self.population = [Individual(Genome(), seq_length, d_model) for _ in range(population_size)]

    def select_parents(self):
        fitness_scores = [individual.fitness for individual in self.population]
        total_fitness = np.sum(fitness_scores)
        if total_fitness == 0:
            return np.random.choice(self.population, size=2)
        probabilities = [fitness / total_fitness for fitness in fitness_scores]
        parents = np.random.choice(self.population, size=2, p=probabilities)
        return parents

    def crossover(self, parent1, parent2):
        child_genome = parent1.genome.crossover(parent2.genome)
        return Individual(child_genome, self.seq_length, self.d_model)

    def mutate(self, individual):
        individual.genome.mutate(mutation_rate=0.1)
        return individual

    def run_one_generation(self):
        next_generation = []
        for _ in range(self.population_size):
            parent1, parent2 = self.select_parents()
            child = self.crossover(parent1, parent2)
            child = self.mutate(child)
            next_generation.append(child)
        self.population = next_generation

    def run_simulation(self, generations):
        fitness_over_time = []
        decisions_over_time = []

        for generation in range(generations):
            print(f"Running Generation {generation + 1}")

            for individual in self.population:
                sensor_data = self.sensor_handler.get_sensor_data()
                processed_data = individual.process_sensor_data(sensor_data)
                decision, reward = individual.make_decision(processed_data)
                individual.execute_decision(decision)
                individual.evaluate_fitness(reward)

            generation_fitness = [individual.fitness for individual in self.population]
            generation_decisions = [individual.decision_log for individual in self.population]
            fitness_over_time.append(generation_fitness)
            decisions_over_time.append(generation_decisions)

            self.run_one_generation()

            for individual in self.population:
                individual.decision_log = []
                individual.reward_log = []

        return fitness_over_time, decisions_over_time
