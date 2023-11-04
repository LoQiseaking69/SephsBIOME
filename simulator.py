
import numpy as np
from individual import Individual
from genome import Genome

class Simulator:
    def __init__(self, population_size, sensor_data_function):
        self.population_size = population_size
        self.population = [Individual(Genome()) for _ in range(self.population_size)]
        self.sensor_data_function = sensor_data_function
    
    def select_parents(self):
        # Select parents based on fitness
        fitness_scores = [individual.fitness for individual in self.population]
        parents = np.random.choice(self.population, size=2, p=fitness_scores/np.sum(fitness_scores))
        return parents
    
    def crossover(self, parent1, parent2):
        # Combine genes of two parents to create an offspring
        child_genome = Genome()
        for gene in child_genome.genes:
            if np.random.rand() > 0.5:
                child_genome.genes[gene] = parent1.genome.genes[gene]
            else:
                child_genome.genes[gene] = parent2.genome.genes[gene]
        return Individual(child_genome)
    
    def mutate(self, individual):
        # Introduce small random changes in the individual’s genes
        for gene in individual.genome.genes:
            if np.random.rand() < 0.1:  # 10% mutation rate
                individual.genome.genes[gene] += np.random.normal(0, 0.1)  # Small random change
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
            
            # Simulate interactions and evolution
            for individual in self.population:
                sensor_data = self.sensor_data_function()
                processed_data = individual.process_sensor_data(sensor_data)
                decision, reward = individual.make_decision(processed_data)
                individual.interact_with_robot(decision)
                individual.evaluate_fitness(reward)
            
            # Evolutionary adaptation
            self.run_one_generation()
