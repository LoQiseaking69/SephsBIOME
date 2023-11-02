
import random
from individual import Individual
from genome import Genome

class Evolution:
    def __init__(self, population_size, mutation_rate):
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        self.population = [Individual() for _ in range(population_size)]

    def select_parents(self):
        # Example implementation: Select two parents based on fitness proportional selection
        total_fitness = sum(individual.evaluate_fitness() for individual in self.population)
        selection_probs = [individual.evaluate_fitness() / total_fitness for individual in self.population]
        parents = random.choices(self.population, weights=selection_probs, k=2)
        return parents

    def create_next_generation(self):
        new_population = []
        for _ in range(self.population_size):
            parent1, parent2 = self.select_parents()
            offspring_genome = parent1.genome.crossover(parent2.genome)
            offspring_genome.mutate(self.mutation_rate)
            offspring = Individual(genome=offspring_genome)
            new_population.append(offspring)
        self.population = new_population

    def run_evolution(self, generations):
        for _ in range(generations):
            self.create_next_generation()
