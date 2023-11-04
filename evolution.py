import random
from individual import Individual
from genome import Genome

class Evolution:
    def __init__(self, population_size, mutation_rate, seq_length, d_model):
        self.population_size = population_size
        self.mutation_rate = mutation_rate
        # Updated to pass seq_length and d_model to Individual
        self.population = [Individual(Genome(), seq_length, d_model) for _ in range(population_size)]

    def select_parents(self):
        # Select two parents based on fitness proportional selection
        total_fitness = sum(individual.fitness for individual in self.population)
        selection_probs = [individual.fitness / total_fitness for individual in self.population]
        parents = random.choices(self.population, weights=selection_probs, k=2)
        return parents

    def create_next_generation(self):
        new_population = []
        for _ in range(self.population_size):
            parent1, parent2 = self.select_parents()
            # Perform crossover and mutation to produce offspring
            offspring_genome = parent1.genome.crossover(parent2.genome)
            offspring_genome.mutate(self.mutation_rate)
            # Create a new Individual with the new genome
            offspring = Individual(offspring_genome, parent1.seq_length, parent1.d_model)
            new_population.append(offspring)
        self.population = new_population

    def run_evolution(self, generations):
        for _ in range(generations):
            self.create_next_generation()
            # Optional: Add code to assess and print population fitness after each generation

