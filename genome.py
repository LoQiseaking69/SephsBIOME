import random
import copy

class Gene:
    def __init__(self, source_type, source_num, sink_type, sink_num, weight, innovation=None):
        self.source_type = source_type  # 0 for NEURON, 1 for SENSOR
        self.source_num = source_num
        self.sink_type = sink_type  # 0 for NEURON, 1 for ACTION
        self.sink_num = sink_num
        self.weight = weight
        self.innovation = innovation

class Genome:
    def __init__(self, genes=None):
        self.genes = genes if genes else []

    def add_connection(self, source_type, source_num, sink_type, sink_num, weight):
        gene = Gene(source_type, source_num, sink_type, sink_num, weight)
        self.genes.append(gene)

    def mutate(self, mutation_rate):
        for gene in self.genes:
            if random.random() < mutation_rate:
                mutation_type = random.choice(["weight", "add_connection", "remove_connection"])
                if mutation_type == "weight":
                    gene.weight += random.uniform(-1, 1)
                elif mutation_type == "add_connection":
                    self.add_random_connection()
                elif mutation_type == "remove_connection":
                    self.remove_random_connection()

    def add_random_connection(self):
        # Randomly select source and sink types and numbers
        # You might want to set max values for source_num and sink_num based on your design
        source_type = random.choice([0, 1])
        sink_type = random.choice([0, 1])
        source_num = random.randint(0, 10)  # 10 is just an example value
        sink_num = random.randint(0, 10)
        weight = random.uniform(-1, 1)
        self.add_connection(source_type, source_num, sink_type, sink_num, weight)

    def remove_random_connection(self):
        if self.genes:
            gene_to_remove = random.choice(self.genes)
            self.genes.remove(gene_to_remove)

    def crossover(self, other_genome):
        new_genes = []
        for gene_self, gene_other in zip(self.genes, other_genome.genes):
            if random.random() < 0.5:
                new_genes.append(copy.deepcopy(gene_self))
            else:
                new_genes.append(copy.deepcopy(gene_other))
        return Genome(new_genes)

    def __repr__(self):
        return "\n".join([str(gene.__dict__) for gene in self.genes])