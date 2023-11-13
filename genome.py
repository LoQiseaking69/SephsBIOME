import random
import copy

class Gene:
    def __init__(self, source_type, source_num, sink_type, sink_num, weight, innovation=None, complex_behavior=False):
        self.source_type = source_type  # 0 for NEURON, 1 for SENSOR
        self.source_num = source_num
        self.sink_type = sink_type      # 0 for NEURON, 1 for ACTION
        self.sink_num = sink_num
        self.weight = weight
        self.innovation = innovation
        self.complex_behavior = complex_behavior  # Indicates if the gene is involved in complex behaviors

class Genome:
    def __init__(self, genes=None):
        self.genes = genes if genes else []

    def add_connection(self, source_type, source_num, sink_type, sink_num, weight, complex_behavior=False):
        gene = Gene(source_type, source_num, sink_type, sink_num, weight, complex_behavior=complex_behavior)
        self.genes.append(gene)

    def mutate(self, mutation_rate):
        for gene in self.genes:
            if random.random() < mutation_rate:
                mutation_type = random.choice(["weight", "add_connection", "remove_connection", "modify_interaction"])
                if mutation_type == "weight":
                    gene.weight += random.uniform(-1, 1)
                elif mutation_type == "add_connection":
                    self.add_random_connection()
                elif mutation_type == "remove_connection":
                    self.remove_random_connection()
                elif mutation_type == "modify_interaction":
                    self.modify_gene_interaction(gene)

    def add_random_connection(self):
        source_type = random.choice([0, 1])
        sink_type = random.choice([0, 1])
        source_num = random.randint(0, 10)  # Assuming a range for source and sink identifiers
        sink_num = random.randint(0, 10)
        weight = random.uniform(-1, 1)
        complex_behavior = random.choice([True, False])  # Randomly decide if it's a complex behavior gene
        self.add_connection(source_type, source_num, sink_type, sink_num, weight, complex_behavior)

    def remove_random_connection(self):
        if self.genes:
            gene_to_remove = random.choice(self.genes)
            self.genes.remove(gene_to_remove)

    def modify_gene_interaction(self, gene):
        print("Modifying Gene Interaction")
        if gene.source_type == 1:
            gene.weight *= random.uniform(0.8, 1.2)
        elif gene.sink_type == 1:
            gene.weight += random.uniform(-0.5, 0.5)
        if gene.complex_behavior:
            self.adjust_complex_behavior(gene)

    def adjust_complex_behavior(self, gene):
        print("Adjusting Complex Behavior Gene")
        # Implement specific adjustments for complex behavior genes
        # Example: Adjusting influence on decision-making or learning processes
        gene.weight += random.uniform(-0.3, 0.3)  # Adjust weight for complex behaviors

    def crossover(self, other_genome):
        new_genes = []
        for gene_self, gene_other in zip(self.genes, other_genome.genes):
            chosen_gene = copy.deepcopy(gene_self) if random.random() < 0.5 else copy.deepcopy(gene_other)
            new_genes.append(chosen_gene)
        return Genome(new_genes)

    def __repr__(self):
        return "\n".join([str(gene.__dict__) for gene in self.genes])
