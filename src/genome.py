import random
import copy

class Gene:
    def __init__(self, source_type, source_num, sink_type, sink_num, weight, kinematic_trait, innovation=None, complex_behavior=False):
        self.source_type = source_type  # 0 for NEURON, 1 for SENSOR
        self.source_num = source_num
        self.sink_type = sink_type      # 0 for NEURON, 1 for ACTION
        self.sink_num = sink_num
        self.weight = weight
        self.kinematic_trait = kinematic_trait  # New attribute for kinematic trait
        self.innovation = innovation
        self.complex_behavior = complex_behavior  # Indicates complex behavior involvement

    def mutate_trait(self):
        # Logic to mutate kinematic trait
        trait_variation = random.uniform(-0.1, 0.1)  # Small random variation
        self.kinematic_trait += trait_variation
        self.kinematic_trait = max(0, min(self.kinematic_trait, 1))  # Keep within bounds

class Genome:
    def __init__(self, genes=None):
        self.genes = genes if genes else []

    def add_connection(self, source_type, source_num, sink_type, sink_num, weight, kinematic_trait, complex_behavior=False):
        gene = Gene(source_type, source_num, sink_type, sink_num, weight, kinematic_trait, complex_behavior=complex_behavior)
        self.genes.append(gene)

    def mutate(self, mutation_rate):
        for gene in self.genes:
            if random.random() < mutation_rate:
                mutation_type = random.choice(["weight", "add_connection", "remove_connection", "modify_interaction", "mutate_trait"])
                if mutation_type == "weight":
                    gene.weight += random.uniform(-1, 1)
                    gene.weight = max(-1, min(gene.weight, 1))  # Keep weight within bounds
                elif mutation_type == "add_connection":
                    self.add_random_connection()
                elif mutation_type == "remove_connection":
                    self.remove_random_connection()
                elif mutation_type == "modify_interaction":
                    self.modify_gene_interaction(gene)
                elif mutation_type == "mutate_trait":
                    gene.mutate_trait()

    def add_random_connection(self):
        source_type = random.choice([0, 1])
        sink_type = random.choice([0, 1])
        source_num = random.randint(0, 10)  # Assuming a range for source and sink identifiers
        sink_num = random.randint(0, 10)
        weight = random.uniform(-1, 1)
        kinematic_trait = random.uniform(0, 1)  # Random kinematic trait value
        complex_behavior = random.choice([True, False])
        self.add_connection(source_type, source_num, sink_type, sink_num, weight, kinematic_trait, complex_behavior)

    def remove_random_connection(self):
        if self.genes:
            gene_to_remove = random.choice(self.genes)
            self.genes.remove(gene_to_remove)

    def modify_gene_interaction(self, gene):
        if gene.source_type == 1:
            gene.weight *= random.uniform(0.8, 1.2)
            gene.weight = max(-1, min(gene.weight, 1))  # Keep weight within bounds
        elif gene.sink_type == 1:
            gene.weight += random.uniform(-0.5, 0.5)
            gene.weight = max(-1, min(gene.weight, 1))  # Keep weight within bounds
        if gene.complex_behavior:
            self.adjust_complex_behavior(gene)

    def adjust_complex_behavior(self, gene):
        gene.weight += random.uniform(-0.3, 0.3)  # Adjust weight for complex behaviors
        gene.weight = max(-1, min(gene.weight, 1))  # Keep weight within bounds

    def crossover(self, other_genome):
        new_genes = []
        for gene_self, gene_other in zip(self.genes, other_genome.genes):
            chosen_gene = copy.deepcopy(gene_self) if random.random() < 0.5 else copy.deepcopy(gene_other)
            new_genes.append(chosen_gene)
        return Genome(new_genes)

    def __repr__(self):
        return "\n".join([str(gene.__dict__) for gene in self.genes])
