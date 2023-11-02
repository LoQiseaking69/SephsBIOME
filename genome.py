
class Gene:
    def __init__(self, source_type, source_num, sink_type, sink_num, weight):
        self.source_type = source_type  # 0 for NEURON, 1 for SENSOR
        self.source_num = source_num
        self.sink_type = sink_type  # 0 for NEURON, 1 for ACTION
        self.sink_num = sink_num
        self.weight = weight

class Genome:
    def __init__(self):
        self.genes = []

    def add_connection(self, source_type, source_num, sink_type, sink_num, weight):
        gene = Gene(source_type, source_num, sink_type, sink_num, weight)
        self.genes.append(gene)

    
    def mutate(self, mutation_rate):
        # Example implementation: Randomly mutate genes based on mutation rate
        for gene in self.genes:
            if random.random() < mutation_rate:
                gene.weight += random.uniform(-1, 1)

    def crossover(self, other_genome):
        # Example implementation: Combine genetic material from two genomes to create a new genome
        new_genes = []
        for gene_self, gene_other in zip(self.genes, other_genome.genes):
            if random.random() < 0.5:
                new_genes.append(copy.deepcopy(gene_self))
            else:
                new_genes.append(copy.deepcopy(gene_other))
        return Genome(new_genes)

    def __repr__(self):
        return "\n".join([str(gene.__dict__) for gene in self.genes])
