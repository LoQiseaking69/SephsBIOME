pub mod individual {
    use crate::genome::{Gene, GeneType};
    use rand::prelude::*;

    #[derive(Debug, Clone)]
    pub struct Individual {
        genome: Vec<Gene>,
        fitness: f64,
        energy: f64,
    }

    impl Individual {
        pub fn new(genome: Vec<Gene>, energy: f64) -> Self {
            Individual { genome, fitness: 0.0, energy }
        }

        pub fn evaluate_fitness(&mut self) {
            self.fitness = self.genome.iter().map(|gene| gene.weight * gene.expression_level).sum();
            self.fitness += self.energy;  // Additional factors can be added here.
        }

        pub fn mutate_genome(&mut self) {
            for gene in &mut self.genome {
                gene.mutate();
            }
        }
    }
}
