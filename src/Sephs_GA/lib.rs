mod genome;
mod individual;
mod evolution;

pub use genome::*;
pub use individual::*;
pub use evolution::*;

pub struct GeneticAlgorithm {
    pub evolution: evolution::Evolution,
    // Other shared properties or configurations
}

impl GeneticAlgorithm {
    pub fn new(population: Vec<individual::Individual>, mutation_rate: f64, crossover_rate: f64) -> Self {
        let evolution = evolution::Evolution::new(population, mutation_rate, crossover_rate);
        GeneticAlgorithm { evolution }
    }

    pub fn run(&mut self) {
        // Implement the logic to run the genetic algorithm
        // For example, running multiple generations
        for _ in 0..100 {  // Assuming 100 generations
            self.evolution.run_generation();
        }
    }
}
