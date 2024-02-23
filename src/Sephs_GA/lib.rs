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
    // Adjust the parameters to match your Evolution struct's initializer
    pub fn new(population_size: usize, seq_length: usize, d_model: usize, mutation_rate: f64, crossover_rate: f64) -> Self {
        // Initialize the Evolution struct with the provided parameters
        let evolution = evolution::Evolution::new(population_size, seq_length, d_model, mutation_rate, crossover_rate);
        GeneticAlgorithm { evolution }
    }

    pub fn run(&mut self, generations: usize) {
        // Implement the logic to run the genetic algorithm
        // Running the specified number of generations
        for _ in 0..generations {
            self.evolution.run_generation();
        }
    }
}

fn main() {
    // Example initialization and running of the Genetic Algorithm
    // The parameters here are placeholders and should be adjusted based on your actual requirements
    let mut ga = GeneticAlgorithm::new(100, 10, 32, 0.1, 0.5);
    ga.run(100);  // Run the genetic algorithm for 100 generations
}
