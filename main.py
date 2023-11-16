import sys
import time
import logging
from simulator import Simulator
from evolution import Evolution
from genome import Genome
from neural_network import NeuralNetwork
from performanceViz import PerformanceViz
from sensor_data import SensorData

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def main():
    try:
        # Initialize components
        simulator = Simulator()
        evolution = Evolution()
        neural_network = NeuralNetwork()
        performance_viz = PerformanceViz()

        # Start the evolution process
        for generation in range(evolution.num_generations):
            logging.info(f"Starting Generation {generation + 1}")

            # Create or evolve genomes
            genomes = evolution.create_generation()

            for genome_id, genome in genomes.items():
                logging.info(f"Simulating Genome {genome_id}")
                
                # Initialize the neural network with the current genome
                neural_network.set_weights(genome.weights)

                # Run the simulation and handle any simulation errors
                try:
                    sensor_data = simulator.run(neural_network)
                except Exception as e:
                    logging.error(f"Error during simulation: {e}")
                    continue

                # Evaluate the performance
                fitness = evolution.evaluate_fitness(sensor_data)
                evolution.update_genome_fitness(genome_id, fitness)

                # Optional: Visualize performance
                performance_viz.update(generation, genome_id, fitness)

            # Evolve to the next generation
            evolution.evolve_generation()

        logging.info("Evolution process complete.")

        # Optionally, save the final state or best performing genomes
        if evolution.save_final_state:
            evolution.save_state("final_state.pkl")

    except Exception as e:
        logging.error(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()