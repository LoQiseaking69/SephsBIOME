import sys
import logging
from simulator import Simulator
from evolution import Evolution
from genome import Genome
from neural_network import NeuralNetwork
from performanceViz import PerformanceViz
from sensor_data import SensorData
from visualization import plot_fitness, plot_decisions
from neural_network import create_neural_network_model

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def main():
    try:
        # Neural Network and Evolutionary Components Initialization
        model = create_neural_network_model(seq_length=10, d_model=32, action_space_size=5)  # Adjust parameters as needed
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
                    processed_data = model.predict(sensor_data)  # Incorporate neural network predictions
                except Exception as e:
                    logging.error(f"Error during simulation: {e}")
                    continue

                # Evaluate the performance
                fitness = evolution.evaluate_fitness(processed_data)  # Use processed data for fitness evaluation
                evolution.update_genome_fitness(genome_id, fitness)

                # Optional: Visualize performance
                performance_viz.update(generation, genome_id, fitness)

            # Evolve to the next generation
            evolution.evolve_generation()

        logging.info("Evolution process complete.")

        # Visualization after completing all generations
        fitness_over_time, decisions_over_time = performance_viz.get_data()
        plot_fitness(fitness_over_time)
        plot_decisions(decisions_over_time)

        # Optionally, save the final state or best performing genomes
        if evolution.save_final_state:
            evolution.save_state("final_state.pkl")

    except Exception as e:
        logging.error(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()