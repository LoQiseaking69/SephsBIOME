import sys
import logging
from simulator import Simulator
from evolution import Evolution
from genome import Genome
from neural_network import NeuralNetwork, create_neural_network_model
from performanceViz import PerformanceViz
from sensor_data import SensorDataHandler  # Updated import
from visualization import plot_fitness, plot_decisions
import rospy  # ROS Python library

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def main():
    rospy.init_node('bipedal_robot_main', anonymous=True)  # Initialize ROS node

    try:
        # Neural Network and Evolutionary Components Initialization
        model = create_neural_network_model(seq_length=10, d_model=32, action_space_size=5)
        simulator = Simulator()
        evolution = Evolution()
        neural_network = NeuralNetwork()
        performance_viz = PerformanceViz()
        sensor_handler = SensorDataHandler()  # Initialize sensor data handler

        # Start the evolution process
        for generation in range(evolution.num_generations):
            logging.info(f"Starting Generation {generation + 1}")

            genomes = evolution.create_generation()
            for genome_id, genome in genomes.items():
                logging.info(f"Simulating Genome {genome_id}")
                neural_network.set_weights(genome.weights)

                try:
                    sensor_data = sensor_handler.get_sensor_data()  # Retrieve real sensor data
                    processed_data = model.predict(sensor_data)  # Incorporate neural network predictions
                except Exception as e:
                    logging.error(f"Error during simulation: {e}")
                    continue

                fitness = evolution.evaluate_fitness(processed_data)
                evolution.update_genome_fitness(genome_id, fitness)
                performance_viz.update(generation, genome_id, fitness)

            evolution.evolve_generation()

        logging.info("Evolution process complete.")
        fitness_over_time, decisions_over_time = performance_viz.get_data()
        plot_fitness(fitness_over_time)
        plot_decisions(decisions_over_time)

        if evolution.save_final_state:
            evolution.save_state("final_state.pkl")

    except Exception as e:
        logging.error(f"An error occurred: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()