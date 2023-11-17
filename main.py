import sys
import logging
from simulator import Simulator
from evolution import Evolution
from genome import Genome
from neural_network import create_neural_network_model
from performanceViz import PerformanceViz
from sensor_data import SensorDataHandler
from visualization import plot_fitness, plot_decisions
import rospy

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def main():
    rospy.init_node('bipedal_robot_main', anonymous=True)  # Initialize ROS node

    try:
        # Neural Network and Evolutionary Components Initialization
        seq_length = 10
        d_model = 32
        action_space_size = 5
        model = create_neural_network_model(seq_length, d_model, action_space_size)
        sensor_handler = SensorDataHandler()  # Initialize sensor data handler

        population_size = 100
        simulator = Simulator(population_size, seq_length, d_model)
        evolution = Evolution(population_size, seq_length, d_model)
        performance_viz = PerformanceViz()

        # Start the evolution process
        for generation in range(evolution.num_generations):
            logging.info(f"Starting Generation {generation + 1}")

            simulator.run_simulation(1)  # Run simulation for each generation

            # Evaluate and update fitness based on simulation results
            for individual in simulator.population:
                sensor_data = sensor_handler.get_sensor_data()  # Retrieve real sensor data
                processed_data = model.predict(sensor_data)  # Incorporate neural network predictions
                individual.evaluate_fitness_from_processed_data(processed_data)
                performance_viz.update(generation, individual.genome, individual.fitness)

            evolution.evolve_population(simulator.population)

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