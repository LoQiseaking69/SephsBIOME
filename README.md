# Seph's Biome: Advanced Bipedal Robot Evolution Simulator

## Project Overview
Seph's Biome is an innovative simulation platform focused on the evolution and analysis of bipedal robotic behaviors within a digitally simulated environment. This project combines neural networks, genetic algorithms, and advanced sensor data processing to develop adaptive and dynamic behaviors in bipedal robots.

## Installation and Setup
To set up Seph's Biome, follow these steps:
1. Clone the repository to your local machine.
2. Run the `setup_sephsbiome.sh` script. This script will set up a Python virtual environment, install necessary dependencies, and perform initial configuration.
3. Ensure that ROS (Robot Operating System) is installed and configured on your machine, as the project relies on ROS for managing robot simulations and sensor data.

## Dependencies
The project requires the following Python libraries:
- Numpy
- Matplotlib
- TensorFlow

Version specifics can be found in `requirements.txt`.

## Key Components
- **Main Module (`main.py`)**: Serves as the entry point, integrating components such as the simulator and the evolutionary process.
- **Bipedal Robot (`bipedal_robot.py`)**: Manages the simulation of the bipedal robot's behavior and interactions within the biome. Utilizes ROS for communication and control.
- **Evolution (`evolution.py`)**: Handles evolutionary algorithms, including selection, mutation, and fitness evaluation.
- **Genome (`genome.py`)**: Manages genetic information and mutations for the evolutionary process.
- **Individual (`individual.py`)**: Represents an individual entity in the simulation.
- **Neural Network (`neural_network.py`)**: Manages the neural network architecture for decision-making processes.
- **Performance Visualization (`performanceViz.py`)**: Provides visualization tools for analyzing the performance and outcomes of the simulation.
- **Sensor Data (`sensor_data.py`)**: Processes and interprets sensor data from the simulated environment. Interfaces with ROS for real-time data handling.
- **Simulator (`simulator.py`)**: Orchestrates the overall simulation environment, managing entities and simulation cycles.

## Running the Simulation
To run Seph's Biome:
1. Ensure all dependencies, including ROS, are properly installed.
2. Execute `main.py` to start the simulation.
3. Monitor the simulation's progress and performance through the visualizations provided by `performanceViz.py`.

## License
Seph's Biome is licensed under the MPL 2.0 License. For more details, refer to the `MPL_2_0_License.md` file.

