
# Seph's Biome Project

## Overview
Seph's Biome is an advanced simulation environment designed to model, analyze, and evolve bipedal robotic behaviors in a digitally simulated biome. This project integrates neural networks, genetic algorithms, and sensor data processing to create a dynamic and adaptive simulation platform. The primary focus is on the development and evolution of bipedal robot behaviors within this artificial ecosystem.

## Project Structure

### Core Components
1. **Bipedal Robot Simulation (`bipedal_robot.py`)**: Simulates the physical aspects and behaviors of a bipedal robot, including its interactions with the biome.

2. **Evolutionary Process Management (`evolution.py`)**: Handles the evolutionary processes of simulated entities, covering aspects like parent selection, generation creation, and fitness evaluation.

3. **Genetic Information Handling (`genome.py`)**: Manages genetic data and mutations, influencing the evolutionary development of entities.

4. **Individual Entity Behavior (`individual.py`)**: Defines the characteristics and decision-making processes of individual entities within the biome.

5. **Neural Network Implementation (`neural_network.py`)**: Facilitates complex decision-making for entities using a neural network model.

6. **Performance Visualization (`performanceViz.py`)**: Provides tools for visualizing various metrics and data, aiding in analysis and refinement of the simulation.

7. **Sensor Data Processing (`sensor_data.py`)**: Acquires and processes sensor data for use in decision-making and environmental interaction.

8. **Simulation Coordination (`simulator.py`)**: Acts as the central script, coordinating the simulation environment and integrating various components.

### Dependencies
- Python 3.x
- TensorFlow
- NumPy
- Additional dependencies are listed in `requirements.txt`.

### Setup and Execution
- Run `setup_sephsbiome.sh` to set up the Python environment and install dependencies.
- Execute `simulator.py` to start the simulation process.

## Usage
To use this project effectively, it is recommended to have a background in neural networks, genetic algorithms, and robotic systems. The project is designed for research and development in robotic behaviors within a simulated ecological environment.
