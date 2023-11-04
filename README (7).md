
# Digital Biome for ROS/RTOS Integration in Bipedal Robot

## Overview
This project implements a digital biome for a bipedal robot, creating an adaptive and interactive environment for robotic systems. The digital biome incorporates neural networks for decision-making, genetic algorithms for evolutionary processes, sensor data processing, and a simulation environment.

## Project Structure

### Python Scripts

#### 1. `neural_network.py`
   - **Purpose**: Implements a neural network model used for decision-making based on processed sensor data.
   - **Usage**: Used by individuals within the digital biome to make decisions based on their sensor inputs.

#### 2. `sensor_data.py`
   - **Purpose**: Handles the acquisition and processing of sensor data.
   - **Usage**: Provides simulated sensor data to the individuals, emulating the input from the environment.

#### 3. `individual.py`
   - **Purpose**: Defines the `Individual` class, representing entities within the digital biome.
   - **Usage**: Individuals are the entities that interact with the bipedal robot, make decisions, and evolve over time.

#### 4. `simulator.py`
   - **Purpose**: Implements the simulation environment and evolutionary processes.
   - **Usage**: Manages the lifecycle of individuals and facilitates the evolutionary processes within the digital biome.

#### 5. `genome.py`
   - **Purpose**: Manages the genetic information and algorithms for the individuals.
   - **Usage**: Provides the genetic foundation for the individuals, influencing their behavior and characteristics.

#### 6. `bipedal_robot.py`
   - **Purpose**: Represents the bipedal robot and its interactions with the digital biome.
   - **Usage**: Serves as the bipedal robot within the simulation, interacting with the individuals and adapting its behavior based on the digital biome.

#### 7. `evolution.py` (Optional)
   - **Purpose**: (Description based on the content of the script)
   - **Usage**: (Description based on how the script is used in the project)

## Usage
To run the simulation:

1. Ensure you have Python and the required packages installed.
2. Run the `simulator.py` script:
   ```
   python simulator.py
   ```

## Dependencies
- Python 3.x
- TensorFlow
- NumPy

## New Files in the Project

### `bipedal_robot.py`
This script defines the BipedalRobot class, which simulates the physical aspects and behaviors of a bipedal robot within the simulation.

### `evolution.py`
The Evolution class within this script manages the evolutionary process. It simulates the genetic algorithm loop, allowing a population of Individual objects to evolve over generations through processes of selection, reproduction (crossover), and mutation.

- **Initialization**: Set up with a population size and mutation rate, creating a starting population.
- **Parent Selection**: Selects parents based on fitness scores using roulette wheel selection.
- **Create Next Generation**: Generates a new population by selecting parents, performing crossover, and applying mutations.
- **Run Evolution**: Executes the evolutionary process over a set number of generations, iterating through cycles of parent selection, reproduction, and population replacement.

### `genome.py`
This script contains the Genome class, which represents the genetic information of an individual and includes methods for crossover and mutation.

### `individual.py`
Defines the Individual class, representing a single entity within the simulation. Each individual has a genome and a neural network model that governs its decision-making.

### `neural_network.py`
Implements the NeuralNetwork class, which defines the architecture and behavior of the neural networks used by the individuals in the simulation.

### `requirements.txt`
Lists all the Python package dependencies required to run the simulation.

### `sensor_data.py`
Contains functions for simulating the retrieval and processing of sensor data, which the neural networks use to make decisions.

### `setup_sephsbiome.sh`
A bash script to set up the Python environment, including the creation of a virtual environment and installation of dependencies from `requirements.txt`.

### `simulator.py`
The Simulator class in this script coordinates the entire simulation, managing the population of individuals, their interactions with the environment, and the evolutionary process.
