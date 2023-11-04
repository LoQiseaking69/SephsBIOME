
# Digital Biome for ROS/RTOS Integration in Bipedal Robot

## Overview
This project implements a digital biome for a bipedal robot, creating an adaptive and interactive environment for robotic systems. Utilizing neural networks for decision-making and genetic algorithms for evolutionary processes, the project facilitates the integration of ROS/RTOS systems for complex robotic behaviors. The simulation environment processes sensor data to emulate real-world interactions.

## Project Structure

### Python Scripts

#### `neural_network.py`
   - Implements a neural network model for decision-making.
   - Used by the simulation individuals based on sensor input.

#### `sensor_data.py`
   - Handles acquisition and processing of sensor data.
   - Provides sensor input to the individuals simulating environmental interactions.

#### `individual.py`
   - Defines the `Individual` class for entities within the biome.
   - Represents entities that interact, make decisions, and evolve within the simulation.

#### `simulator.py`
   - Coordinates the simulation environment and evolutionary process.
   - Manages the lifecycle and evolution of individuals in the digital biome.

#### `genome.py`
   - Manages genetic information for the simulation individuals.
   - Influences behaviors and characteristics through genetic algorithms.

#### `bipedal_robot.py`
   - Simulates the physical aspects and behaviors of a bipedal robot.
   - Acts as the primary agent within the simulation, reacting and adapting to the biome.

#### `evolution.py`
   - Manages the genetic algorithm loop for evolutionary simulation.
   - Executes the evolution of individuals through selection, reproduction, and mutation.

#### `requirements.txt`
   - Lists the dependencies required for the simulation.

#### `setup_sephsbiome.sh`
   - Bash script for setting up the Python environment, including virtual environment creation and dependency installation.

## Usage
To set up and run the simulation:

1. Ensure Python 3.x is installed.
2. Execute the setup script to create a virtual environment and install dependencies:
   ```bash
   ./setup_sephsbiome.sh
   ```
3. Run the simulation using the `simulator.py` script:
   ```bash
   python simulator.py
   ```

## Dependencies
Ensure the following dependencies are installed:

- Python 3.x
- TensorFlow (version as per `requirements.txt`)
- NumPy (version as per `requirements.txt`)

Install all dependencies using `pip` within the virtual environment:

```bash
pip install -r requirements.txt
```
