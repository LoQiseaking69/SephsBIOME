
# Seph's Biome
## Overview
Seph's Biome aims to integrate cutting-edge technologies and a diverse set of algorythmically advanced methodololgies; to evolve, kinematic abilities in bipedal robots.

## Features
- **Evolutionary Algorithms**: Employ genetic algorithms for the progressive evolution of robot behaviors.
- **Neural Network Modeling**: Develop and refine neural networks for dynamic robot movement control.
- **ROS Integration**: Utilize the Robot Operating System (ROS) for streamlined robot management.
- **Performance Visualization**: Implement tools to monitor and display robot performance metrics.

## Getting Started
### Prerequisites
- Python 3.8 or 3.9
- ROS Noetic
- Docker (optional)

### Setup
To set up the project, clone the repository and navigate to its directory:

```bash
git clone 'https://github.com/LoQiseaking69/SephsBIOME.git'
cd SephsBIOME
```

### Using Docker
To build and run the project using Docker:

```bash
docker build -t sephsbiome .
docker run -it sephsbiome
```

### Manual Setup
Run the setup script to install dependencies and set up the environment:

```bash
./setup_sephsbiome.sh
```

### Running the Project
After setup, you can start the project using:

```bash
python3 main.py
```

## Project Structure
- `bipedal_robot.py`: Manages bipedal robot mechanics and ROS integration.
- `evolution.py`: Handles evolutionary processes and interfaces with ROS.
- `genome.py`: Defines the genetic structure and mutation logic.
- `individual.py`: Represents individual robots with genomes and neural networks.
- `main.py`: Central execution point for simulations and evolutionary processes.
- `neural_network.py`: Manages neural network models.
- `performanceViz.py`: Visualizes performance metrics.
- `sensor_data.py`: Processes real-time sensor data.
- `simulator.py`: Simulates robot behavior and runs genetic algorithms.

## Contributing
We welcome contributions to the Seph's Biome project. Please read our contributing guidelines before submitting pull requests.

## License
This project is licensed under the MPL 2.0 License - see the LICENSE.md file for details.

## Acknowledgments
- Contributors and community members who have contributed to this project.
- Organizations supporting the research and development of advanced robotics.

For more information, visit our project page. (To be Established)
