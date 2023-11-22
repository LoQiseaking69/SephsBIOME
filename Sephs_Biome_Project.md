
# Seph's Biome
## Overview
Seph's Biome is an attempt at integrating cutting-edge technologies and methodologies focusing on evolving a biome of mechanisms to enhance the kinematic abilities of bipedal robots. This project integrates evolutionary algorithms, neural networks, and sensor data processing to simulate and evolve robotic behaviors.

## Features
- **Evolutionary Algorithms**: Utilize genetic algorithms to evolve robot behaviors over generations.
- **Neural Network Modeling**: Implement and train neural networks to control and adapt robot movements.
- **ROS Integration**: Leverage the Robot Operating System for efficient robot management and simulation.
- **Performance Visualization**: Monitor and visualize the evolution of robot performance metrics.

## Getting Started
### Prerequisites
- Python 3.8 or 3.9
- ROS Noetic
- Docker (optional)

### Setup
Clone the repository and navigate to the project directory:

\```bash
git clone [repository-link]
cd Sephs-Biome
\```

### Using Docker
To build and run the project using Docker:

\```arduino
docker build -t sephsbiome .
docker run -it sephsbiome
\```

### Manual Setup
Run the setup script to install dependencies and set up the environment:

\```bash
./setup_sephsbiome.sh
\```

### Running the Project
After setup, you can start the project using:

\```css
python3 main.py
\```

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

For more information, visit our project page.
