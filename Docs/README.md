# Seph's Biome

## Overview

Seph’s Biome is an attempt at innovative project that stands at the intersection of robotics, computer science, and evolutionary biology. Drawing inspiration and fundamental principles from the bioSim project by David Miller

‘’’
(https://youtube.com/@davidrandallmiller?si=kAktZ_CpiCddPpU1)
‘’’

 and the seminal thesis on symbolic Boolean regression by Stephen D’Ascoli (https://sdascoli.github.io/)
 
 (DOI:2309.12207). 

This project embarks on a journey to evolve kinematic abilities in bipedal robots through sophisticated algorithms and neural network modeling.

### Key Aspects:

- **Evolutionary Algorithms**: At its core, Seph’s Biome leverages evolutionary algorithms as a driving force for the continuous improvement and adaptation of robotic behaviors. These algorithms, inspired by natural selection, are designed to simulate and enhance the evolutionary processes in a controlled environment, leading to increasingly effective robotic motion and decision-making capabilities.
- **Symbolic Boolean Regression**: Building on the concepts presented in D’Ascoli’s thesis, the project integrates symbolic Boolean regression techniques to refine the decision-making processes. This approach allows the system to evolve and optimize complex logical functions, enhancing the robot’s ability to interact with and adapt to varying environments.
- **Neural Network Modeling**: Neural networks are employed to mimic the learning and processing capabilities of the human brain, offering dynamic control over the robots’ movements. The models are continuously refined through iterative learning, drawing parallels with the way living organisms adapt and evolve.
- **Robot Operating System (ROS) Integration**: Utilizing ROS, Seph’s Biome achieves a high level of modularity and flexibility, enabling seamless integration and communication between various robotic components and systems.

### Project Goals:

The primary aim of Seph’s Biome is to push the boundaries of robotic capabilities, making them more adaptive, efficient, and intelligent. By combining the principles of evolutionary biology with advanced computational techniques, the project seeks to not only enhance robotic performance but also contribute to our understanding of evolutionary processes and artificial intelligence.


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
