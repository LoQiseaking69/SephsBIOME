
# Seph's Biome

## Overview

Seph's Biome is an ongoing, cutting-edge project at the intersection of robotics, computer science, and evolutionary biology. Inspired by David Miller's bioSim project and Stephen D’Ascoli's thesis on symbolic Boolean regression, this project delves into the world of artificial evolution, aiming to replicate and analyze complex biological behaviors within a computational framework.

### Inspirations
- **bioSim by David Miller**: [YouTube video](https://youtu.be/N3tRFayqVtk)
- **Symbolic Boolean Regression by Stephen D’Ascoli**: D'Ascoli, S. (2023). Boolformer: Symbolic Regression of Logic Functions with Transformers. Retrieved from [https://sdascoli.github.io/](https://sdascoli.github.io/)

## Getting Started

### Prerequisites
- Python - latest release 
- ROS Noetic (to be updated to Foxy)
- Docker (optional)

### Setup
Clone the repository and navigate to the directory:
```bash
git clone 'https://github.com/LoQiseaking69/SephsBIOME.git'
cd SephsBIOME
```
After setup, the directory structure should appear as follows:
![Project Directory Structure](https://github.com/LoQiseaking69/SephsBIOME/blob/master/Docs/IMG_4622.jpg)
Main runtime files are located in the `./src` directory.

### Using Docker
To build and run the project using Docker:
```bash
docker build -t sephsbiome .
docker run -it sephsbiome
```

### Manual Setup
Run the setup script:
```bash
./setup_sephsbiome.sh
```

### Running the Project
Start the project using:
```bash
python3 main.py
```

## Capabilities and Goals

Seph's Biome combines principles from robotics, AI, and biology to achieve:
- **Robotic Simulation**: Utilizing ROS for simulating realistic robotic behaviors in various environments or simulations.
- **Evolutionary Algorithms**: Implementing genetic algorithms to evolve complex robotic behaviors.
- **Neural Network Integration**: Incorporating neural networks for decision-making and adaptive learning.
- **Real-Time Data Handling**: Managing sensor data essential for real-time decisions and performance analysis.
- **Dynamic Visualization**: Providing real-time visualization of key metrics to aid in understanding and analyzing the evolutionary processes.

The overarching goal is to create a self-evolving robotic system, emulating biological processes and behaviors.
