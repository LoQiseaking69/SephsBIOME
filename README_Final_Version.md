
# Seph's Biome

## Overview

Seph's Biome is a cutting-edge project situated at the nexus of robotics, computer science, and evolutionary biology. Inspired by David Miller's bioSim project and Stephen D’Ascoli's thesis on symbolic Boolean regression, Seph's Biome explores the fascinating domain of artificial evolution, aiming to replicate and analyze complex biological behaviors in a computational setting.

### Inspirations
- **bioSim by David Miller**: [YouTube Channel](https://youtube.com/@davidrandallmiller?si=kAktZ_CpiCddPpU1)
- **Symbolic Boolean Regression by Stephen D’Ascoli**: [Thesis](https://sdascoli.github.io/) (DOI:2309.12207)

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

After following the setup instructions, the directory structure should look like this:

![Project Directory Structure](https://github.com/LoQiseaking69/SephsBIOME/blob/master/Docs/IMG_4622.jpg)

You will find the main runtime files within the ./src directory.

![Project Directory Structure](https://github.com/LoQiseaking69/SephsBIOME/blob/master/Docs/IMG_4617.jpg)

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

## Capabilities and Goals

Seph's Biome integrates advanced concepts from various fields to achieve the following capabilities and goals:

- **Robotic Simulation**: Utilizes ROS (Robot Operating System) for simulating realistic robotic behaviors and interactions within an environment or a simulation setup.
  
- **Evolutionary Algorithms**: Employs sophisticated genetic algorithms for evolving complex behaviors in robotic entities, inspired by natural evolutionary processes.
  
- **Neural Network Integration**: Incorporates neural networks to facilitate decision-making and adaptive learning, enabling robots to respond to environmental stimuli and improve over time.
  
- **Real-Time Data Handling**: Efficiently manages and processes sensor data, including IMU (Inertial Measurement Unit) and joint state information, crucial for real-time decision-making and performance analysis.

- **Dynamic Visualization**: Features a real-time performance visualization system, providing insights into key metrics like fitness, energy levels, and decision patterns, aiding in the understanding and analysis of the evolutionary process.

The overarching goal of Seph's Biome is to create a platform where robotic systems can evolve and adapt, mimicking biological processes and behaviors. This project stands as a testament to the potential of interdisciplinary research, blending robotics, AI, and biology to explore new frontiers in artificial evolution and intelligent systems.
