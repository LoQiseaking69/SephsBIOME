#!/bin/bash

# Function to deactivate virtual environment and exit
cleanup_and_exit() {
    deactivate 2>/dev/null
    exit $1
}

# Function to check Python version supporting a range of versions
check_python_version() {
    PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    MIN_REQUIRED_VERSION="3.8"  # Minimum Python version required
    MAX_REQUIRED_VERSION="3.9"  # Maximum Python version supported

    if [[ $(echo -e "$PYTHON_VERSION\n$MIN_REQUIRED_VERSION" | sort -V | head -n1) != "$MIN_REQUIRED_VERSION" ]] || \
       [[ $(echo -e "$PYTHON_VERSION\n$MAX_REQUIRED_VERSION" | sort -V | tail -n1) != "$MAX_REQUIRED_VERSION" ]]; then
        echo "Python version between $MIN_REQUIRED_VERSION and $MAX_REQUIRED_VERSION is required."
        return 1
    else
        return 0
    fi
}

# Function to check if ROS is installed and the correct version
check_ros_installation() {
    if ! command -v roscore &> /dev/null; then
        echo "ROS is not installed. Please install ROS to continue."
        exit 1
    fi

    # Optionally, check for a specific ROS version here
}

# Function to install system dependencies
install_system_dependencies() {
    echo "This script will install system dependencies required for the project."
    echo "This includes ROS, TensorFlow, and other necessary packages."
    read -p "Do you want to proceed with installing these packages? (y/n) " -n 1 -r
    echo  # move to a new line

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Installing system dependencies..."
        sudo apt-get update
        sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
        sudo rosdep init
        rosdep update
    else
        echo "System dependencies installation skipped. Exiting script."
        exit 1
    fi
}

# Check and prompt for system-level dependencies installation
install_system_dependencies

# Check for Python 3.8 or 3.9
if ! command -v python3 &> /dev/null || ! check_python_version; then
    echo "Python 3.8 or 3.9 is required and could not be found."
    exit 1
fi

# Check for pip
if ! command -v pip &> /dev/null; then
    echo "pip could not be found. Ensure it is installed and in your PATH."
    exit 1
fi

# Check for ROS installation and version
check_ros_installation

# Create a Python virtual environment in the 'env' directory
VENV_DIR="sephsbiome-env"
echo "Creating Python virtual environment '$VENV_DIR'..."
python3 -m venv $VENV_DIR || { echo "Failed to create virtual environment."; exit 1; }

# Activate the virtual environment
echo "Activating the virtual environment..."
source $VENV_DIR/bin/activate || { echo "Failed to activate virtual environment."; cleanup_and_exit 1; }

# Upgrade pip in the virtual environment
echo "Upgrading pip..."
pip install --upgrade pip

# Check for requirements.txt file in the project directory
REQUIREMENTS_FILE="requirements.txt"
if [ ! -f $REQUIREMENTS_FILE ]; then
    echo "$REQUIREMENTS_FILE file not found in the project directory."
    cleanup_and_exit 1
fi

# Install requirements
echo "Installing requirements from $REQUIREMENTS_FILE..."
if ! pip install -r $REQUIREMENTS_FILE; then
    echo "Failed to install requirements."
    cleanup_and_exit 1
fi

echo "Setup complete. Virtual environment '$VENV_DIR' is ready."

# Setup is complete, now running the main program
echo "Running the main program..."
python3 main.py