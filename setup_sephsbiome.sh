#!/bin/bash

# Function to deactivate virtual environment and exit
cleanup_and_exit() {
    deactivate 2>/dev/null
    exit $1
}

# Function to check Python version
check_python_version() {
    PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    REQUIRED_VERSION="3.6"
    if [[ $(echo -e "$PYTHON_VERSION\n$REQUIRED_VERSION" | sort -V | head -n1) != "$REQUIRED_VERSION" ]]; then
        return 1
    else
        return 0
    fi
}

# Check for Python 3.6 or newer
if ! command -v python3 &> /dev/null || ! check_python_version; then
    echo "Python 3.6 or newer is required and could not be found."
    exit 1
fi

# Check for pip
if ! command -v pip &> /dev/null; then
    echo "pip could not be found. Ensure it is installed and in your PATH."
    exit 1
fi

# Create a Python virtual environment
echo "Creating Python virtual environment 'sephsbiome-env'..."
python3 -m venv sephsbiome-env || { echo "Failed to create virtual environment."; exit 1; }

# Activate the virtual environment
echo "Activating the virtual environment..."
source sephsbiome-env/bin/activate || { echo "Failed to activate virtual environment."; cleanup_and_exit 1; }

# Check for requirements.txt file
if [ ! -f requirements.txt ]; then
    echo "requirements.txt file not found."
    cleanup_and_exit 1
fi

# Install requirements
echo "Installing requirements from requirements.txt..."
if ! pip install -r requirements.txt; then
    echo "Failed to install requirements."
    cleanup_and_exit 1
fi

echo "Setup complete. Virtual environment 'sephsbiome-env' is ready."
echo "Activate it using 'source sephsbiome-env/bin/activate'."
echo "To deactivate, simply type 'deactivate'."