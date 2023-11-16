#!/bin/bash

# Function to deactivate virtual environment and exit
cleanup_and_exit() {
    deactivate 2>/dev/null
    exit $1
}

# Check for Python 3.6 or newer
if ! command -v python3 &> /dev/null || [[ $(python3 -c 'import sys; print(sys.version_info.major)') -lt 3 ]] || [[ $(python3 -c 'import sys; print(sys.version_info.minor)') -lt 6 ]]; then
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

# Install requirements
echo "Installing requirements from requirements.txt..."
if ! pip install -r requirements.txt; then
    echo "Failed to install requirements."
    cleanup_and_exit 1
fi

echo "Setup complete. Virtual environment 'sephsbiome-env' is ready."
echo "Activate it using 'source sephsbiome-env/bin/activate'."
echo "To deactivate, simply type 'deactivate'."