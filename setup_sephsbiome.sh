
#!/bin/bash

# Check for Python 3
if ! command -v python3 &> /dev/null; then
    echo "Python 3 could not be found"
    exit 1
fi

# Check for pip
if ! command -v pip &> /dev/null; then
    echo "pip could not be found"
    exit 1
fi

# Create a Python virtual environment
python3 -m venv sephsbiome-env

# Activate the virtual environment
source sephsbiome-env/bin/activate

# Install requirements
pip install -r requirements.txt

echo "Setup complete. Virtual environment 'sephsbiome-env' is ready."
echo "Activate it using 'source sephsbiome-env/bin/activate'."
