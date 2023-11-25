#!/bin/bash

# Default configurations
VENV_DIR_DEFAULT="sephsbiome-env"
REQUIRED_ROS_VERSION_DEFAULT="noetic"
MIN_PYTHON_VERSION_DEFAULT="3.8"
MAX_PYTHON_VERSION_DEFAULT="3.9"
LOG_FILE="setup_log.txt"
RETRY_MAX=3

# Parse command-line arguments for configuration
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -v|--venv) VENV_DIR="$2"; shift ;;
        -r|--ros-version) REQUIRED_ROS_VERSION="$2"; shift ;;
        -p|--python-min) MIN_PYTHON_VERSION="$2"; shift ;;
        -P|--python-max) MAX_PYTHON_VERSION="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Use default values if not set by arguments
VENV_DIR=${VENV_DIR:-$VENV_DIR_DEFAULT}
REQUIRED_ROS_VERSION=${REQUIRED_ROS_VERSION:-$REQUIRED_ROS_VERSION_DEFAULT}
MIN_PYTHON_VERSION=${MIN_PYTHON_VERSION:-$MIN_PYTHON_VERSION_DEFAULT}
MAX_PYTHON_VERSION=${MAX_PYTHON_VERSION:-$MAX_PYTHON_VERSION_DEFAULT}

# Start logging
exec > >(tee "$LOG_FILE") 2>&1

# Function definitions
print_success() {
    echo -e "\033[0;32m$1\033[0m"
}

print_warning() {
    echo -e "\033[0;33m$1\033[0m"
}

error_exit() {
    echo -e "\033[0;31m$1\033[0m" >&2
    exit 1
}

confirm_action() {
    read -p "$1 (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        return 0
    else
        print_warning "Action skipped by user."
        return 1
    fi
}

retry() {
    local n=1
    until [ "$n" -ge "$RETRY_MAX" ]; do
        "$@" && break || {
            if [ "$n" -lt "$RETRY_MAX" ]; then
                n=$((n+1))
                print_warning "Attempt $n/$RETRY_MAX failed! Trying again in 5 seconds..."
                sleep 5
            else
                error_exit "The command has failed after $n attempts."
            fi
        }
    done
}

# Check Python version
check_python_version() {
    PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
    if [[ $(echo -e "$PYTHON_VERSION\n$MIN_PYTHON_VERSION" | sort -V | head -n1) != "$MIN_PYTHON_VERSION" ]] || \
       [[ $(echo -e "$PYTHON_VERSION\n$MAX_PYTHON_VERSION" | sort -V | tail -n1) != "$MAX_PYTHON_VERSION" ]]; then
        error_exit "Python version between $MIN_PYTHON_VERSION and $MAX_PYTHON_VERSION is required. Found: $PYTHON_VERSION"
    else
        print_success "Compatible Python version found: $PYTHON_VERSION"
    fi
}

# Check ROS installation
check_ros_installation() {
    if ! command -v roscore &> /dev/null; then
        error_exit "ROS is not installed. Please install ROS to continue."
    else
        INSTALLED_ROS_VERSION=$(rosversion -d)
        if [ "$INSTALLED_ROS_VERSION" != "$REQUIRED_ROS_VERSION" ]; then
            error_exit "ROS $REQUIRED_ROS_VERSION is required. Found ROS $INSTALLED_ROS_VERSION."
        else
            print_success "Correct ROS version found: $INSTALLED_ROS_VERSION"
        fi
    fi
}

# Install system dependencies
install_system_dependencies() {
    if confirm_action "Install system dependencies required for the project?"; then
        DEPENDENCIES="python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential"
        retry sudo apt-get update
        retry sudo apt-get install -y $DEPENDENCIES
        sudo rosdep init
        rosdep update
        sudo apt-get clean
        sudo rm -rf /var/lib/apt/lists/*
        print_success "System dependencies installed successfully."
    fi
}

# Install Python dependencies
install_python_dependencies() {
    if confirm_action "Create a virtual environment and install Python dependencies?"; then
        if [ -d "$VENV_DIR" ]; then
            print_warning "Virtual environment $VENV_DIR already exists. Skipping creation."
        else
            python3 -m venv $VENV_DIR || error_exit "Failed to create virtual environment."
        fi

        source $VENV_DIR/bin/activate || error_exit "Failed to activate virtual environment."
        retry pip install --upgrade pip

        REQUIREMENTS_FILE="requirements.txt"
        if [ ! -f "$REQUIREMENTS_FILE" ]; then
            error_exit "$REQUIREMENTS_FILE not found in the project directory."
        fi

        retry pip install -r $REQUIREMENTS_FILE
        print_success "Python dependencies installed successfully."
    fi
}

# Main execution
check_python_version
check_ros_installation
install_system_dependencies
install_python_dependencies

print_success "Setup complete. Virtual environment '$VENV_DIR' is ready to use."