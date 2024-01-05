#!/bin/bash

# Configuration for API Assistant
API_VENV_DIR="api-env"
API_SCRIPT="api.py"
API_LOG_FILE="api_assistant_log_$(date +%Y%m%d_%H%M%S).txt"
RETRY_MAX=3

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

# Start logging
exec > >(tee "$API_LOG_FILE") 2>&1

echo "Setting up API Assistant..."

# Check if virtual environment already exists
if [ -d "$API_VENV_DIR" ]; then
    print_warning "Virtual environment $API_VENV_DIR already exists. Activating environment."
else
    print_warning "Creating virtual environment..."
    python3 -m venv "$API_VENV_DIR" || error_exit "Failed to create virtual environment."
fi

# Activate virtual environment
source "$API_VENV_DIR/bin/activate" || error_exit "Failed to activate virtual environment."

# Upgrade pip
print_log "Upgrading pip..."
retry pip install --upgrade pip

# Install Flask and other dependencies
print_log "Installing Flask and other dependencies..."
retry pip install flask flask_jwt_extended requests

# Set up necessary environment variables
export SEPHSBIOME_DATABASE_URL='sephsbiome.db'
export JWT_SECRET_KEY='super-secret-key'
export GITHUB_TOKEN='your-github-token'
export SENTRY_AUTH_TOKEN='your-sentry-auth-token'
export SENTRY_ORG_SLUG='your-org'
export SENTRY_PROJECT_SLUG='your-project'

# Check if API script exists
if [ ! -f "$API_SCRIPT" ]; then
    error_exit "$API_SCRIPT not found. Please ensure it is in the current directory."
fi

print_success "API dependencies installed successfully."

# Run the API script in the background
print_log "Starting the API server..."
python "$API_SCRIPT" &

# Store the API process ID
API_PROCESS_ID=$!

# Deactivate the virtual environment on script exit
trap 'deactivate; kill $API_PROCESS_ID' EXIT

# Keep the script running
wait $API_PROCESS_ID