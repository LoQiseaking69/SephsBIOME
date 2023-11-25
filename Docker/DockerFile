# Use an official Python runtime as a parent image, ensuring compatibility with Python 3.8
FROM python:3.8-slim

# Set a non-root user for security purposes
RUN useradd -m appuser

# Set the working directory in the container
WORKDIR /usr/src/app

# Install system dependencies in a more efficient manner
# Also segregating ROS-specific and other system dependencies for clarity
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    curl \
    lsb-release \
    gnupg2 && \
    apt-get install -y --no-install-recommends \
    libasound2-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    ffmpeg \
    libopencv-dev \
    python3-opencv && \
    apt-get install -y --no-install-recommends \
    nvidia-driver-latest \
    nvidia-container-toolkit \
    libi2c-dev \
    i2c-tools && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install and setup ROS (replace 'noetic' with your ROS version if necessary)
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-sound-play \
    ros-noetic-audio-common \
    ros-noetic-speech-recognition-msgs \
    ros-noetic-pocketsphinx \
    ros-noetic-opencv-apps \
    ros-noetic-image-transport-plugins \
    ros-noetic-robot-localization \
    ros-noetic-move-base \
    ros-noetic-navigation \
    ros-noetic-sensor-msgs \
    ros-noetic-control-msgs \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-trajectory-controller && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup the ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/appuser/.bashrc

# Copy only the requirements.txt initially to leverage Docker cache
COPY requirements.txt ./

# Upgrade pip and install Python dependencies from requirements.txt
RUN pip install --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Now copy the rest of the application source code
COPY . .

# Change ownership of the working directory to the non-root user
RUN chown -R appuser:appuser /usr/src/app

# Switch to non-root user
USER appuser

# Expose the port the app runs on, if necessary
EXPOSE 8000

# Define environment variable
ENV NAME SephsBiome

# Add an argument to control whether to run setup_sephsbiome.sh
ARG RUN_SETUP_SCRIPT=false

# Conditional execution of the setup script
RUN if [ "$RUN_SETUP_SCRIPT" = "true" ] ; then ./setup_sephsbiome.sh ; fi

# Run app.py when the container launches
CMD ["python3", "./main.py"]