import pandas as pd
import numpy as np
from datetime import datetime, timedelta
import os

def generate_synthetic_dataset():
    # Define the simulation parameters
    num_samples = 1000  # Adjust based on your requirement
    num_joints = 6  # Number of joints to simulate
    start_time = datetime.now()

    # Generate timestamps in a readable format
    timestamps = [start_time + timedelta(seconds=i) for i in range(num_samples)]
    formatted_timestamps = [ts.strftime('%Y-%m-%d %H:%M:%S') for ts in timestamps]

    # Simulate IMU data
    imu_data = {
        'linear_accel_x': np.random.uniform(-10, 10, num_samples),
        'linear_accel_y': np.random.uniform(-10, 10, num_samples),
        'linear_accel_z': np.random.uniform(-10, 10, num_samples),
        'angular_vel_x': np.random.uniform(-5, 5, num_samples),
        'angular_vel_y': np.random.uniform(-5, 5, num_samples),
        'angular_vel_z': np.random.uniform(-5, 5, num_samples)
    }

    # Simulate joint state data
    joint_state_data = {}
    for joint in range(num_joints):
        joint_state_data[f'joint_{joint}_position'] = np.random.uniform(-180, 180, num_samples)
        joint_state_data[f'joint_{joint}_velocity'] = np.random.uniform(-90, 90, num_samples)
        joint_state_data[f'joint_{joint}_effort'] = np.random.uniform(-50, 50, num_samples)

    # Combine all data into a DataFrame
    synthetic_data = pd.DataFrame({
        'timestamp': formatted_timestamps,
        **imu_data,
        **joint_state_data
    })

    # Save the dataset in the same folder as the script
    file_name = 'synthetic_sensor_dataset.csv'
    current_directory = os.getcwd()
    file_path = os.path.join(current_directory, file_name)

    synthetic_data.to_csv(file_path, index=False)
    print(f'Dataset saved to {file_path}')

    # Display the first 10 entries of the dataset
    print("First 10 entries of the dataset:")
    print(synthetic_data.head(10))

if __name__ == "__main__":
    generate_synthetic_dataset()
