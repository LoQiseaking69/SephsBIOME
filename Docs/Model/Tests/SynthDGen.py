import pandas as pd
import numpy as np
from datetime import datetime, timedelta

def generate_synthetic_dataset(num_samples=1000, num_sensors=128, num_actions=10, noise_level=0.05):
    timestamps = [datetime.now() + timedelta(seconds=i) for i in range(num_samples)]
    data = {'timestamp': [timestamp.strftime('%Y-%m-%d %H:%M:%S') for timestamp in timestamps]}
    
    for i in range(num_sensors):
        sensor_data = np.random.uniform(-1, 1, num_samples)
        noise = np.random.normal(0, noise_level, num_samples)
        data[f'sensor_{i}'] = sensor_data + noise

    data['action'] = np.random.randint(0, num_actions, num_samples)
    data['reward'] = np.random.uniform(-1, 1, num_samples)

    return pd.DataFrame(data)

def normalize_column(column):
    return (column - column.mean()) / column.std()

def feature_engineering(dataset, num_sensors=128, window_size=5):
    new_features = {}
    
    for i in range(num_sensors):
        col_name = f'sensor_{i}'
        dataset[col_name] = normalize_column(dataset[col_name])
        new_features[f'{col_name}_diff'] = dataset[col_name].diff().fillna(0)
        new_features[f'{col_name}_roll_avg'] = dataset[col_name].rolling(window=window_size).mean().fillna(0)
        new_features[f'{col_name}_exp_mov_avg'] = dataset[col_name].ewm(span=window_size).mean().fillna(0)

    for i in range(0, num_sensors, 2):
        if i + 1 < num_sensors:
            new_features[f'sensor_{i}_{i+1}_interaction'] = dataset[f'sensor_{i}'] * dataset[f'sensor_{i+1}']

    new_feature_df = pd.DataFrame(new_features)
    return pd.concat([dataset, new_feature_df], axis=1)

def reorganize_for_rl(dataset, num_sensors=128):
    sensor_columns = [f'sensor_{i}' for i in range(num_sensors)]
    sensor_diff_columns = [f'sensor_{i}_diff' for i in range(num_sensors)]
    sensor_roll_avg_columns = [f'sensor_{i}_roll_avg' for i in range(num_sensors)]
    sensor_exp_mov_avg_columns = [f'sensor_{i}_exp_mov_avg' for i in range(num_sensors)]
    
    dataset['sensor_mean'] = dataset[sensor_columns].mean(axis=1)
    dataset['sensor_std'] = dataset[sensor_columns].std(axis=1)
    dataset['sensor_diff_mean'] = dataset[sensor_diff_columns].mean(axis=1)
    dataset['sensor_roll_avg_mean'] = dataset[sensor_roll_avg_columns].mean(axis=1)
    dataset['sensor_exp_mov_avg_mean'] = dataset[sensor_exp_mov_avg_columns].mean(axis=1)
    
    # Drop individual sensor columns to simplify the dataset
    drop_columns = sensor_columns + sensor_diff_columns + sensor_roll_avg_columns + sensor_exp_mov_avg_columns
    dataset.drop(columns=drop_columns, inplace=True)
    
    return dataset

# Generate the dataset
synthetic_dataset = generate_synthetic_dataset()

# Apply feature engineering
engineered_dataset = feature_engineering(synthetic_dataset)

# Reorganize dataset for RL
rl_dataset = reorganize_for_rl(engineered_dataset)

# Display the first 10 rows of the reorganized dataset
print(rl_dataset.head(10))

# Save the reorganized dataset to a CSV file
file_path = 'synthetic_datasetV2.csv'
rl_dataset.to_csv(file_path, index=False)

print(f'Dataset saved at {file_path}')
