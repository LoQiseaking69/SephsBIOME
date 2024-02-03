import pandas as pd
import numpy as np
from datetime import datetime, timedelta
import random
import string

# Define a list of words for NLP data generation
words = ["alpha", "beta", "gamma", "delta", "epsilon", "zeta", "eta", "theta", "iota", "kappa"]

def generate_synthetic_dataset(num_samples=1000, num_sensors=128, num_actions=10, noise_level=0.05):
    timestamps = [datetime.now() + timedelta(seconds=i) for i in range(num_samples)]
    data = {'timestamp': [timestamp.strftime('%Y-%m-%d %H:%M:%S') for timestamp in timestamps]}
    
    # Enhanced NLP data generation
    data['text_data'] = [' '.join(random.choices(words, k=5)) for _ in range(num_samples)]
    
    for i in range(num_sensors):
        # Introduce periodic trends in sensor data
        period = np.random.randint(10, 100)
        sensor_data = np.sin(np.linspace(0, 2 * np.pi * period, num_samples))
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
        # Normalizing the entire column after computing diff, rolling, and ewm
        normalized_col = normalize_column(dataset[col_name])
        new_features[f'{col_name}_diff'] = normalized_col.diff().fillna(0)
        new_features[f'{col_name}_roll_avg'] = normalized_col.rolling(window=window_size).mean().fillna(0)
        new_features[f'{col_name}_exp_mov_avg'] = normalized_col.ewm(span=window_size).mean().fillna(0)

    new_feature_df = pd.DataFrame(new_features)
    return pd.concat([dataset, new_feature_df], axis=1)

def reorganize_for_rl(dataset, num_sensors=128):
    sensor_columns = [f'sensor_{i}' for i in range(num_sensors)]
    
    # Balancing the dataset for actions and rewards
    action_counts = dataset['action'].value_counts()
    min_count = action_counts.min()
    balanced_dataset = pd.concat([dataset[dataset['action'] == action].sample(min_count) for action in action_counts.index])
    
    # Simplifying dataset
    balanced_dataset['sensor_mean'] = balanced_dataset[sensor_columns].mean(axis=1)
    balanced_dataset['sensor_std'] = balanced_dataset[sensor_columns].std(axis=1)
    
    # Drop less relevant columns to focus on key features
    drop_columns = sensor_columns
    balanced_dataset.drop(columns=drop_columns, inplace=True)
    
    return balanced_dataset

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