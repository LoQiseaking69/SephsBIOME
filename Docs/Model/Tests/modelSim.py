import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression

  # This script, modelSim.py, simulates a model's decision-making process based on sensor data.

def main():
      # Load the dataset
      dataset = pd.read_csv('synthetic_datasetV2.csv')

      # Automatically detect sensor columns (numeric only) and the reward column
      numeric_features = dataset.select_dtypes(include=[np.number])
      input_features = numeric_features.iloc[:, :-1].values
      rewards = numeric_features.iloc[:, -1].values

      # Normalize input features
      scaler = StandardScaler()
      input_data_normalized = scaler.fit_transform(input_features)

      # Simulate Positional Encoding
      def simulate_positional_encoding(length, dimension):
          pos = np.arange(length)[:, np.newaxis]
          dim = np.arange(dimension)[np.newaxis, :]
          angle_rates = 1 / np.power(10000, (2 * (dim // 2)) / np.float32(dimension))
          angle_rads = pos * angle_rates
          angle_rads[:, 0::2] = np.sin(angle_rads[:, 0::2])
          angle_rads[:, 1::2] = np.cos(angle_rads[:, 1::2])
          return angle_rads

      # Apply positional encoding
      length, dimension = input_data_normalized.shape
      positional_encoding = simulate_positional_encoding(length, dimension)
      input_data_with_pos = input_data_normalized + positional_encoding

      # PCA for dimensionality reduction
      pca = PCA(n_components=min(length, dimension))
      transformed_data = pca.fit_transform(input_data_with_pos)

      # Boolformer logic simulation
      threshold = 0.5
      boolformed_data = transformed_data > threshold

      # Initialize Q-values for Q-Learning simulation
      action_space_size = 10  # Replace with actual action space size
      q_values = np.random.rand(length, action_space_size)

      # Simulate Q-Learning updates
      for i in range(1, length):
          action = np.argmax(q_values[i - 1])
          q_values[i, action] += 0.01 * (rewards[i - 1] + 0.95 * np.max(q_values[i]) - q_values[i, action])

      # Predict actions and rewards
      predicted_actions = np.argmax(q_values, axis=1)
      model = LinearRegression()
      model.fit(boolformed_data, rewards)
      predicted_rewards = model.predict(boolformed_data)

      # Scale predicted rewards
      predicted_rewards_scaled = (predicted_rewards - predicted_rewards.mean()) / predicted_rewards.std()
      predicted_rewards_scaled *= rewards.std()
      predicted_rewards_scaled += rewards.mean()

      # Visualizations
      plt.figure(figsize=(18, 10))

      # Histogram of Predicted Actions
      plt.subplot(2, 3, 1)
      sns.histplot(predicted_actions, bins=action_space_size, kde=False)
      plt.title('Histogram of Predicted Actions')

      # Actual vs. Predicted Rewards
      plt.subplot(2, 3, 2)
      plt.plot(rewards[:100], label='Actual Rewards')
      plt.plot(predicted_rewards_scaled[:100], label='Predicted Rewards', linestyle='--')
      plt.title('Actual vs. Predicted Rewards')
      plt.legend()

      # Heatmap of Q-Values
      plt.subplot(2, 3, 3)
      sns.heatmap(q_values[:100, :], cmap='viridis')
      plt.title('Heatmap of Q-Values')

      # PCA Component Scatter Plot
      plt.subplot(2, 3, 4)
      sns.scatterplot(x=transformed_data[:, 0], y=transformed_data[:, 1])
      plt.title('PCA Component Scatter Plot')

      # Correlation Heatmap of Boolformed Features
      plt.subplot(2, 3, 5)
      correlation_matrix = np.corrcoef(boolformed_data.T)
      sns.heatmap(correlation_matrix, cmap='coolwarm')
      plt.title('Correlation Heatmap of Boolformed Features')

      plt.tight_layout()
      plt.show()
 

if __name__ == "__main__":
    main()
