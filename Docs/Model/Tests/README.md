
# Model Simulation Script

## Overview
`modelSim.py` is a Python script designed to simulate a neural network model's decision-making process using sensor data. This script is particularly useful for understanding how a model might interpret and act upon a sequence of inputs from various sensors.

## How it Works
The script performs several key operations to simulate model behavior:

1. **Data Loading**: Reads in a dataset, assuming the last column to be the target (reward) and all others as features (sensor readings).

2. **Feature Normalization**: Scales the features to have a mean of zero and a standard deviation of one.

3. **Positional Encoding Simulation**: Adds synthetic positional information to the input features to mimic the sequence processing of transformer models.

4. **Dimensionality Reduction**: Applies Principal Component Analysis (PCA) to reduce the feature space and simulate the data combination of a transformer's encoder.

5. **Boolformer Simulation**: Applies a threshold to the reduced features to simulate the behavior of a Boolformer layer, which processes inputs based on boolean logic.

6. **Q-Learning Simulation**: Simulates the reinforcement learning aspect by updating Q-values based on the actions taken and rewards received.

7. **Action and Reward Prediction**: Uses the simulated Q-values to predict actions and employs a regression model to predict rewards.

8. **Result Visualization**: Generates plots to visualize the distribution of predicted actions, the comparison of actual vs. predicted rewards, the Q-values heatmap, the PCA components scatter plot, and the correlation heatmap of the transformed features.

## Requirements
The script requires the following Python libraries:
- NumPy
- Pandas
- Matplotlib
- Seaborn
- Scikit-learn

## Usage
To run the script, Ensure pandas is in the environment; and, pyarrow soon (as i've noticed with package updates...) and an up-to-date version of the  `'synthetic_datasetV2.csv'`
generated csv file, is within the function tuple for: `pd.read_csv('synthetic_datasetV2.csv')`

## Visualizations Produced
- Histogram of Predicted Actions: Shows the frequency of each action chosen by the simulation.
- Actual vs. Predicted Rewards Plot: Compares the actual rewards from the dataset with those predicted by the simulation.
- Heatmap of Q-Values: Displays the Q-values for each action across sequences.
- PCA Component Scatter Plot: Visualizes the principal components obtained from the dimensionality reduction step.
- Correlation Heatmap: Shows correlations between features after applying the Boolformer layer logic.

The visualizations help in interpreting the model's behavior and the effectiveness of the simulation.
