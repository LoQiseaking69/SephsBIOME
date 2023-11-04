# visualization.py
import matplotlib.pyplot as plt

# Assume simulator has been run and we have the data
fitness_over_time, decisions_over_time = ... # Get this data from the simulator

# Visualize the fitness over generations
plt.figure(figsize=(10, 5))
for i in range(len(fitness_over_time[0])):
    plt.plot([generation[i] for generation in fitness_over_time], label=f'Individual {i+1}')
plt.title('Fitness Over Generations')
plt.xlabel('Generation')
plt.ylabel('Fitness')
plt.legend()
plt.show()

# Visualize the decisions over generations
plt.figure(figsize=(10, 5))
for generation in range(len(decisions_over_time)):
    for decision in set(decisions_over_time[0]):
        plt.bar(generation, [dec.count(decision) for dec in decisions_over_time], label=f'Decision: {decision}')
plt.title('Decision Counts Over Generations')
plt.xlabel('Generation')
plt.ylabel('Count')
plt.legend()
plt.show()
