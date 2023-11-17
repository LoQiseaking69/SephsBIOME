import rospy
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray
from collections import deque
from threading import Thread

class PerformanceVisualizer:
    def __init__(self, buffer_size=100, refresh_rate=0.1):
        rospy.init_node('performance_visualizer')
        
        self.buffer_size = buffer_size
        self.refresh_rate = refresh_rate
        self.data_buffers = {
            'fitness': deque(maxlen=buffer_size),
            'energy': deque(maxlen=buffer_size),
            'health': deque(maxlen=buffer_size),
            'decision': deque(maxlen=buffer_size)
        }

        self.init_subscribers()
        Thread(target=self.visualize_data).start()

    def init_subscribers(self):
        rospy.Subscriber('/fitness_data', Float32MultiArray, self.data_callback, callback_args='fitness')
        rospy.Subscriber('/energy_data', Float32MultiArray, self.data_callback, callback_args='energy')
        rospy.Subscriber('/health_data', Float32MultiArray, self.data_callback, callback_args='health')
        rospy.Subscriber('/decision_data', Float32MultiArray, self.data_callback, callback_args='decision')

    def data_callback(self, data, data_type):
        try:
            if data.data:
                self.data_buffers[data_type].append(data.data)
        except Exception as e:
            rospy.logwarn(f"Error in data_callback: {e}")

    def visualize_data(self):
        plt.ion()
        fig, axs = plt.subplots(4, 1, figsize=(12, 10))
        plot_titles = ['Fitness Over Time', 'Energy Levels Over Time', 'Health Scores Over Time', 'Decision Patterns Over Time']
        data_types = ['fitness', 'energy', 'health', 'decision']

        while not rospy.is_shutdown():
            for i, data_type in enumerate(data_types):
                if self.data_buffers[data_type]:
                    axs[i].clear()
                    axs[i].plot(np.array(self.data_buffers[data_type]), label=data_type.capitalize())
                    axs[i].set_title(plot_titles[i])
                    axs[i].legend()
                    axs[i].grid(True)
                    axs[i].set_xlabel('Time')
                    axs[i].set_ylabel(data_type.capitalize())

            plt.pause(self.refresh_rate)

        plt.ioff()
        plt.show()

if __name__ == '__main__':
    visualizer = PerformanceVisualizer()
    rospy.spin()