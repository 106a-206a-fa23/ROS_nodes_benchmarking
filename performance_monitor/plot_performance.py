#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from race_msgs.msg import PerformanceData

import matplotlib.pyplot as plt
import time


SUBSCRIBE_TOPIC = "performance_data"
MAX_PLOT_TIMESTEPS = 30


class PlotPerformanceNode(Node):
    def __init__(self):
        super().__init__('plot_performance_node')
        self.subscribter_ = self.create_subscription(PerformanceData, SUBSCRIBE_TOPIC, self.sub_callback, 10)  # TODO change depth?

        self.processes_cpu_usage = {}
        self.processes_mem_usage = {}
        self.time_points = []

        plt.subplots(2, 1, figsize=(20, 12))  # set figure size
        self.timesteps = 1


    def sub_callback(self, msg):
        if (not self.processes_cpu_usage):
            self.init_time = time.time()

        self.time_points.append(time.time() - self.init_time)

        for i in range(len(msg.cpu_usage_process)):
            process = msg.cpu_usage_process[i]
            cpu_usage = msg.cpu_usage[i]
            if (process not in self.processes_cpu_usage):
                self.processes_cpu_usage[process] = []
            self.processes_cpu_usage[process].append(cpu_usage)
        for i in range(len(msg.mem_usage_process)):
            process = msg.mem_usage_process[i]
            mem_usage = msg.mem_usage[i]
            if (process not in self.processes_mem_usage):
                self.processes_mem_usage[process] = []
            self.processes_mem_usage[process].append(mem_usage)
        plt.clf()  # clear
        plot_data("cpu", self.processes_cpu_usage, self.time_points, plt.subplot(2, 1, 1))
        plot_data("mem", self.processes_mem_usage, self.time_points, plt.subplot(2, 1, 2))
        plt.tight_layout()  # make sure legend is visible cuz it gets huge
        plt.pause(0.001)  # pause to draw plot
        plt.draw()



def plot_data(attribute, data_dict, time_points, ax):
    # plot data points
    for process, data in data_dict.items():
        data_cropped = data[-MAX_PLOT_TIMESTEPS:]
        time_cropped = time_points[-len(data_cropped):]
        ax.plot(time_cropped, data_cropped, label=process, linewidth=2.5)


    # for label sorting (pain)
    handles, labels = plt.gca().get_legend_handles_labels()
    sorted_labels = [item[0] for item in sorted(list(data_dict.items()), key=lambda item:item[1][-1], reverse=True) if item[1] and item[1][-1]]
    label_order = [labels.index(label) for label in sorted_labels]

    ax.set_xlabel('time')
    ax.set_ylabel(attribute + '(%)')
    ax.set_title(attribute + ' over time')
    ax.legend([handles[idx] for idx in label_order], [labels[idx] for idx in label_order], loc='upper left', bbox_to_anchor=(1.02, 1), ncol=1)


def main(args=None):
    rclpy.init(args=args)
    node = PlotPerformanceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
        


if __name__ == '__name__':
    main()