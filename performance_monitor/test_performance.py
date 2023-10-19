#!/usr/bin/env python

# Import required libraries
import rclpy
from rclpy.node import Node
from race_msgs.msg import PerformanceData
import subprocess
import matplotlib.pyplot as plt

# Define default constants and settings
SHOULD_PLOT = False  # Whether to plot data
SHOULD_PUBLISH = True  # Whether to publish the data
PUBLISH_TOPIC = "performance_data"  # Topic to publish on
FREQUENCY = 5  # Frequency (Hz) at which data will be published
MAX_PLOT_TIME = 6  # Maximum time for which to plot
ALL_PROCESS_STR = "ALL_PROCESSES"  # Keyword for all processes
ROS_PROCESS_STR = "ROS_PROCESSES"  # Keyword for ROS processes
IGNORE_ALL_PROCESS = True
IGNORE_ROS_PROCESS = True
ATTRIBUTES = ["user", "pid", "cpu", "mem", "vsz", "rss", "tty", "stat", "start", "time", "command"]  
MAX_PLOT_TIMESTEPS = MAX_PLOT_TIME * FREQUENCY  # Max plot time steps

class PerformanceNode(Node):
    def __init__(self):
        super().__init__('performance_node')
        
        # Update global variables with parameter values from a parameter file or defaults
        # This provides the ability to configure behavior through ROS2 parameterization.
        global SHOULD_PLOT, SHOULD_PUBLISH, PUBLISH_TOPIC, FREQUENCY
        global MAX_PLOT_TIME, ALL_PROCESS_STR, ROS_PROCESS_STR
        global IGNORE_ALL_PROCESS, IGNORE_ROS_PROCESS

        # Each of these lines declare a ROS2 parameter for the node and then update
        # the corresponding global variable. The "declare_parameter" function will
        # search for an existing parameter with the given name or use the default value.
        SHOULD_PLOT = self.declare_parameter("should_plot", SHOULD_PLOT).value
        SHOULD_PUBLISH = self.declare_parameter("should_publish", SHOULD_PUBLISH).value
        PUBLISH_TOPIC = self.declare_parameter("publish_topic", PUBLISH_TOPIC).value
        FREQUENCY = self.declare_parameter("frequency", FREQUENCY).value
        MAX_PLOT_TIME = self.declare_parameter("max_plot_time", MAX_PLOT_TIME).value
        ALL_PROCESS_STR = self.declare_parameter("all_process_str", ALL_PROCESS_STR).value
        ROS_PROCESS_STR = self.declare_parameter("ros_process_str", ROS_PROCESS_STR).value
        IGNORE_ALL_PROCESS = self.declare_parameter("ignore_all_process", IGNORE_ALL_PROCESS).value
        IGNORE_ROS_PROCESS = self.declare_parameter("ignore_ros_process", IGNORE_ROS_PROCESS).value

        # Initialize the publisher with the topic and the custom message type PerformanceData.
        # The "10" here refers to the size of the message queue.
        self.publisher_ = self.create_publisher(PerformanceData, PUBLISH_TOPIC, 10)

        # Create a timer for invoking the callback function based on the frequency parameter.
        self.timer_period = 1/FREQUENCY # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize dictionaries to store CPU and Memory usage statistics for each process.
        self.processes_cpu_usage = {}
        self.processes_mem_usage = {}

        # Initialize lists for ALL_PROCESS and ROS_PROCESS if they are not to be ignored.
        if(not IGNORE_ALL_PROCESS):
            self.processes_cpu_usage[ALL_PROCESS_STR] = []
            self.processes_mem_usage[ALL_PROCESS_STR] = []
        if(not IGNORE_ROS_PROCESS):
            self.processes_cpu_usage[ROS_PROCESS_STR] = []
            self.processes_mem_usage[ROS_PROCESS_STR] = []
        
        # Initialize plotting if enabled by the parameter.
        if (SHOULD_PLOT):
            plt.subplots(2, 1, figsize=(20, 12))
            self.timesteps = 1

    def timer_callback(self):
        # Collect current CPU and memory usage stats.
        track_data("cpu", self.processes_cpu_usage)
        track_data("mem", self.processes_mem_usage)
        
        # Plot the collected data if plotting is enabled.
        if (SHOULD_PLOT):
            plt.clf()
            plot_data("cpu", self.processes_cpu_usage, self.timesteps, plt.subplot(2, 1, 1))
            plot_data("mem", self.processes_mem_usage, self.timesteps, plt.subplot(2, 1, 2))
            plt.tight_layout()
            plt.pause(0.001)
            plt.draw()
            self.timesteps += 1 

        # Publish the CPU and memory usage information through a ROS2 message.
        # Populate the message fields with the latest stats collected.
        if (SHOULD_PUBLISH):
            msg = PerformanceData()
            msg.cpu_usage = []
            msg.cpu_usage_process = []
            msg.mem_usage = []
            msg.mem_usage_process = []
            for process, cpu_percent in self.processes_cpu_usage.items():
                msg.cpu_usage.append(cpu_percent[-1])
                msg.cpu_usage_process.append(process)
            for process, mem_percent in self.processes_mem_usage.items():
                msg.mem_usage.append(mem_percent[-1])
                msg.mem_usage_process.append(process)

            # Publish the message.
            self.publisher_.publish(msg)

# Function to retrieve a specific attribute from a line of process information
def get_line_attribute(attribute, line):
    # Tokenize the line by spaces, ignoring empty tokens
    tokens = [token for token in line.split(" ") if token]

    # Special case for the 'command' attribute since it can contain spaces
    # Example: If attribute is "command" and line is "root 123 1.2 4.5 ... /usr/bin/python3 my_script.py",
    # this will return "/usr/bin/python3 my_script.py"
    if attribute == "command":
        return ' '.join(tokens[ATTRIBUTES.index(attribute):])

    # For all other attributes, find the index of the attribute and return it
    # Example: If attribute is "cpu" and line is "root 123 1.2 4.5 ...",
    # this will return "1.2"
    if len(tokens) >= len(ATTRIBUTES):
        return tokens[ATTRIBUTES.index(attribute)]
    else:
        # Temporarily print and return 0 for any malformed lines
        print("RIPP:", len(line))
        return "0"

def extract_process_attribute(target_attribute, process_info_line):
    """
    Extracts a specific attribute from a line containing process information.
    
    Parameters:
    - target_attribute (str): The attribute to be extracted (e.g., 'command', 'cpu').
    - process_info_line (str): A string containing information about a process, separated by spaces.
    
    Returns:
    - str: The value of the specified attribute, if found.
    """
    
    # Tokenize the process information line by spaces, filtering out any empty tokens
    tokenized_line = [token for token in process_info_line.split(" ") if token]
    
    # If the target attribute is 'command', it's likely to contain spaces.
    # Thus, join all the relevant tokens to form the complete command.
    if target_attribute == "command":
        return ' '.join(tokenized_line[ATTRIBUTE_NAMES.index(target_attribute):])
    
    # For all other attributes, locate their index in the global ATTRIBUTE_NAMES list
    # and return the corresponding token from the tokenized line.
    if len(tokenized_line) >= len(ATTRIBUTE_NAMES):
        return tokenized_line[ATTRIBUTE_NAMES.index(target_attribute)]
    
    # Handle malformed lines by logging and returning a placeholder value.
    # Future versions may implement more robust error handling.
    print(f"Malformed Line Length: {len(process_info_line)}")
    return "0"


# # Function to get lines related to a specific process
# def find_process_lines(process_name, psaux):
#     """
#     Generates a list of lines from a psaux command output that are related to a specific process.
    
#     Parameters:
#         process_name (str): The name of the process to search for.
#         psaux (str): The output of the psaux command as a string.

#     Returns:
#         list: A list of lines from the psaux command output that are related to the specified process.
        
#     Examples:
#         find_process_lines("my_process", psaux_output)  # Returns lines related to "my_process"
#         find_process_lines(ALL_PROCESS_STR, psaux_output)  # Returns all lines excluding the header
#     """
#     # If the query is for all processes, return all lines except the header
#     if process_name == ALL_PROCESS_STR:
#         return [line for line in psaux.stdout.split("\n")[1:] if len(line) > 0]

#     # Find lines specifically related to ROS processes
#     ros_processes = [line for line in psaux.stdout.split("\n") if "node:=" in line or "/opt/ros/galactic" in line]

#     # If the query is for ROS processes, return the collected lines
#     if process_name == ROS_PROCESS_STR:
#         return ros_processes

#     # If the query is for a specific ROS process, filter by its name
#     return [line for line in ros_processes if process_name in get_line_attribute("command", line) and "ros" in get_line_attribute("command", line)]

def find_related_process_lines_in_psaux_output(
    target_process_name: str, 
    raw_psaux_output: str
) -> list:
    """
    Generates a list of lines from a psaux command output that are related to a specific process.
    
    Parameters:
        target_process_name (str): The name of the process to search for.
        raw_psaux_output (str): The output of the 'ps aux' command as a string.

    Returns:
        list[str]: A list of lines from the 'ps aux' output that are related to the specified process.
        
    Examples:
        find_related_process_lines_in_psaux_output("my_process", psaux_output)  
        # Returns lines related to "my_process"
        
        find_related_process_lines_in_psaux_output(ALL_PROCESS_STR, psaux_output)  
        # Returns all lines excluding the header
    """

    # Split the raw psaux output into lines
    all_psaux_lines = raw_psaux_output.stdout.split("\n")

    # If the query is for all processes, return all lines except the header
    if target_process_name == ALL_PROCESS_STR:
        return [line for line in all_psaux_lines[1:] if line]

    # Identify lines related to ROS processes
    ros_related_lines = [
        line for line in all_psaux_lines 
        if "node:=" in line or "/opt/ros/galactic" in line
    ]

    # If the query is for ROS processes, return the identified ROS-related lines
    if target_process_name == ROS_PROCESS_STR:
        return ros_related_lines

    # Filter ROS-related lines by the specific target process name
    filtered_ros_lines = [
        line for line in ros_related_lines
        if target_process_name in get_line_attribute("command", line) 
        and "ros" in get_line_attribute("command", line)
    ]

    return filtered_ros_lines

# Function to aggregate CPU usage or other attributes for processes with a given name
def aggregate_process_data(process_name, attribute, psaux):
    """
    Aggregates a specific attribute (like CPU usage) for all lines related to a given process name in psaux output.
    
    Parameters:
        process_name (str): The name of the process to aggregate data for.
        attribute (str): The attribute to aggregate (e.g., "cpu").
        psaux (str): The output of the psaux command as a string.

    Returns:
        float: The aggregated value of the specified attribute.
        
    Examples:
        aggregate_process_data("my_process", "cpu", psaux_output)  # Returns total CPU usage for "my_process"
    """
    cpu_usage = 0  # Initialize the aggregated CPU usage

    # Iterate through all lines related to the given process_name
    for line in find_related_process_lines_in_psaux_output(process_name, psaux):
        # Add up the CPU usage for each line
        cpu_usage += float(get_line_attribute(attribute, line))
    
    return cpu_usage  # Return the total CPU usage

# Function to identify all active ROS nodes
def find_nodes(psaux):
    """
    Identifies all active ROS nodes based on the psaux command output.
    
    Parameters:
        psaux (str): The output of the psaux command as a string.

    Returns:
        list: A list of identified ROS nodes.
        
    Examples:
        find_nodes(psaux_output)  # Returns a list of all active ROS nodes
    """


    nodes = []  # List to store found nodes

    # Iterate through each line of process data
    for line in psaux.stdout.split("\n"):
        # If the line indicates a ROS node, extract its name
        if "node:=" in line:
            cmd = get_line_attribute("command", line)
            node = cmd.split("node:=")[1].split(" ")[0]
            nodes.append(node)
        # If the line indicates a generic ROS process, extract its name
        elif "/opt/ros/galactic" in line:
            cmd = get_line_attribute("command", line)
            node = cmd.split("/opt/ros/galactic")[1]
            nodes.append(node)
    return nodes  # Return the list of found nodes  
    
def track_data(attribute, data_dict):
    """
    Track the specified attribute of system processes and update the data dictionary.
    
    Parameters:
        attribute (str): The attribute to track (e.g., 'cpu', 'memory').
        data_dict (dict): Dictionary mapping process names to a list of attribute values.
        
    Example Usage:
        track_data('cpu', {'my_process': [], 'another_process': []})
    """
    # Run the 'ps -aux' command to get details about all running processes
    psaux = subprocess.run("ps -aux", capture_output=True, text=True, shell=True) #TODO Can we just grep some parts of it?

    # Add new ROS nodes found in the 'ps -aux' output to data_dict
    for node in find_nodes(psaux):
        if node not in data_dict:
            data_dict[node] = []

    # For each process in data_dict, compute a new data point for the specified attribute
    for process, data in data_dict.items():
        new_data = aggregate_process_data(process, attribute, psaux)
        data.append(new_data)  # Append the new data point to the list of tracked values


def plot_data(attribute, data_dict, timesteps, ax):
    """
    Plot the tracked data for each process over time.
    
    Parameters:
        attribute (str): The attribute being plotted (e.g., 'cpu', 'memory').
        data_dict (dict): Dictionary mapping process names to a list of attribute values.
        timesteps (int): Total number of time points for which data has been collected.
        ax (matplotlib.axes.Axes): The matplotlib axes object to plot on.
        
    Example Usage:
        plot_data('cpu', {'my_process': [10, 20, 30], 'another_process': [5, 10, 15]}, 3, ax)
    """
    # Plot each data series in data_dict
    for process, data in data_dict.items():
        time_points = [t / FREQUENCY for t in range(timesteps - len(data), timesteps)]
        ax.plot(time_points[-MAX_PLOT_TIMESTEPS:], data[-MAX_PLOT_TIMESTEPS:], label=process, linewidth=2.5)

    # Sort the legend labels by the latest data point in descending order
    handles, labels = plt.gca().get_legend_handles_labels()
    sorted_labels = [item[0] for item in sorted(data_dict.items(), key=lambda item: item[1][-1] if item[1] else 0, reverse=True)]
    label_order = [labels.index(label) for label in sorted_labels]

    # Set axis labels and title
    ax.set_xlabel('time')
    ax.set_ylabel(attribute + '(%)')
    ax.set_title(attribute + ' over time')

    # Add legend with sorted labels
    ax.legend([handles[idx] for idx in label_order], [labels[idx] for idx in label_order], loc='upper left', bbox_to_anchor=(1.02, 1), ncol=1)


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
        


if __name__ == '__name__':
    main()


# for line in find_process_lines("ros"):
#     print(get_line_attribute("command", line))


# process_names = [proc.name() for proc in psutil.process_iter()]
# print("detected_object_feature_remover" in process_names)

# for proc in psutil.process_iter():
#     print("node" in proc.name())

# result = subprocess.run("ros2 node list", capture_output=True, text=True, shell=True)
# nodes = [node[1:] for node in result.stdout.split("\n")]
# for node in nodes:
#     print(node, any([1 for process in process_names if node in process]))
