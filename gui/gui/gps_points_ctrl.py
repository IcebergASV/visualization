# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy
# from geometry_msgs.msg import PoseStamped
# from geographic_msgs.msg import GeoPoseStamped
# from mavros_msgs.msg import State
# from sensor_msgs.msg import NavSatFix
# from std_msgs.msg import String
# import tkinter as tk

# class GUI(tk.Tk):
#     def __init__(self):
#         super().__init__()
#         self.title("Debug GUI")
#         self.geometry("600x250")
#         self.grid_labels = []
#         self.status_labels = {}

#     def add_label(self, text):
#         label = tk.Label(self, text=text, font=("Arial", 14), anchor="w", justify="left")
#         label.grid(row=len(self.grid_labels), column=0, pady=5, sticky="w")  # Left column
#         self.grid_labels.append(label)
#         return label

#     def update_label(self, label, text, flash=True):
#         label.config(text=text)
#         if flash:
#             label.config(fg="green")
#             self.after(500, lambda: label.config(fg="black"))
#         else:
#             label.config(fg="black")

#     def add_status_label(self, node_name, row):
#         node_label = tk.Label(self, text=node_name, font=("Arial", 14))
#         node_label.grid(row=row, column=1, sticky="w", padx=10)  # Right column, align left
#         status_label = tk.Label(self, text="●", font=("Arial", 14), fg="red")
#         status_label.grid(row=row, column=2, padx=5)  # Status indicator in rightmost column
#         self.status_labels[node_name] = status_label

#     def update_status(self, node_name, is_running):
#         if node_name in self.status_labels:
#             color = "green" if is_running else "red"
#             self.status_labels[node_name].config(fg=color)

# class SetpointNode(Node):
#     def __init__(self, gui):
#         super().__init__('setpoint_listener')
#         self.gui = gui
        
#         # Define a QoS profile
#         qos_profile = QoSProfile(depth=10)
#         qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
#         self.target_nodes = ["rosbag2_recorder", "maneuvering", "camera", "mavros", "yolov8_node"]

#         # Set up node status labels
#         for idx, node_name in enumerate(self.target_nodes, start=len(gui.grid_labels)):
  
#             gui.add_status_label(node_name, row=idx)

#         # Set up timer to check node statuses every second
#         self.node_check_timer = self.create_timer(1.0, self.check_node_statuses)
    
#         # Last message time tracking
#         self.last_local_setpoint_time = self.get_clock().now()
#         self.last_global_setpoint_time = self.get_clock().now()
#         self.last_local_position_time = self.get_clock().now()
#         self.last_global_position_time = self.get_clock().now()
#         self.last_mode_time = self.get_clock().now()
#         self.last_status_time = self.get_clock().now()

#         self.message_check_timer = self.create_timer(1.0, self.check_message_statuses)

#         # Subscription for Local Setpoint
#         self.local_subscription = self.create_subscription(
#             PoseStamped,
#             '/mavros/setpoint_position/local',
#             self.local_callback,
#             qos_profile
#         )
        
#         # Subscription for Global Setpoint
#         self.global_subscription = self.create_subscription(
#             NavSatFix,
#             '/mavros/setpoint_position/global',
#             self.global_callback,
#             qos_profile
#         )
        
#         # Subscription for Local Position
#         self.local_position_subscription = self.create_subscription(
#             PoseStamped,
#             '/mavros/local_position/pose',
#             self.local_position_callback,
#             qos_profile
#         )

#         # Subscription for Global Position
#         self.global_position_subscription = self.create_subscription(
#             NavSatFix,
#             '/mavros/global_position/global',
#             self.global_position_callback,
#             qos_profile
#         )

#         # Subscription for State (Mode)
#         self.state_subscription = self.create_subscription(
#             State,
#             '/mavros/state',
#             self.state_callback,
#             qos_profile
#         )

#         # Subscription for Behavior and Search Status
#         self.status_subscription = self.create_subscription(
#             String,
#             '/njord_tasks/maneuvering/status',
#             self.status_callback,
#             qos_profile
#         )

#         # Add labels to GUI
#         self.local_coords_label = self.gui.add_label("Local Setpoint: X: 0.0, Y: 0.0")
#         self.global_coords_label = self.gui.add_label("Global Setpoint: Lat: 0.0, Lon: 0.0")
#         self.local_position_label = self.gui.add_label("Local Position: X: 0.0, Y: 0.0")
#         self.global_position_label = self.gui.add_label("Global Position: Lat: 0.0, Lon: 0.0")
#         self.mode_label = self.gui.add_label("Mode: Unknown")
#         self.behaviour_label = self.gui.add_label("Behaviour Status: Unknown")
#         self.search_label = self.gui.add_label("Search Status: Unknown")

#         self.previous_behaviour_status = None
#         self.previous_search_status = None

#     def check_node_statuses(self):
#         active_nodes = self.get_node_names()
#         for node_name in self.target_nodes:
#             is_running = node_name in active_nodes
#             self.gui.update_status(node_name, is_running)

#     def check_message_statuses(self):
#         current_time = self.get_clock().now()

#         # Check elapsed time for each topic and update label color if needed
#         def check_and_update_label(label, last_time):
#             time_since_last_update = (current_time - last_time).nanoseconds
#             if time_since_last_update > 3000000000:  # 1 second in nanoseconds
#                 self.gui.update_label(label, label.cget("text"), flash=False)
#                 label.config(fg="red")
#             else:
#                 label.config(fg="black")

#         check_and_update_label(self.local_position_label, self.last_local_position_time)
#         check_and_update_label(self.global_position_label, self.last_global_position_time)
#         check_and_update_label(self.mode_label, self.last_mode_time)
#         check_and_update_label(self.behaviour_label, self.last_status_time)
#         check_and_update_label(self.search_label, self.last_status_time)

#     def local_callback(self, msg):
#         x = msg.pose.position.x
#         y = msg.pose.position.y
#         self.get_logger().debug(f"Received local setpoint: X: {x}, Y: {y}")
#         self.gui.update_label(self.local_coords_label, f"Local Setpoint: X: {x:.2f}, Y: {y:.2f}", flash=True)

#     def global_callback(self, msg):
#         lat = msg.pose.position.latitude
#         lon = msg.pose.position.longitude
#         self.get_logger().debug(f"Received global setpoint: Lat: {lat}, Lon: {lon}")
#         self.gui.update_label(self.global_coords_label, f"Global Setpoint: Lat: {lat:.6f}, Lon: {lon:.6f}", flash=True)

#     def local_position_callback(self, msg):
#         x = msg.pose.position.x
#         y = msg.pose.position.y
#         self.get_logger().debug(f"Received local position: X: {x}, Y: {y}")
#         self.gui.update_label(self.local_position_label, f"Local Position: X: {x:.2f}, Y: {y:.2f}", flash=False)
#         self.last_local_position_time = self.get_clock().now()

#     def global_position_callback(self, msg):
#         lat = msg.latitude
#         lon = msg.longitude
#         self.get_logger().debug(f"Received global position: Lat: {lat}, Lon: {lon}")
#         self.gui.update_label(self.global_position_label, f"Global Position: Lat: {lat:.6f}, Lon: {lon:.6f}", flash=False)
#         self.last_global_position_time = self.get_clock().now()

#     def state_callback(self, msg):
#         mode = msg.mode  # Get the mode
#         self.get_logger().debug(f"Received state: Mode: {mode}")
#         self.gui.update_label(self.mode_label, f"Mode: {mode}", flash=False)
#         self.last_mode_time = self.get_clock().now()

#     def status_callback(self, msg):
#         # Parse the status message
#         data = msg.data.split('\n')
#         for line in data:
#             if 'BEHAVIOUR STATUS' in line:
#                 behaviour_status = line.split(': ')[1]
#                 if behaviour_status != self.previous_behaviour_status:
#                     self.get_logger().debug(f"Behaviour Status changed: {behaviour_status}")
#                     self.gui.update_label(self.behaviour_label, f"Behaviour Status: {behaviour_status}", flash=True)
#                     self.previous_behaviour_status = behaviour_status  # Update previous status
#             elif 'SEARCH STATUS' in line:
#                 search_status = line.split(': ')[1]
#                 if search_status != self.previous_search_status:
#                     self.get_logger().debug(f"Search Status changed: {search_status}")
#                     self.gui.update_label(self.search_label, f"Search Status: {search_status}", flash=True)
#                     self.previous_search_status = search_status  # Update previous status
#         self.last_status_time = self.get_clock().now()
                    
# def main(args=None):
#     rclpy.init(args=args)
    
#     gui = GUI()  # Create the GUI

#     # Initialize the ROS node
#     node = SetpointNode(gui)
    
#     # Run the GUI and ROS spinning together
#     def ros_spin():
#         rclpy.spin_once(node, timeout_sec=0.1)
#         gui.after(100, ros_spin)  # Call this function again after 100ms

#     gui.after(100, ros_spin)  # Start the ROS spinning loop
#     gui.mainloop()  # Start the GUI event loop

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# write me a python gui that is also a ros2 node. 

# It should have a single button that says "Man Recovery". 
# The ros node should be subscribing to '/mavros/global_position/global' and always have the updated position. 
# when the button is pressed it should do 2 things. 
# 1. it should modify a file in the ros2 package "comp_tasks". Where the file is in "comp_tasks/config/gps_points.yaml"
# the yaml file looks like this:
# maneuvering:
#   ros__parameters:
#     recovery_lat: 63.4411718
#     recovery_lon: 10.4237096

# When the button is pressed, it should overwrite the values in the yaml file with whereever the robot currently is (/mavros/global_position/global topic has the data)

# 2. it should call the parameter server and change the values of the current running node. The node is named maneuvering and its in the comp_tasks package. 


# write me a bash script that will:

# Accept a ros2 node name
# Accept a string identifier

# echo /mavros/global_position/global and extract the most recent message's latitude and logitude. 

# it should then find a file in the ros2 comp_tasks package. Where the file is "comp_tasks/config/gps_points.yaml"
# the yaml file looks like this:
# <node_name>:
#   ros__parameters:
#     recovery_lat: 63.4411718
#     recovery_lon: 10.4237096
#     finish_lat: 63.4411718
#     finish_lon: 10.4237096

# It will find the section for the aproprotate node name and then 
# if the string identifier is "recovery", it will update the recovery lat and lon values to whatever the echod values were, and likewise for "finish". 
