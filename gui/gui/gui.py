import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State  # Import State message for mode
from sensor_msgs.msg import NavSatFix
import tkinter as tk

class GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Debug GUI")
        self.geometry("400x400")  # Increased size to accommodate more labels
        self.grid_labels = []

    def add_label(self, text):
        label = tk.Label(self, text=text, font=("Arial", 14))
        label.grid(row=len(self.grid_labels), column=0, pady=5)
        self.grid_labels.append(label)
        return label

    def update_label(self, label, text, flash=True):
        label.config(text=text)
        if flash:
            label.config(fg="green")
            self.after(500, lambda: label.config(fg="black"))  # Flash green for 500ms
        else:
            label.config(fg="black")  # Keep text black

class SetpointNode(Node):
    def __init__(self, gui):
        super().__init__('setpoint_listener')
        self.gui = gui
        
        # Define a QoS profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        # Subscription for Local Setpoint
        self.local_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.local_callback,
            qos_profile
        )
        
        # Subscription for Global Setpoint
        self.global_subscription = self.create_subscription(
            NavSatFix,
            '/mavros/setpoint_position/global',
            self.global_callback,
            qos_profile
        )
        
        # Subscription for Local Position
        self.local_position_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_position_callback,
            qos_profile
        )

        # Subscription for Global Position
        self.global_position_subscription = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_position_callback,
            qos_profile
        )

        # Subscription for State (Mode)
        self.state_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )

        # Add labels to GUI
        self.local_coords_label = self.gui.add_label("Local Setpoint: X: 0.0, Y: 0.0")
        self.global_coords_label = self.gui.add_label("Global Setpoint: Lat: 0.0, Lon: 0.0, Alt: 0.0")
        self.local_position_label = self.gui.add_label("Local Position: X: 0.0, Y: 0.0")
        self.global_position_label = self.gui.add_label("Global Position: Lat: 0.0, Lon: 0.0")
        self.mode_label = self.gui.add_label("Mode: Unknown")

    def local_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().debug(f"Received local setpoint: X: {x}, Y: {y}")
        self.gui.update_label(self.local_coords_label, f"Local Setpoint: X: {x:.2f}, Y: {y:.2f}", flash=True)

    def global_callback(self, msg):
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        alt = msg.pose.position.altitude
        self.get_logger().debug(f"Received global setpoint: Lat: {lat}, Lon: {lon}, Alt: {alt}")
        self.gui.update_label(self.global_coords_label, f"Global Setpoint: Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.2f}", flash=True)

    def local_position_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().debug(f"Received local position: X: {x}, Y: {y}")
        self.gui.update_label(self.local_position_label, f"Local Position: X: {x:.2f}, Y: {y:.2f}", flash=False)

    def global_position_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        self.get_logger().debug(f"Received global position: Lat: {lat}, Lon: {lon}")
        self.gui.update_label(self.global_position_label, f"Global Position: Lat: {lat:.6f}, Lon: {lon:.6f}", flash=False)

    def state_callback(self, msg):
        mode = msg.mode  # Get the mode
        self.get_logger().debug(f"Received state: Mode: {mode}")
        self.gui.update_label(self.mode_label, f"Mode: {mode}", flash=False)

def main(args=None):
    rclpy.init(args=args)
    
    gui = GUI()  # Create the GUI

    # Initialize the ROS node
    node = SetpointNode(gui)
    
    # Run the GUI and ROS spinning together
    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        gui.after(100, ros_spin)  # Call this function again after 100ms

    gui.after(100, ros_spin)  # Start the ROS spinning loop
    gui.mainloop()  # Start the GUI event loop

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
