import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped  # Import GeoPoseStamped
import tkinter as tk

class GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ROS2 Setpoints")
        self.geometry("300x150")

        # Local Setpoint
        self.local_label = tk.Label(self, text="Local Setpoint:", font=("Arial", 14))
        self.local_label.pack(pady=5)
        self.local_coords_label = tk.Label(self, text="X: 0.0, Y: 0.0", font=("Arial", 12))
        self.local_coords_label.pack()

        # Global Setpoint
        self.global_label = tk.Label(self, text="Global Setpoint:", font=("Arial", 14))
        self.global_label.pack(pady=5)
        self.global_coords_label = tk.Label(self, text="Lat: 0.0, Lon: 0.0, Alt: 0.0", font=("Arial", 12))
        self.global_coords_label.pack()

    def update_local_setpoint(self, x, y):
        self.local_coords_label.config(text=f"X: {x:.2f}, Y: {y:.2f}")
        self.local_coords_label.config(fg="green")
        self.after(500, lambda: self.local_coords_label.config(fg="black"))  # Flash green for 500ms

    def update_global_setpoint(self, lat, lon, alt):
        self.global_coords_label.config(text=f"Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.2f}")
        self.global_coords_label.config(fg="green")
        self.after(500, lambda: self.global_coords_label.config(fg="black"))  # Flash green for 500ms

class SetpointNode(Node):
    def __init__(self, gui):
        super().__init__('setpoint_listener')
        self.gui = gui
        
        # Subscription for Local Setpoint
        self.local_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.local_callback,
            10
        )

        # Subscription for Global Setpoint
        self.global_subscription = self.create_subscription(
            GeoPoseStamped,
            '/mavros/setpoint_position/global',
            self.global_callback,
            10
        )

    def local_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().debug(f"Received local setpoint: X: {x}, Y: {y}")
        # Update the GUI when a message is received
        self.gui.update_local_setpoint(x, y)

    def global_callback(self, msg):
        lat = msg.pose.position.latitude  # Get latitude from GeoPose
        lon = msg.pose.position.longitude  # Get longitude from GeoPose
        alt = msg.pose.position.altitude    # Get altitude from GeoPose
        self.get_logger().debug(f"Received global setpoint: Lat: {lat}, Lon: {lon}, Alt: {alt}")
        # Update the GUI when a message is received
        self.gui.update_global_setpoint(lat, lon, alt)

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
