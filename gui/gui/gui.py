import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk

class GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ROS2 Local Setpoint")
        self.geometry("300x100")

        self.label = tk.Label(self, text="Local Setpoint:", font=("Arial", 14))
        self.label.pack(pady=10)

        self.coords_label = tk.Label(self, text="X: 0.0, Y: 0.0", font=("Arial", 12))
        self.coords_label.pack()

    def update_setpoint(self, x, y):
        self.coords_label.config(text=f"X: {x:.2f}, Y: {y:.2f}")
        self.coords_label.config(fg="green")
        self.after(500, lambda: self.coords_label.config(fg="black"))  # Flash green for 500ms

class SetpointNode(Node):
    def __init__(self, gui):
        super().__init__('setpoint_listener')
        self.gui = gui
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().debug(f"Received setpoint: X: {x}, Y: {y}")
        # Update the GUI when a message is received
        self.gui.update_setpoint(x, y)

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
