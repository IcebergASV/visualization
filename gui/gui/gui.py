import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk

class GUI(Node):
    def __init__(self):
        super().__init__('gui')
        
        # Initialize the GUI
        self.root = tk.Tk()
        self.root.title("Local Setpoint GUI")

        # Create the label for the setpoint
        self.setpoint_label = tk.Label(self.root, text="Local Setpoint: ", font=("Helvetica", 16))
        self.setpoint_label.pack()

        self.coords_label = tk.Label(self.root, text="X: 0.0, Y: 0.0", font=("Helvetica", 16), fg="black")
        self.coords_label.pack()

        # Create a ROS 2 subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.setpoint_callback,
            10
        )

        # To manage the flashing effect
        self.flash_color = False
        self.last_coords = "X: 0.0, Y: 0.0"
        
        # Start the GUI loop
        self.update_gui()
        self.root.mainloop()

    def setpoint_callback(self, msg):
        # Extract X and Y coordinates from the PoseStamped message
        x = msg.pose.position.x
        y = msg.pose.position.y
        print("callback")
        coords_text = f"X: {x:.2f}, Y: {y:.2f}"
        
        # Update the coordinates and flash green
        self.coords_label.config(text=coords_text, fg="green")
        self.last_coords = coords_text
        self.flash_color = True

        self.update_gui()

    def update_gui(self):
        # Reset color to black after flashing
        if self.flash_color:
            self.coords_label.after(500, lambda: self.coords_label.config(fg="black"))
            self.flash_color = False

        # Keep updating the GUI
        #self.root.after(100, self.update_gui)


def main(args=None):
    rclpy.init(args=args)

    gui_node = GUI()

    rclpy.spin(gui_node)
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
