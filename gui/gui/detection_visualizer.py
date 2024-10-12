import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import DetectionArray  # Adjust import based on your package structure
import tkinter as tk
import time

class DetectionVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("YOLO Detection Visualizer")
        self.geometry("640x480")
        self.canvas = tk.Canvas(self, width=640, height=480, bg="white")
        self.canvas.pack()
        self.detections = []

        # Status indicators
        self.warning_message = self.canvas.create_text(10, 10, text="", fill="red", anchor="nw")  # Initialize warning message
        self.last_message_time = time.time()  # Track the last message time
        self.topic_name = "/yolo/detections"  # Topic name for display
        self.check_status()  # Start the status check

    def update_detections(self, detections):
        self.canvas.delete("all")  # Clear previous drawings
        self.detections = detections
        for detection in self.detections:
            self.draw_bounding_box(detection)

        self.last_message_time = time.time()  # Update last message time
        self.clear_warning_signal()  # Clear the warning signal when a message is received

    def draw_bounding_box(self, detection):
        # Get bounding box coordinates
        center_x = detection.bbox.center.position.x
        center_y = detection.bbox.center.position.y
        width = detection.bbox.size.x
        height = detection.bbox.size.y
        
        # Calculate top-left corner of the bounding box
        top_left_x = center_x - width / 2
        top_left_y = center_y - height / 2

        # Determine color and label
        class_name = detection.class_name
        color, label = self.get_color_and_label(class_name)

        # Draw rectangle and label
        self.canvas.create_rectangle(top_left_x, top_left_y, top_left_x + width, top_left_y + height, fill=color, outline="black")
        label_color = "black" if class_name == "yellow_buoy" else "white"
        self.canvas.create_text(center_x, center_y, text=label, fill=label_color)

    def get_color_and_label(self, class_name):
        color_map = {
            "yellow_buoy": "yellow",
            "red_buoy": "red",
            "green_buoy": "green",
            "blue_buoy": "blue",
            "black_buoy": "black",
            "red_marker": "red",
            "green_marker": "green"
        }
        # Default color if not found
        color = color_map.get(class_name, "white")
        # Set label
        label = "B" if "buoy" in class_name else "M" if "marker" in class_name else ""
        return color, label

    def clear_warning_signal(self):
        self.canvas.itemconfig(self.warning_message, text="")  # Clear the warning message

    def show_warning_signal(self):
        self.canvas.itemconfig(self.warning_message, text=f"No messages published to {self.topic_name}")  # Show warning message

    def check_status(self):
        current_time = time.time()
        if current_time - self.last_message_time > 1.0:
            self.show_warning_signal()  # Show warning if no message in 1 second
        else:
            self.clear_warning_signal()  # Clear warning if receiving messages
        
        self.after(100, self.check_status)  # Check status every 100ms

class DetectionNode(Node):
    def __init__(self, visualizer):
        super().__init__('detection_listener')
        self.visualizer = visualizer

        # Create a subscription to the /yolo/detections topic
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        self.get_logger().debug(f"Received {len(msg.detections)} detections.")
        self.visualizer.update_detections(msg.detections)

def main(args=None):
    rclpy.init(args=args)

    visualizer = DetectionVisualizer()  # Create the GUI
    node = DetectionNode(visualizer)  # Create the ROS node

    # Run the GUI and ROS spinning together
    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        visualizer.after(100, ros_spin)  # Call this function again after 100ms

    visualizer.after(100, ros_spin)  # Start the ROS spinning loop
    visualizer.mainloop()  # Start the GUI event loop

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
