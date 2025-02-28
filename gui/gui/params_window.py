import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters
from rcl_interfaces.msg import ParameterEvent
import tkinter as tk
import signal
import os

def get_used_value(param_value):
    """Extract the actual value from the parameter object."""
    if param_value.string_value:
        return param_value.string_value
    elif param_value.integer_value != 0:
        return param_value.integer_value
    elif param_value.double_value != 0.0:
        return param_value.double_value
    elif param_value.bool_value:
        return param_value.bool_value
    elif param_value.string_array_value:
        return param_value.string_array_value
    elif param_value.integer_array_value:
        return param_value.integer_array_value
    elif param_value.double_array_value:
        return param_value.double_array_value
    elif param_value.bool_array_value:
        return param_value.bool_array_value
    elif param_value.byte_array_value:
        return param_value.byte_array_value
    return None


class GUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Parameters GUI")
        self.geometry("300x600")

        self.label = tk.Label(self, text="Waiting for parameters...", font=("Arial", 14))
        self.label.pack(pady=20)

        self.text_box = tk.Text(self, wrap=tk.WORD, height=25, width=70)
        self.text_box.pack(pady=10)

    def update_message(self, message):
        """Update the label and text box with the received message."""
        self.label.config(text="Latest Parameters:")
        self.text_box.delete(1.0, tk.END)
        self.text_box.insert(tk.END, message)
        self.text_box.see(tk.END)


class SetpointNode(Node):
    def __init__(self, gui):
        super().__init__('setpoint_node')
        self.gui = gui  # Reference to the GUI instance

        # Service clients
        self.client_list = self.create_client(ListParameters, '/maneuvering/list_parameters')
        self.client_get = self.create_client(GetParameters, '/maneuvering/get_parameters')

        # Subscription to parameter events
        self.subscription = self.create_subscription(
            ParameterEvent, '/parameter_events', self.parameter_event_callback, 10
        )

        self.get_parameters_from_node()

    def get_parameters_from_node(self):
        """Retrieve parameter names, then request their values."""
        while not self.client_list.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /maneuvering/list_parameters service...')

        request_list = ListParameters.Request()
        future_list = self.client_list.call_async(request_list)
        future_list.add_done_callback(self.get_values_from_node)

    def get_values_from_node(self, future):
        """Callback to request parameter values after retrieving names."""
        try:
            response = future.result()
            param_names = response.result.names

            if not param_names:
                self.get_logger().info("No parameters found.")
                self.gui.update_message("No parameters available.")
                return

            # Now call get_parameters with the retrieved names
            request_get = GetParameters.Request()
            request_get.names = param_names

            future_get = self.client_get.call_async(request_get)
            future_get.add_done_callback(lambda future: self.parameters_callback(future, param_names))

        except Exception as e:
            self.get_logger().error(f"Failed to list parameters: {e}")

    def parameters_callback(self, future, param_names):
        """Callback to update GUI with parameter values."""
        try:
            response = future.result()
            param_message = response.values

            message = ""

            for name, param in zip(param_names, param_message):
                message += f"{name}: {get_used_value(param)}\n"

            self.gui.update_message(message)

        except Exception as e:
            self.get_logger().error(f"Failed to get parameter values: {e}")

    def parameter_event_callback(self, msg):
        """Triggered whenever a parameter is added, changed, or deleted."""
        self.get_logger().info("Parameter update detected, refreshing parameters...")
        self.get_parameters_from_node()


def main(args=None):
    rclpy.init(args=args)

    gui = GUI()  # Create the GUI
    node = SetpointNode(gui)  # Initialize ROS node

    def ros_spin():
        rclpy.spin_once(node, timeout_sec=0.1)
        gui.after(10, ros_spin)  # Call this function again after 10ms

    def signal_handler(sig, frame):
        os.kill(os.getpid(), signal.SIGKILL)

    signal.signal(signal.SIGINT, signal_handler)

    gui.after(10, ros_spin)  # Start ROS spinning loop
    gui.mainloop()  # Start GUI event loop

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
