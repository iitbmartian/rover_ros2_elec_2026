import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputPublisher(Node):
    def __init__(self):
        super().__init__('input_publisher')
        self.publisher = self.create_publisher(String, 'user_input', 10)
        self.get_logger().info('Input Publisher started. Type messages and press Enter to publish.')
        
    def publish_input(self, user_input):
        msg = String()
        msg.data = user_input
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = InputPublisher()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            try:
                user_input = input("Enter message to publish (Ctrl+C to exit): ")
                if user_input:  # Only publish if not empty
                    node.publish_input(user_input)
            except EOFError:
                # Handle EOF when input stream closes
                break
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected')
    finally:
        node.get_logger().info('Shutting down cleanly...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
