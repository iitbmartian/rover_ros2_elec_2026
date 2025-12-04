import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from pos_control_interfaces.action import Trajectory
from action_msgs.msg import GoalStatus
from threading import Lock, Condition, Thread, Event
import time


class SafeQueue:
    """Thread-safe queue with condition variable for efficient waiting."""
    def __init__(self):
        self.queue = []
        self.condition = Condition(Lock())

    def append(self, item):
        with self.condition:
            self.queue.append(item)
            self.condition.notify_all()

    def pop_first(self):
        with self.condition:
            if self.queue:
                return self.queue.pop(0)
            return None

    def clear(self):
        with self.condition:
            self.queue.clear()

    def is_empty(self):
        with self.condition:
            return not self.queue

    def wait_for_item(self, timeout=None):
        """Wait until queue has items or timeout occurs."""
        with self.condition:
            while not self.queue:
                if not self.condition.wait(timeout=timeout):
                    return False
            return True


class ActionQueueClient(Node):
    """
    Thread-safe action client that processes a queue of commands from a topic.
    """
    def __init__(self):
        super().__init__('action_queue_client')
        self.queue = SafeQueue()
        self.action_client = ActionClient(self, Trajectory, 'pid_controller_actions')
        self.subscription = self.create_subscription(
            String, 'user_input', self.listener_callback, 10)
        
        # Shutdown flag for graceful termination
        self.shutdown_event = Event()
        self.pending_goal_handle = None
        self.pending_goal_lock = Lock()
        
        self.processing_thread = Thread(target=self.process_queue_loop, daemon=False)
        self.processing_thread.start()

    def listener_callback(self, msg):
        """Callback for the 'user_input' topic subscriber."""
        self.get_logger().info(f'Received command: "{msg.data}"')
        self.queue.append(msg)

    def send_goal(self, command):
        """
        Sends a goal to the action server and waits for the result.
        Returns: 'succeeded', 'rejected', 'aborted', or 'error'
        """
        try:
            goal_msg = Trajectory.Goal()
            # TODO: Parse command and populate goal_msg fields appropriately
            # Example: goal_msg.trajectory_data = self.parse_command(command)
            goal_msg.command = command
            
            self.get_logger().info(f'Sending goal for command: "{command.data}"')

            # Wait for server with 5-second timeout
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Action server not available after 5 seconds')
                return 'error'

            # Send goal asynchronously
            try:
                send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                # Wait with timeout instead of spin_until_future_complete
                goal_handle = self._wait_for_future(send_goal_future, timeout=5.0)
                
                if goal_handle is None:
                    self.get_logger().error('Goal send timed out')
                    return 'error'

            except Exception as e:
                self.get_logger().error(f'Failed to send goal: {str(e)}')
                return 'error'

            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal rejected for command: "{command.data}"')
                return 'rejected'

            self.get_logger().info('Goal accepted. Waiting for result...')

            # Store goal handle for potential cancellation
            with self.pending_goal_lock:
                self.pending_goal_handle = goal_handle

            try:
                get_result_future = goal_handle.get_result_async()
                result = self._wait_for_future(get_result_future, timeout=30.0)
                
                if result is None:
                    self.get_logger().error('Goal result timed out')
                    return 'error'

            except Exception as e:
                self.get_logger().error(f'Failed to get goal result: {str(e)}')
                return 'error'
            finally:
                with self.pending_goal_lock:
                    self.pending_goal_handle = None

            # Check goal status using proper constants
            if result.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error(f'Goal aborted for command: "{command.data}"')
                return 'aborted'
            elif result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Goal succeeded for command: "{command.data}"')
                return 'succeeded'
            else:
                self.get_logger().warn(f'Goal finished with status {result.status}')
                return 'succeeded'  # Treat other terminal states as success

        except Exception as e:
            self.get_logger().error(f'Unexpected error in send_goal: {str(e)}')
            return 'error'

    def _wait_for_future(self, future, timeout=5.0):
        """
        Wait for a future to complete without using spin_until_future_complete.
        Uses polling to avoid thread safety issues.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'Future returned exception: {str(e)}')
                    return None
            time.sleep(0.01)  # Small sleep to avoid busy-waiting
        
        self.get_logger().error('Future wait timed out')
        return None

    def process_queue_loop(self):
        """Continuously processes commands from the queue."""
        while not self.shutdown_event.is_set():
            try:
                # Wait for item with timeout to allow checking shutdown
                if not self.queue.wait_for_item(timeout=1.0):
                    continue

                command = self.queue.pop_first()
                if command is None:
                    continue

                self.get_logger().info(f'Processing command from queue: "{command.data}"')
                status = self.send_goal(command)

                if status == 'rejected':
                    self.get_logger().warn(f'Skipping rejected goal for: "{command.data}"')
                    continue

                elif status == 'aborted':
                    self.get_logger().warn('Clearing queue due to aborted goal')
                    self.queue.clear()
                    # Clear any pending messages before continuing
                    continue

                elif status == 'succeeded':
                    # Use event-based wait instead of blocking sleep
                    self.get_logger().info('Waiting 2 seconds before processing next command')
                    if self.shutdown_event.wait(timeout=2.0):
                        break  # Shutdown requested during sleep
                    continue

                elif status == 'error':
                    self.get_logger().error('Error sending goal, skipping this command')
                    continue

            except Exception as e:
                self.get_logger().error(f'Exception in process_queue_loop: {str(e)}')
                continue

        self.get_logger().info('Processing thread shutting down')

    def feedback_callback(self, feedback_msg):
        end_eff_pos = feedback_msg.feedback.end_effector_pos
        self.get_logger().info(f'End Effector at x = {end_eff_pos.x}, y = {end_eff_pos.y}')


    def destroy_node(self):
        """Override destroy_node to implement graceful shutdown."""
        self.get_logger().info('Initiating graceful shutdown')
        
        # Cancel any pending goal
        with self.pending_goal_lock:
            if self.pending_goal_handle is not None:
                try:
                    cancel_future = self.pending_goal_handle.cancel_goal_async()
                    self.get_logger().info('Sent goal cancellation request')
                except Exception as e:
                    self.get_logger().error(f'Failed to cancel goal: {str(e)}')
        
        # Signal processing thread to shut down
        self.shutdown_event.set()
        
        # Wait for processing thread to finish (with timeout)
        self.processing_thread.join(timeout=5.0)
        if self.processing_thread.is_alive():
            self.get_logger().warn('Processing thread did not terminate in time')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    action_queue_client = ActionQueueClient()

    try:
        rclpy.spin(action_queue_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_queue_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
