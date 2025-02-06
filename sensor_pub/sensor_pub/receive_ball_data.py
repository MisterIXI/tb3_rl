import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import socket
import threading
 
class BallAnglePublisher(Node):
    def __init__(self):
        super().__init__('socket_publisher')
        self.publisher_ = self.create_publisher(Float32, 'ball_angle', 1)
        # Start socket thread
        self.socket_thread = threading.Thread(target=self.socket_loop, daemon=True)
        self.socket_thread.start()
 
    def socket_loop(self):
        # Setup a simple TCP server
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 12345))
        server_socket.listen(1)
        self.get_logger().info("Waiting for connection on port 12345...")
        conn, addr = server_socket.accept()
        self.get_logger().info(f"Connected by {addr}")
        try:
            while rclpy.ok():
                data = conn.recv(1024)
                if not data:
                    break  # Connection closed
                # Create and publish a ROS message
                try:
                    msg = Float32()
                    msg.data = float(data.decode('utf-8').strip())
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")
                except ValueError as ve:
                    self.get_logger().warning(f"Received invalid float data: {data}. Error: {ve}")
                    
        except Exception as e:
            self.get_logger().error(f"Socket error: {e}")
        finally:
            conn.close()
            server_socket.close()
 
def main(args=None):
    rclpy.init(args=args)
    node = BallAnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()