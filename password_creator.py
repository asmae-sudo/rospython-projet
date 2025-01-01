import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import string

class PasswordCreator(Node):
    def __init__(self):
        super().__init__('password_creator')
        self.publisher_ = self.create_publisher(String, 'password_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'password_request',  # Nouveau topic pour la demande
            self.request_callback,
            10
        )
        self.get_logger().info("Password Creator node started.")
        
    def request_callback(self, msg):
        # Quand une demande est reçue, génère un mot de passe
        self.get_logger().info(f"Password requested: {msg.data}")
        password = self.generate_password()
        self.get_logger().info(f"Generated password: {password}")
        msg_to_publish = String()
        msg_to_publish.data = password
        self.publisher_.publish(msg_to_publish)

    def generate_password(self, length=12):
        characters = string.ascii_letters + string.digits + string.punctuation
        return ''.join(random.choice(characters) for i in range(length))

def main(args=None):
    rclpy.init(args=args)
    node = PasswordCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

