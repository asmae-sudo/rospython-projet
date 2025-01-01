import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class SpeechExecutor(Node):
    def __init__(self):
        super().__init__('speech_executor')
        self.subscription = self.create_subscription(
            String,
            'speech_command_topic',  # Topic pour recevoir les commandes vocales
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'password_request', 10)  # Pour envoyer des commandes aux autres nœuds

        self.get_logger().info("SpeechExecutor Node Initialized.")

    def listener_callback(self, msg):
        # Lorsque la commande est reçue sur le topic
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")

        # Traitement des commandes vocales
        if 'générer un mot de passe' in command:
            self.publish_command('generate_password')
        elif 'supprimer un mot de passe' in command:
            self.publish_command('delete_password')
        elif 'afficher un mot de passe' in command:
            self.publish_command('show_password')
        else:
            self.get_logger().info("Unrecognized command.")
def publish_command(self, command):
        # Envoie la commande au topic approprié
        msg = String()
        msg.data = command
        self.get_logger().info(f"Publishing command: {command}")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

