import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')
        self.publisher_ = self.create_publisher(String, 'speech_command_topic', 10)
        self.timer = self.create_timer(1.0, self.listen_for_command)  # Timer pour écouter les commandes

    def listen_for_command(self):
        recognizer = sr.Recognizer()

        with sr.Microphone() as source:
            self.get_logger().info("Listening for commands...")
            recognizer.adjust_for_ambient_noise(source)  # Réduit le bruit ambiant
            audio = recognizer.listen(source)  # Écoute le son
            
            try:
                # Utilisation de Google Web Speech API pour la reconnaissance vocale
                command = recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Recognized command: {command}")

                # Publier la commande sur le topic
                msg = String()
                msg.data = command
                self.publisher_.publish(msg)

            except sr.UnknownValueError:
                self.get_logger().info("Sorry, I did not understand that.")
            except sr.RequestError as e:
                self.get_logger().info(f"Could not request results from Google Speech service; {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

