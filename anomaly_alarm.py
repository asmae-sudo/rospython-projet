import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame  # Pour jouer des sons
import os

class AnomalyAlarm(Node):
    def __init__(self):
        super().__init__('anomaly_alarm')
        self.subscription = self.create_subscription(
            String,
            'security_alert',  # Topic où les alertes de sécurité sont publiées
            self.listener_callback,
            10
        )
        self.get_logger().info("AnomalyAlarm Node Initialized.")
        
        # Initialisation de pygame pour jouer des sons
        pygame.mixer.init()

        # METTEZ ICI LE CHEMIN DU FICHIER SONORE
        self.alarm_sound = "/home/asmae/sounds/alarm.wav"  # Chemin vers votre fichier sonore
        # REMARQUE : remplacez "/home/asmae/sounds/alarm.wav" par le chemin de votre fichier sonore réel

    def listener_callback(self, msg):
        self.get_logger().info(f"Received security alert: {msg.data}")
        
        # Vérifier si une alarme doit être déclenchée
        if "anomaly detected" in msg.data:  # Vous pouvez personnaliser la condition
            self.play_alarm()

    def play_alarm(self):
        # Joue l'alarme sonore
        if os.path.exists(self.alarm_sound):
            self.get_logger().info("Playing alarm sound!")
            pygame.mixer.music.load(self.alarm_sound)
            pygame.mixer.music.play()

            # Attendez que l'alarme se termine avant de poursuivre
            while pygame.mixer.music.get_busy():  # Tant que la musique joue
                continue
        else:
            self.get_logger().warn(f"Sound file {self.alarm_sound} not found!")

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyAlarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

