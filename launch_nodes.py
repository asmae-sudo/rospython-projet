import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pygame
import os
import threading

# Classe de nœud SpeechRecognizer
class SpeechRecognizer(Node):
    def __init__(self):
        super().__init__('speech_recognizer')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.get_logger().info("SpeechRecognizer Node Initialized.")
        self.listener_thread = threading.Thread(target=self.listen_for_speech)
        self.listener_thread.start()

    def listen_for_speech(self):
        recognizer = sr.Recognizer()
        mic = sr.Microphone()

        while rclpy.ok():
            with mic as source:
                recognizer.adjust_for_ambient_noise(source)
                self.get_logger().info("Listening for command...")

                try:
                    audio = recognizer.listen(source)
                    command = recognizer.recognize_google(audio)
                    self.get_logger().info(f"Command received: {command}")
                    self.publish_command(command)
                except sr.UnknownValueError:
                    self.get_logger().info("Could not understand audio.")
                except sr.RequestError as e:
                    self.get_logger().warn(f"Speech recognition error: {e}")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {command}")


# Classe de nœud AnomalyAlarm
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
        pygame.mixer.init()
        self.alarm_sound = "/home/asmae/sounds/alarm.wav"  # Changez avec le chemin de votre son

    def listener_callback(self, msg):
        self.get_logger().info(f"Received security alert: {msg.data}")
        if "anomaly detected" in msg.data:
            self.play_alarm()

    def play_alarm(self):
        if os.path.exists(self.alarm_sound):
            self.get_logger().info("Playing alarm sound!")
            pygame.mixer.music.load(self.alarm_sound)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                continue
        else:
            self.get_logger().warn(f"Sound file {self.alarm_sound} not found!")


# Classe de nœud PasswordManager (exemple)
class PasswordManager(Node):
    def __init__(self):
        super().__init__('password_manager')
        self.publisher_ = self.create_publisher(String, 'password_activity', 10)
        self.get_logger().info("PasswordManager Node Initialized.")

    def create_password(self):
        msg = String()
        msg.data = "Generated password"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published password: {msg.data}")


# Fonction pour initialiser et faire tourner les nœuds
def launch_nodes():
    rclpy.init()

    # Créer les nœuds
    speech_recognizer_node = SpeechRecognizer()
    anomaly_alarm_node = AnomalyAlarm()
    password_manager_node = PasswordManager()

    # Lancer les nœuds dans une boucle avec spin_once
    try:
        while rclpy.ok():
            rclpy.spin_once(speech_recognizer_node)  # Écoute des commandes vocales
            rclpy.spin_once(anomaly_alarm_node)  # Alarme de sécurité
            rclpy.spin_once(password_manager_node)  # Gestion des mots de passe

    except KeyboardInterrupt:
        pass
    finally:
        speech_recognizer_node.destroy_node()
        anomaly_alarm_node.destroy_node()
        password_manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    launch_nodes()

