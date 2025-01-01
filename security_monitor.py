import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SecurityMonitor(Node):
    def __init__(self):
        super().__init__('security_monitor')
        
        # Abonnement au topic /password_activity pour surveiller les activités liées aux mots de passe
        self.password_subscription = self.create_subscription(
            String,
            '/password_activity',  # Topic des activités de mots de passe
            self.password_activity_callback,
            10
        )
        
        # Création du publisher pour publier les alertes de sécurité
 self.security_alert_publisher = self.create_publisher(String, '/security_alert', 10)

    # Callback pour surveiller les activités de mots de passe
    def password_activity_callback(self, msg):
        self.get_logger().info(f"Received password activity: {msg.data}")
        
        # Détection d'anomalies : Si l'activité est suspecte, émettre une alerte
        if self.detect_anomaly(msg.data):
            self.trigger_security_alert(f"Anomaly detected: {msg.data}")

    # Méthode pour détecter une anomalie dans l'activité
    def detect_anomaly(self, activity):
        # Exemple simple d'anomalie : Vérification des mots-clés d'anomalie
        suspicious_keywords = ['unauthorized', 'error', 'failed']
        
        for keyword in suspicious_keywords:
            if keyword in activity:
                return True
        return False

    # Méthode pour déclencher une alerte de sécurité
    def trigger_security_alert(self, alert_message):
        alert_msg = String()
        alert_msg.data = alert_message
        self.security_alert_publisher.publish(alert_msg)
        self.get_logger().warn(f"Security alert triggered: {alert_message}")


def main(args=None):
    rclpy.init(args=args)

    security_monitor_node = SecurityMonitor()
    rclpy.spin(security_monitor_node)

    security_monitor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

