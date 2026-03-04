import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motor_msgs.msg import MotorCommand, EncoderData

class SimAdapter(Node):
    def __init__(self):
        super().__init__('sim_adapter')
        self.declare_parameter('manual_mode', True)
        self.manual_mode = self.get_parameter('manual_mode').value
        self.pub = self.create_publisher(Twist, '/model/carro/cmd_vel', 10)

        if self.manual_mode:
            self.create_subscription(MotorCommand, '/motor_command', self.manual_cb, 10)
        else:
            self.create_subscription(EncoderData, '/encoder_data', self.encoder_cb, 10)

    def manual_cb(self, msg):
        t = Twist()
        speed = msg.speed_dc / 100.0
        t.linear.x = speed if msg.dir_dc == 1 else -speed if msg.dir_dc == 2 else 0.0
        t.angular.z = 0.6 if msg.dir_servo == 1 else -0.6 if msg.dir_servo == 2 else 0.0
        self.pub.publish(t)

    def encoder_cb(self, msg):
        t = Twist()
        # Promedio de velocidad de ambos motores (ticks/ciclo a m/s)
        avg_vel = (msg.vel_m1 + msg.vel_m2) / 2.0
        speed = avg_vel * 0.012  # Factor de escala sugerido
        t.linear.x = speed if msg.dir_m1 == 1 else -speed if msg.dir_m1 == 2 else 0.0
        self.pub.publish(t)

def main():
    rclpy.init()
    rclpy.spin(SimAdapter())
    rclpy.shutdown()