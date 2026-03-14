import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_msgs.msg import MotorCommand

class NodoMando(Node):
    def __init__(self):
        super().__init__('nodo_mando')
        
        self.publisher_ = self.create_publisher(MotorCommand, '/motor_command', 10)
        
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.get_logger().info('>>> NODO MANDO ACTIVO: Publicando en /motor_command')

    def joy_callback(self, msg):
        comando = MotorCommand()

        # 1. TRACCIÓN TRASERA: STICK IZQUIERDO
        eje_potencia = msg.axes[1]
        
        pwm_speed = int(abs(eje_potencia) * 255)
        comando.speed_dc = max(0, min(255, pwm_speed)) 
        
        if eje_potencia > 0.1: # Adelante
            comando.dir_dc = 1
        elif eje_potencia < -0.1: # Atrás
            comando.dir_dc = 2
        else: # Detenido
            comando.dir_dc = 0
            comando.speed_dc = 0

        # 2. DIRECCIÓN ACKERMANN: STICK DERECHO
        eje_direccion = msg.axes[2]

        servo_pwm = 1500 + int(eje_direccion * 300) 
        comando.dir_servo = max(1110, min(1740, servo_pwm))

        # 3. FRENO DE EMERGENCIA (Botón A / X)
        if msg.buttons[0] == 1:
            comando.dir_dc = 0
            comando.speed_dc = 0
            comando.dir_servo = 1500
            self.get_logger().warn('!!! PARO DE EMERGENCIA ACTIVADO !!!')

        self.publisher_.publish(comando)

def main(args=None):
    rclpy.init(args=args)
    node = NodoMando()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
