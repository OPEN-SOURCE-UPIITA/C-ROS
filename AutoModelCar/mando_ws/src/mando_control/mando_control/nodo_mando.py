import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_msgs.msg import MotorCommand

class NodoMando(Node):
    def __init__(self):
        super().__init__('nodo_mando')
        self.publisher_ = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.hazards_active = False
        self.last_button_y = 0
        self.get_logger().info('>>> NODO MANDO ACTIVO: Motores (MAX 255) y Luces')

    def joy_callback(self, msg):
        comando = MotorCommand()
        
        # ==========================================
        # 1. TRACCIÓN: REGRESAMOS A 255 DE POTENCIA
        # ==========================================
        eje_potencia = msg.axes[1]
        
        # Multiplicamos por 255 para vencer la inercia del carro
        pwm_speed = int(abs(eje_potencia) * 255)
        comando.speed_dc = max(0, min(255, pwm_speed)) 
        
        if eje_potencia > 0.1: 
            comando.dir_dc = 1
            comando.stop_lights = 0
        elif eje_potencia < -0.1: 
            comando.dir_dc = 2
            comando.stop_lights = 0
        else: 
            comando.dir_dc = 0
            comando.speed_dc = 0
            comando.stop_lights = 1 # Freno automático

        # ==========================================
        # 2. DIRECCIÓN ACKERMANN
        # ==========================================
        eje_direccion = msg.axes[2]
        servo_pwm = 1500 + int(eje_direccion * 300) 
        comando.dir_servo = max(1110, min(1740, servo_pwm))

        # ==========================================
        # 3. LUCES DIRECCIONALES (D-Pad)
        # ==========================================
        dpad_x = msg.axes[6] if len(msg.axes) > 6 else 0.0
        
        if msg.buttons[3] == 1 and self.last_button_y == 0:
            self.hazards_active = not self.hazards_active
        self.last_button_y = msg.buttons[3]

        if self.hazards_active:
            comando.turn_signals = 3
        elif dpad_x > 0.5:   
            comando.turn_signals = 2 
        elif dpad_x < -0.5:  
            comando.turn_signals = 1 
        else:
            comando.turn_signals = 0

        # ==========================================
        # 4. FRENO DE EMERGENCIA (Botón A / X)
        # ==========================================
        if msg.buttons[0] == 1:
            comando.dir_dc = 0
            comando.speed_dc = 0
            comando.dir_servo = 1500
            comando.stop_lights = 1 
            comando.turn_signals = 3 
            self.get_logger().warn('!!! PARO DE EMERGENCIA !!!')

        self.publisher_.publish(comando)

def main(args=None):
    rclpy.init(args=args)
    node = NodoMando()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()