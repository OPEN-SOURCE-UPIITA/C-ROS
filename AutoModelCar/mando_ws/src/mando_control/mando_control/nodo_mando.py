import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_msgs.msg import MotorCommand

class NodoMando(Node):
    def __init__(self):
        super().__init__('nodo_mando')
        
        self.publisher_ = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Variables de estado para los botones tipo "Toggle"
        self.hazards_active = False
        self.last_button_y = 0
        
        self.get_logger().info('>>> NODO MANDO ACTIVO: Motores (0-100) y Luces')

    def joy_callback(self, msg):
        comando = MotorCommand()
        
        # ==========================================
        # 1. TRACCIÓN Y FRENO AUTOMÁTICO
        # ==========================================
        eje_potencia = msg.axes[1]
        
        # Nueva escala: 0 a 100
        pwm_speed = int(abs(eje_potencia) * 255)
        comando.speed_dc = max(0, min(255, pwm_speed)) 
        
        if eje_potencia > 0.1: # Acelerando adelante
            comando.dir_dc = 1
            comando.stop_lights = 0
        elif eje_potencia < -0.1: # Reversa
            comando.dir_dc = 2
            comando.stop_lights = 0
        else: # Detenido (Stick en el centro)
            comando.dir_dc = 0
            comando.speed_dc = 0
            comando.stop_lights = 1 # Freno automático encendido

        # ==========================================
        # 2. DIRECCIÓN ACKERMANN
        # ==========================================
        eje_direccion = msg.axes[3]
        servo_pwm = 1500 + int(eje_direccion * 300) 
        comando.dir_servo = max(1110, min(1740, servo_pwm))

        # ==========================================
        # 3. LUCES (Direccionales e Intermitentes)
        # ==========================================
        # La cruz (D-Pad) suele estar en los ejes 6 o 7. Ajusta si tu control es diferente.
        dpad_x = msg.axes[6] if len(msg.axes) > 6 else 0.0
        
        # Lógica del Botón Y (Botón 3) para prender/apagar intermitentes
        if msg.buttons[3] == 1 and self.last_button_y == 0:
            self.hazards_active = not self.hazards_active
        self.last_button_y = msg.buttons[3]

        # Prioridad a las intermitentes (Ambas luces)
        if self.hazards_active:
            comando.turn_signals = 3
        elif dpad_x > 0.5:   # Cruz Izquierda
            comando.turn_signals = 2 
        elif dpad_x < -0.5:  # Cruz Derecha
            comando.turn_signals = 1 
        else:
            comando.turn_signals = 0

        # ==========================================
        # 4. FRENO DE EMERGENCIA ABSOLUTO (Botón A / X)
        # ==========================================
        if msg.buttons[0] == 1:
            comando.dir_dc = 0
            comando.speed_dc = 0
            comando.dir_servo = 1500
            comando.stop_lights = 1  # Freno encendido
            comando.turn_signals = 3 # Intermitentes de emergencia
            self.get_logger().warn('!!! PARO DE EMERGENCIA ACTIVADO !!!')

        self.publisher_.publish(comando)

def main(args=None):
    rclpy.init(args=args)
    node = NodoMando()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()