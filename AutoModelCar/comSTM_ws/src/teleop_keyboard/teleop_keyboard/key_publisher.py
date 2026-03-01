#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
from pynput.keyboard import Key, Listener
import sys
import termios
import tty

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        
        self.dir_dc = 0
        self.speed_dc = 0
        self.dir_servo = 1500 
        
        # Sensibilidad de Joystick
        self.paso_servo = 15      
        self.retorno_servo = 25   
        self.paso_vel = 5         
        self.friccion_vel = 10    
        
        self.keys_pressed = set() # Aquí guardaremos todas las teclas presionadas simultáneamente
        
        # --- APAGAR EL ECO DE LA TERMINAL ---
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        # --- INICIAR EL LECTOR DE HARDWARE ---
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        self.get_logger().info('🕹️ Simulador de Joystick Multitecla Iniciado.')
        self.get_logger().info('W/A/S/D o Flechas para conducir simultáneamente.')
        self.get_logger().info('Espacio = Freno | ESC = Salir')

    def on_press(self, key):
        """Se activa al bajar el dedo"""
        self.keys_pressed.add(key)
        if key == Key.esc:
            self.destroy_node()
            sys.exit(0)

    def on_release(self, key):
        """Se activa al levantar el dedo"""
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)

    def timer_callback(self):
        # 1. Leer todas las teclas que están presionadas en este instante
        up = False; down = False; left = False; right = False; space = False

        for k in self.keys_pressed:
            if k == Key.up or (hasattr(k, 'char') and k.char and k.char.lower() == 'w'):
                up = True
            if k == Key.down or (hasattr(k, 'char') and k.char and k.char.lower() == 's'):
                down = True
            if k == Key.left or (hasattr(k, 'char') and k.char and k.char.lower() == 'a'):
                left = True
            if k == Key.right or (hasattr(k, 'char') and k.char and k.char.lower() == 'd'):
                right = True
            if k == Key.space:
                space = True

        # 2. Lógica de Aceleración (DC)
        if up and not down:
            self.dir_dc = 1
            self.speed_dc = min(100, self.speed_dc + self.paso_vel)
        elif down and not up:
            self.dir_dc = 2
            self.speed_dc = min(100, self.speed_dc + self.paso_vel)
        else:
            # Fricción automática si sueltas
            if self.speed_dc > 0:
                self.speed_dc = max(0, self.speed_dc - self.friccion_vel)
            if self.speed_dc == 0:
                self.dir_dc = 0

        # 3. Lógica de Dirección (Servo)
        if left and not right:
            self.dir_servo = min(1740, self.dir_servo + self.paso_servo)
        elif right and not left:
            self.dir_servo = max(1110, self.dir_servo - self.paso_servo)
        else:
            # Retorno automático al centro
            if self.dir_servo > 1500:
                self.dir_servo = max(1500, self.dir_servo - self.retorno_servo)
            elif self.dir_servo < 1500:
                self.dir_servo = min(1500, self.dir_servo + self.retorno_servo)

        # 4. Freno de Emergencia
        if space:
            self.dir_dc = 0
            self.speed_dc = 0
            self.dir_servo = 1500

        # Publicar los valores
        msg = MotorCommand()
        msg.dir_dc = self.dir_dc
        msg.speed_dc = self.speed_dc
        msg.dir_servo = self.dir_servo
        self.publisher.publish(msg)

    def destroy_node(self):
        msg = MotorCommand()
        msg.dir_dc = 0
        msg.speed_dc = 0
        msg.dir_servo = 1500
        self.publisher.publish(msg)
        
        # Restaurar la terminal a la normalidad
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.listener.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()