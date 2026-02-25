#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from motor_msgs.msg import MotorCommand
import sys
import termios
import tty
import select
import threading

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.last_key = ''
        self.running = True
        
        # Configurar terminal para lectura de teclas sin esperar Enter
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.read_keys)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.get_logger().info('Nodo KeyPublisher iniciado')
        self.get_logger().info('Usa las flechas para controlar el robot')
    
    def read_keys(self):
        """Hilo que lee las teclas presionadas"""
        while self.running:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
                # Mapeo de teclas especiales (flechas)
                if key == '\x1b':  # Secuencia de escape para flechas
                    key += sys.stdin.read(2)  # Leer los siguientes dos caracteres
                self.last_key = key
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def timer_callback(self):
        """Publica la última tecla presionada"""
        msg = String()
        msg.data = self.last_key
        self.publisher.publish(msg)
        self.last_key = ''  # Limpiar para no repetir
    
    def destroy_node(self):
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
