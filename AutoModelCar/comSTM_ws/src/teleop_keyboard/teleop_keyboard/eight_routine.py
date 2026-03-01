#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
import time
import math

class EightRoutine(Node):
    def __init__(self):
        super().__init__('eight_routine')
        self.publisher = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.phase = 0  # 0=primera curva, 1=segunda curva
        
        self.get_logger().info('Rutina del 8 iniciada')
    
    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9  # segundos
        
        msg = MotorCommand()
        
        # Velocidad constante hacia adelante
        msg.dir_dc = 1  # Adelante
        msg.speed_dc = 80  # Velocidad media
        
        # Patrón en 8: alternar dirección del servo
        if elapsed < 3.0: 
            msg.dir_servo = 1740  # <--- Cambiado (Izquierda)
            self.get_logger().debug('Curva izquierda')
        elif elapsed < 6.0: 
            msg.dir_servo = 1110  # <--- Cambiado (Derecha)
            self.get_logger().debug('Curva derecha')
        else:
            # Reiniciar ciclo
            self.start_time = self.get_clock().now()
            return
        
        self.publisher.publish(msg)
    
    def destroy_node(self):
        # Detener motores al salir
        msg = MotorCommand()
        msg.dir_dc = 0
        msg.speed_dc = 0
        msg.dir_servo = 1500
        self.publisher.publish(msg)
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EightRoutine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
