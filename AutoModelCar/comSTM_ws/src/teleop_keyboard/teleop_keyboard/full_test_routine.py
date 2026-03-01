#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
import time

class FullTestRoutine(Node):
    def __init__(self):
        super().__init__('full_test_routine')
        self.publisher = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('🚗 Rutina de Prueba Completa Iniciada')
        self.get_logger().info('Secuencia: Recta -> Curvas Adelante -> Freno -> Curvas Reversa')
        
        # Variable para evitar imprimir el mismo log 10 veces por segundo
        self.current_phase = -1 

    def log_phase(self, phase, message):
        """Imprime el mensaje solo cuando cambia de fase"""
        if self.current_phase != phase:
            self.get_logger().info(message)
            self.current_phase = phase

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9  # tiempo en segundos
        
        msg = MotorCommand()
        vel_prueba = 60  # Velocidad moderada para pruebas seguras
        
        # ==========================================
        # MÁQUINA DE ESTADOS BASADA EN EL TIEMPO
        # ==========================================
        if elapsed < 3.0:
            self.log_phase(0, 'Fase 1: Adelante Recto')
            msg.dir_dc = 1; msg.speed_dc = vel_prueba; msg.dir_servo = 1500
            
        elif elapsed < 6.0:
            self.log_phase(1, 'Fase 2: Adelante + Giro Izquierda')
            msg.dir_dc = 1; msg.speed_dc = vel_prueba; msg.dir_servo = 1740
            
        elif elapsed < 9.0:
            self.log_phase(2, 'Fase 3: Adelante + Giro Derecha')
            msg.dir_dc = 1; msg.speed_dc = vel_prueba; msg.dir_servo = 1110
            
        elif elapsed < 11.0:
            self.log_phase(3, 'Fase 4: Freno Activo de Seguridad')
            msg.dir_dc = 0; msg.speed_dc = 0; msg.dir_servo = 1500
            
        elif elapsed < 14.0:
            self.log_phase(4, 'Fase 5: Reversa Recta')
            msg.dir_dc = 2; msg.speed_dc = vel_prueba; msg.dir_servo = 1500
            
        elif elapsed < 17.0:
            self.log_phase(5, 'Fase 6: Reversa + Giro Izquierda')
            msg.dir_dc = 2; msg.speed_dc = vel_prueba; msg.dir_servo = 1740
            
        elif elapsed < 20.0:
            self.log_phase(6, 'Fase 7: Reversa + Giro Derecha')
            msg.dir_dc = 2; msg.speed_dc = vel_prueba; msg.dir_servo = 1110
            
        elif elapsed < 22.0:
            self.log_phase(7, 'Fase 8: Deteniendo y esperando reinicio...')
            msg.dir_dc = 0; msg.speed_dc = 0; msg.dir_servo = 1500
            
        else:
            # Reiniciar el ciclo
            self.start_time = self.get_clock().now()
            self.current_phase = -1
            return
        
        self.publisher.publish(msg)
    
    def destroy_node(self):
        self.get_logger().info('Deteniendo la rutina de prueba...')
        msg = MotorCommand()
        msg.dir_dc = 0; msg.speed_dc = 0; msg.dir_servo = 1500
        self.publisher.publish(msg)
        time.sleep(0.1)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FullTestRoutine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()