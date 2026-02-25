#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand
import serial
import time

class STM32Controller(Node):
    def __init__(self):
        super().__init__('stm32_controller')
        
        # Parámetros
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Estado actual
        self.dir_dc = 0
        self.speed_dc = 0
        self.dir_servo = 0
        
        # Watchdog
        self.last_command_time = time.time()
        self.watchdog_timeout = 0.2
        
        # Conectar serial
        self.get_logger().info(f'Conectando a {port}...')
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout
            )
            self.get_logger().info('Conexión serial exitosa')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            raise e
        
        # Suscriptor al tópico de comandos (AHORA USA MENSAJE PERSONALIZADO)
        self.subscription = self.create_subscription(
            MotorCommand,  # <-- Cambió de String a MotorCommand
            '/motor_command',
            self.command_callback,
            10
        )
        
        # Timer para watchdog y envío continuo
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info('Nodo STM32Controller iniciado')
        self.get_logger().info('Esperando comandos en /motor_command')
    
    def command_callback(self, msg):
        """Recibe comandos directamente (no teclas)"""
        self.last_command_time = time.time()
        
        self.dir_dc = msg.dir_dc
        self.speed_dc = msg.speed_dc
        self.dir_servo = msg.dir_servo
        
        self.get_logger().debug(f'Comando: DC:{self.dir_dc},{self.speed_dc} Servo:{self.dir_servo}')
    
    def send_command(self):
        packet = bytearray([0xAA, 0x55, 0x01, self.dir_dc, self.speed_dc, self.dir_servo, 0xFF])
        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f'Error enviando: {e}')
    
    def timer_callback(self):
        if time.time() - self.last_command_time > self.watchdog_timeout:
            if self.dir_dc != 0 or self.speed_dc != 0 or self.dir_servo != 0:
                self.get_logger().warn('Watchdog timeout - deteniendo')
                self.dir_dc = 0
                self.speed_dc = 0
                self.dir_servo = 0
        self.send_command()
    
    def destroy_node(self):
        self.get_logger().info('Cerrando nodo...')
        self.dir_dc = 0
        self.speed_dc = 0
        self.dir_servo = 0
        self.send_command()
        time.sleep(0.1)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
