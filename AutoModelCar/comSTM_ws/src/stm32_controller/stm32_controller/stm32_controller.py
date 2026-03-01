#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand, EncoderData
import serial
import time

class STM32Controller(Node):
    def __init__(self):
        super().__init__('stm32_controller')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.01) # Timeout súper bajo para no bloquear
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        self.dir_dc = 0
        self.speed_dc = 0
        self.dir_servo = 1500 
        
        # Variables para calcular la velocidad (Delta de Ticks)
        self.last_izq = 0
        self.last_der = 0
        
        self.last_command_time = time.time()
        self.watchdog_timeout = 0.2
        
        self.get_logger().info(f'Conectando a {port}...')
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            self.get_logger().info('Conexión serial bidireccional exitosa')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            raise e
        
        # Suscriptor (Recibe órdenes de la PC)
        self.subscription = self.create_subscription(MotorCommand, '/motor_command', self.command_callback, 10)
        
        # Publicador (Manda datos de los encoders a la PC)
        self.pub_encoder = self.create_publisher(EncoderData, '/encoder_data', 10)
        
        # Timers
        self.timer_write = self.create_timer(0.05, self.timer_write_callback) # 20 Hz (Escritura)
        self.timer_read = self.create_timer(0.01, self.timer_read_callback)   # 100 Hz (Lectura rápida)
        
        self.get_logger().info('Nodo Puente STM32 (Tx/Rx) Iniciado')
    
    def command_callback(self, msg):
        self.last_command_time = time.time()
        self.dir_dc = msg.dir_dc
        self.speed_dc = msg.speed_dc
        self.dir_servo = msg.dir_servo
    
    def send_command(self):
        servo_pwm = int(self.dir_servo)
        if servo_pwm == 0: servo_pwm = 1500 
        servo_pwm = max(1110, min(1740, servo_pwm))

        high_byte = (servo_pwm >> 8) & 0xFF
        low_byte = servo_pwm & 0xFF

        packet = bytearray([0xAA, 0x55, 0x01, self.dir_dc, self.speed_dc, high_byte, low_byte, 0xFF])
        try:
            self.ser.write(packet)
        except Exception as e:
            pass
            
    def timer_write_callback(self):
        # Watchdog
        if time.time() - self.last_command_time > self.watchdog_timeout:
            if self.dir_dc != 0 or self.speed_dc != 0 or self.dir_servo != 1500:
                self.dir_dc = 0
                self.speed_dc = 0
                self.dir_servo = 1500
        self.send_command()

    def timer_read_callback(self):
        """Olfatea el puerto serial en busca de los 7 bytes de telemetría"""
        try:
            # Mientras haya suficientes bytes para armar un paquete
            while self.ser.in_waiting >= 7:
                # Buscar la cabecera 0xAA
                if self.ser.read(1)[0] == 0xAA:
                    # Confirmar segunda cabecera 0x55
                    if self.ser.read(1)[0] == 0x55:
                        # Leer los 4 bytes de datos + 1 byte de cola
                        data = self.ser.read(5) 
                        if data[4] == 0xFF: # Trama perfecta
                            current_izq = (data[0] << 8) | data[1]
                            current_der = (data[2] << 8) | data[3]
                            self.process_encoder_data(current_izq, current_der)
        except Exception as e:
            pass

    def process_encoder_data(self, current_izq, current_der):
        """Calcula la velocidad real superando el desbordamiento de 16-bits"""
        # Calcular el cambio (Delta) de ticks desde la última lectura
        delta_izq = current_izq - self.last_izq
        delta_der = current_der - self.last_der
        
        # Compensación por desbordamiento del Timer de 16 bits del STM32 (0 a 65535)
        if delta_izq > 32767: delta_izq -= 65536
        elif delta_izq < -32768: delta_izq += 65536
        
        if delta_der > 32767: delta_der -= 65536
        elif delta_der < -32768: delta_der += 65536

        # Guardar para la siguiente iteración
        self.last_izq = current_izq
        self.last_der = current_der

        # ======== LA MAGIA DEL ESPEJEADO ========
        # Invertimos el delta del motor derecho por estar físicamente volteado
        delta_der = -delta_der 

        # Crear y empaquetar el mensaje de ROS
        msg = EncoderData()
        
        # Motor Izquierdo (M1)
        msg.vel_m1 = abs(delta_izq)
        if delta_izq > 0: msg.dir_m1 = 1      # Adelante
        elif delta_izq < 0: msg.dir_m1 = 2    # Atrás
        else: msg.dir_m1 = 0

        # Motor Derecho (M2)
        msg.vel_m2 = abs(delta_der)
        if delta_der > 0: msg.dir_m2 = 1      # Adelante
        elif delta_der < 0: msg.dir_m2 = 2    # Atrás
        else: msg.dir_m2 = 0

        self.pub_encoder.publish(msg)

    def destroy_node(self):
        self.dir_dc = 0
        self.speed_dc = 0
        self.dir_servo = 1500
        self.send_command()
        time.sleep(0.1)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32Controller()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()