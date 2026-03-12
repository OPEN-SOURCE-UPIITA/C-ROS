#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motor_msgs.msg import MotorCommand

from evdev import InputDevice, categorize, ecodes
import threading

class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__('key_publisher')

        self.publisher = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Estado vehículo
        self.dir_dc = 0
        self.speed_dc = 0
        self.dir_servo = 1500
        
        # --- NUEVO: Estado Luces ---
        self.stop_lights = 0
        self.turn_signals = 0

        # Parámetros control
        self.paso_servo = 15
        self.retorno_servo = 25
        self.paso_vel = 5
        self.friccion_vel = 10

        # Teclas activas
        self.keys_pressed = set()

        # --- AUTODETECCIÓN INTELIGENTE DEL TECLADO ---
        import evdev
        self.dev = None
        dispositivos = [evdev.InputDevice(path) for path in evdev.list_devices()]
        
        for d in dispositivos:
            if 'keyboard' in d.name.lower():
                self.dev = d
                break
                
        if self.dev is None:
            self.get_logger().error("❌ ¡No se detectó ningún teclado físico!")
            self.dev = evdev.InputDevice('/dev/input/event2')
        else:
            self.get_logger().info(f"✅ Teclado detectado en: {self.dev.path}")
            self.get_logger().info("🕹️ Controles: WASD (Mover) | Espacio (Freno) | K/L (Direccionales)")

        # Hilo que lee teclado
        self.thread = threading.Thread(target=self.read_keyboard)
        self.thread.daemon = True
        self.thread.start()

    def read_keyboard(self):
        for event in self.dev.read_loop():
            if event.type == ecodes.EV_KEY:
                key_event = categorize(event)
                key = key_event.keycode
                
                # evdev a veces devuelve una lista de teclas, extraemos la principal
                if isinstance(key, list): 
                    key = key[0]

                if key_event.keystate == key_event.key_down:
                    self.keys_pressed.add(key)
                elif key_event.keystate == key_event.key_up:
                    self.keys_pressed.discard(key)

    def timer_callback(self):
        up = 'KEY_W' in self.keys_pressed
        down = 'KEY_S' in self.keys_pressed
        left = 'KEY_A' in self.keys_pressed
        right = 'KEY_D' in self.keys_pressed
        space = 'KEY_SPACE' in self.keys_pressed
        esc = 'KEY_ESC' in self.keys_pressed
        
        # --- NUEVAS TECLAS ---
        key_k = 'KEY_K' in self.keys_pressed
        key_l = 'KEY_L' in self.keys_pressed

        if esc:
            rclpy.shutdown()
            return

        # ========================
        # 1. LÓGICA DE MOTORES DC
        # ========================
        if up and not down:
            self.dir_dc = 1
            self.speed_dc = min(100, self.speed_dc + self.paso_vel)
        elif down and not up:
            self.dir_dc = 2
            self.speed_dc = min(100, self.speed_dc + self.paso_vel)
        else:
            if self.speed_dc > 0:
                self.speed_dc = max(0, self.speed_dc - self.friccion_vel)
            if self.speed_dc == 0:
                self.dir_dc = 0

        # ========================
        # 2. LÓGICA DE DIRECCIÓN
        # ========================
        if left and not right:
            self.dir_servo = min(1740, self.dir_servo + self.paso_servo)
        elif right and not left:
            self.dir_servo = max(1110, self.dir_servo - self.paso_servo)
        else:
            if self.dir_servo > 1500:
                self.dir_servo = max(1500, self.dir_servo - self.retorno_servo)
            elif self.dir_servo < 1500:
                self.dir_servo = min(1500, self.dir_servo + self.retorno_servo)

        # ========================
        # 3. LÓGICA DE LUCES Y FRENO
        # ========================
        if space:
            self.dir_dc = 0
            self.speed_dc = 0
            self.dir_servo = 1500

        # Luces de Freno (Espacio = Freno Manual | dir_dc == 2 = Reversa)
        if space or self.dir_dc == 2:
            self.stop_lights = 1
        else:
            self.stop_lights = 0

        # Direccionales (K = Izquierda, L = Derecha)
        if key_k and key_l:
            self.turn_signals = 3  # Intermitentes (Ambas)
        elif key_k:
            self.turn_signals = 2  # Izquierda
        elif key_l:
            self.turn_signals = 1  # Derecha
        else:
            self.turn_signals = 0  # Apagadas

        # ========================
        # 4. PUBLICAR MENSAJE
        # ========================
        msg = MotorCommand()
        msg.dir_dc = self.dir_dc
        msg.speed_dc = self.speed_dc
        msg.dir_servo = self.dir_servo
        msg.stop_lights = self.stop_lights
        msg.turn_signals = self.turn_signals

        self.publisher.publish(msg)

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
