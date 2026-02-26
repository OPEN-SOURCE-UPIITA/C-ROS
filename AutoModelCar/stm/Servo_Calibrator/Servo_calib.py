import serial
import curses
import time

PUERTO = '/dev/ttyACM0'
BAUDIOS = 115200

try:
    ser = serial.Serial(PUERTO, BAUDIOS, timeout=0.1)
except Exception as e:
    print(f"Error abriendo el puerto {PUERTO}: {e}")
    exit()

def send_servo_value(val):
    # Separar el número grande (ej. 1500) en dos bytes pequeños
    high_byte = (val >> 8) & 0xFF
    low_byte = val & 0xFF
    packet = bytearray([0xAA, 0x55, high_byte, low_byte, 0xFF])
    ser.write(packet)

def main(stdscr):
    stdscr.nodelay(1) 
    curses.noecho()   
    
    # Valores iniciales
    servo_pwm = 1500  # Centro exacto
    paso = 10         # Cuánto se mueve con cada clic

    # Enviar posición inicial
    send_servo_value(servo_pwm)

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "=== CALIBRADOR DE LÍMITES DE DIRECCIÓN ===")
        stdscr.addstr(2, 0, "CONTROLES:")
        stdscr.addstr(3, 0, "[<--] Izquierda | [-->] Derecha")
        stdscr.addstr(4, 0, "[W] Aumentar paso | [S] Reducir paso")
        stdscr.addstr(5, 0, "[ESPACIO] Centrar a 1500 | [Q] Salir")
        
        # Interfaz de datos en tiempo real
        stdscr.addstr(7, 0, f"--> VALOR PWM ACTUAL: {servo_pwm} us")
        stdscr.addstr(8, 0, f"--> TAMAÑO DEL PASO:  {paso}")

        key = stdscr.getch()

        if key != -1:
            if key == curses.KEY_LEFT:
                servo_pwm -= paso
            elif key == curses.KEY_RIGHT:
                servo_pwm += paso
            elif key == ord('w') or key == ord('W'):
                paso += 5
            elif key == ord('s') or key == ord('S'):
                paso = max(1, paso - 5) # Evitar que el paso sea 0 o negativo
            elif key == ord(' '):
                servo_pwm = 1500
            elif key == ord('q') or key == ord('Q'):
                break

            # Limitar para no destruir el servo internamente (límites estándar)
            if servo_pwm < 500: servo_pwm = 500
            if servo_pwm > 2500: servo_pwm = 2500

            # Enviar solo cuando hay un cambio
            send_servo_value(servo_pwm)
        
        stdscr.refresh()
        time.sleep(0.05)

curses.wrapper(main)
send_servo_value(1500) # Dejarlo centrado al salir
ser.close()