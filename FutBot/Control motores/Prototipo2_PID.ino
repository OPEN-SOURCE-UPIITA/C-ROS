#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <ESP32Encoder.h>
#include <PID_v1.h> // <--- NUEVA LIBRERÍA

#define LED_PIN 48 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- PINES DE LOS MOTORES ---
#define ENA_L 4  
#define IN1_L 5  
#define IN2_L 6  

#define ENB_R 7  
#define IN3_R 15 
#define IN4_R 16 

// --- PINES DE LOS ENCODERS ---
#define ENC_L_A 10 
#define ENC_L_B 11 
#define ENC_R_A 12 
#define ENC_R_B 13 

// ==========================================
// --- CONSTANTES FÍSICAS EN MILÍMETROS ---
// ==========================================
const float TICKS_PER_REV = 578.0; 
const float WHEEL_RADIUS = 34.0; 
const float WHEEL_CIRCUMFERENCE = 2.0 * PI * WHEEL_RADIUS; 
const float L = 200.0; 

// ==========================================
// --- VARIABLES DEL CONTROLADOR PID ---
// ==========================================
// Variables obligatorias como 'double' para la librería PID
double Setpoint_L = 0, Input_L = 0, Output_L = 0;
double Setpoint_R = 0, Input_R = 0, Output_R = 0;

// Constantes de sintonización (¡Aquí está la magia!)
double Kp = 0.09, Ki = 1, Kd = 0.0;

// Instancias de los controladores PID
PID pid_left(&Input_L, &Output_L, &Setpoint_L, Kp, Ki, Kd, DIRECT);
PID pid_right(&Input_R, &Output_R, &Setpoint_R, Kp, Ki, Kd, DIRECT);

// Direcciones deseadas (1 = Adelante, -1 = Atrás, 0 = Detenido)
int dir_L = 0;
int dir_R = 0;

// --- OBJETOS MICRO-ROS ---
rcl_subscription_t subscriber_cmd_vel;
geometry_msgs__msg__Twist msg_twist;

rcl_publisher_t pub_vel_left;
rcl_publisher_t pub_vel_right;
std_msgs__msg__Float32 msg_vel_left;  
std_msgs__msg__Float32 msg_vel_right; 

rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- OBJETOS ENCODER ---
ESP32Encoder encoder_left;
ESP32Encoder encoder_right;

int32_t last_ticks_l = 0;
int32_t last_ticks_r = 0;
unsigned long last_time = 0;

void error_loop(){
  while(1){
    neopixelWrite(LED_PIN, 255, 0, 0);
    delay(100);
    neopixelWrite(LED_PIN, 0, 0, 0);
    delay(100);
  }
}

// Nueva función setMotor orientada al PID
void setMotor(int pwm_pin, int in1_pin, int in2_pin, double pwm_val, int direction, bool invert_physical) {
  
  // Invertir lógicamente si el motor está montado en espejo
  if (invert_physical) {
    direction = -direction;
  }

  // Aplicar límite estricto de seguridad al PWM
  int pwm_out = min(max((int)pwm_val, 0), 255);

  if (direction == 1) {
    digitalWrite(in1_pin, HIGH); digitalWrite(in2_pin, LOW); // Adelante
    analogWrite(pwm_pin, pwm_out);
  } else if (direction == -1) {
    digitalWrite(in1_pin, LOW); digitalWrite(in2_pin, HIGH); // Atrás
    analogWrite(pwm_pin, pwm_out);
  } else {
    digitalWrite(in1_pin, LOW); digitalWrite(in2_pin, LOW);  // Freno duro
    analogWrite(pwm_pin, 0);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    unsigned long current_time = millis();
    unsigned long delta_time = current_time - last_time;

    if (delta_time > 0) {
      int32_t current_ticks_l = encoder_left.getCount();
      int32_t current_ticks_r = encoder_right.getCount();

      int32_t delta_ticks_l = current_ticks_l - last_ticks_l;
      int32_t delta_ticks_r = current_ticks_r - last_ticks_r;

      float delta_rev_l = (float)delta_ticks_l / TICKS_PER_REV;
      float delta_rev_r = (float)delta_ticks_r / TICKS_PER_REV;

      float dt_segundos = (float)delta_time / 1000.0;
      
      // 1. Calculamos velocidad real (conservando el signo para publicarla)
      float v_real_l = (delta_rev_l * WHEEL_CIRCUMFERENCE) / dt_segundos;
      float v_real_r = (delta_rev_r * WHEEL_CIRCUMFERENCE) / dt_segundos;

      // 2. Alimentamos el PID con el valor ABSOLUTO (magnitud del error)
      Input_L = abs(v_real_l);
      Input_R = abs(v_real_r);

      // 3. Calculamos el nuevo PWM solo si se requiere movimiento
      if (Setpoint_L > 0) {
        pid_left.Compute();
      } else {
        Output_L = 0; // Apagar motor si el setpoint es 0
      }

      if (Setpoint_R > 0) {
        pid_right.Compute();
      } else {
        Output_R = 0;
      }

      // 4. Mandamos la potencia calculada a los motores
      setMotor(ENA_L, IN1_L, IN2_L, Output_L, dir_L, true); 
      setMotor(ENB_R, IN3_R, IN4_R, Output_R, dir_R, false);

      // 5. Publicar métricas reales a ROS 2
      msg_vel_left.data = v_real_l;
      msg_vel_right.data = v_real_r;
      RCSOFTCHECK(rcl_publish(&pub_vel_left, &msg_vel_left, NULL));
      RCSOFTCHECK(rcl_publish(&pub_vel_right, &msg_vel_right, NULL));

      last_ticks_l = current_ticks_l;
      last_ticks_r = current_ticks_r;
      last_time = current_time;
    }
  }
}

// Callback de movimiento modificado para el PID
void twist_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float v_left_cmd = msg->linear.x - (msg->angular.z * L / 2.0);
  float v_right_cmd = msg->linear.x + (msg->angular.z * L / 2.0);
  
  // Asignar los Setpoints como valores absolutos (magnitud de la velocidad)
  Setpoint_L = abs(v_left_cmd);
  Setpoint_R = abs(v_right_cmd);

  // Extraer la dirección deseada
  if (v_left_cmd > 5.0) dir_L = 1;
  else if (v_left_cmd < -5.0) dir_L = -1;
  else dir_L = 0;

  if (v_right_cmd > 5.0) dir_R = 1;
  else if (v_right_cmd < -5.0) dir_R = -1;
  else dir_R = 0;

  neopixelWrite(LED_PIN, 0, 0, 64); 
  delay(5);
  neopixelWrite(LED_PIN, 0, 0, 0);
}

void setup() {
  set_microros_transports(); 

  pinMode(ENA_L, OUTPUT); pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT);
  pinMode(ENB_R, OUTPUT); pinMode(IN3_R, OUTPUT); pinMode(IN4_R, OUTPUT);
  setMotor(ENA_L, IN1_L, IN2_L, 0, 0, true);
  setMotor(ENB_R, IN3_R, IN4_R, 0, 0, false);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_left.attachHalfQuad(ENC_L_A, ENC_L_B); 
  encoder_right.attachHalfQuad(ENC_R_A, ENC_R_B);
  encoder_left.clearCount();
  encoder_right.clearCount();
  last_time = millis();

  // --- CONFIGURACIÓN DEL PID ---
  pid_left.SetMode(AUTOMATIC);
  pid_right.SetMode(AUTOMATIC);
  pid_left.SetSampleTime(50); // Mismo tiempo que el timer de ROS (50ms)
  pid_right.SetSampleTime(50);
  pid_left.SetOutputLimits(0, 255); // Rango de PWM
  pid_right.SetOutputLimits(0, 255);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_base_controller", "", &support));

  RCCHECK(rclc_subscription_init_default(&subscriber_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(&pub_vel_left, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_vel_left"));
  RCCHECK(rclc_publisher_init_default(&pub_vel_right, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_vel_right"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &msg_twist, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(5);
}