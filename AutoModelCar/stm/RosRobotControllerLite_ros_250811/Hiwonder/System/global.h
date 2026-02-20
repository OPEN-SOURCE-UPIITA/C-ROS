#ifndef __GLOBAL_H_
#define __GLOBAL_H_

#include "global_conf.h"

#include "SEGGER_RTT.h"
#include "main.h"

#include "stm32f4xx_hal.h"

#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "dma.h"
#include "tim.h"
#include "lwrb.h"
#include "lwmem.h"

#include "button.h"
#include "led.h"
#include "buzzer.h"
#include "encoder_motor.h"
#include "pid.h"
#include "log.h"
#include "pwm_servo.h"
#include "packet.h"
#include "object.h"
#include "chassis.h"
#include "serial_servo.h"
#include "rgb_spi.h"

// 全系统全局变量
extern struct PacketController packet_controller;
extern ButtonObjectTypeDef *buttons[2];
extern BuzzerObjectTypeDef *buzzers[1];
extern LEDObjectTypeDef *leds[LED_NUM];
extern PWMServoObjectTypeDef *pwm_servos[4];
extern EncoderMotorObjectTypeDef *motors[4];
extern ChassisTypeDef *chassis;
extern SerialServoControllerTypeDef serial_servo_controller;


void set_chassis_type(uint8_t chassis_type);
void change_battery_limit(uint16_t limit);

#endif

