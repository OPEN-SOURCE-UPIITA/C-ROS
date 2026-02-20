/**
 * @file app.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 主应用逻辑
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "cmsis_os2.h"
#include "led.h"
#include "lwmem_porting.h"
#include "global.h"
#include "global.h"
#include "adc.h"
#include "u8g2_porting.h"
#include "packet_reports.h"
#include "packet_handle.h"
#include "serial_servo.h"
#include "rgb_spi.h"
#include "iwdg.h"

void buzzers_init(void);
void buttons_init(void);
void leds_init(void);
//void motors_init(void);
void serial_servo_init(void);
void pwm_servos_init(void);
void chassis_init(void);


void button_event_callback(ButtonObjectTypeDef *button,  ButtonEventIDEnum event)
{
    PacketReportKeyEventTypeDef report = {
        .key_id = button->id,
        .event = (uint8_t)(int)event,
    };
    packet_transmit(&packet_controller, PACKET_FUNC_KEY, &report, sizeof(PacketReportKeyEventTypeDef));
	if(event == BUTTON_EVENT_CLICK) {
		buzzer_didi(buzzers[0], 2000, 50, 50, 1);
	}
}

void app_task_entry(void *argument)
{
    extern osTimerId_t led_timerHandle;
    extern osTimerId_t buzzer_timerHandle;
    extern osTimerId_t button_timerHandle;
    extern osTimerId_t battery_check_timerHandle;
    extern osMessageQueueId_t moving_ctrl_queueHandle;
    
    leds_init();
//    motors_init(); 
    set_chassis_type(CHASSIS_TYPE_TANKBLACK);
    pwm_servos_init();
	serial_servo_init();
    buzzers_init();
    buttons_init();
	WS2812b_Configuration();
    
    button_register_callback(buttons[0], button_event_callback);
    button_register_callback(buttons[1], button_event_callback);
    
    osTimerStart(led_timerHandle, LED_TASK_PERIOD);
    osTimerStart(buzzer_timerHandle, BUZZER_TASK_PERIOD);
    osTimerStart(button_timerHandle, BUTTON_TASK_PERIOD);
    osTimerStart(battery_check_timerHandle, BATTERY_TASK_PERIOD);
    packet_handle_init();
    
//    osDelay(1000);
//    buzzer_didi(buzzers[0],1500 , 100,50,1);
//  
//    char msg = '\0';
//    uint8_t msg_prio;
//    osMessageQueueReset(moving_ctrl_queueHandle);
    
//    chassis_init();
//    set_chassis_type(CHASSIS_TYPE_TI4WD);
//    led_on(leds[0]);
//        led_off(leds[1]);
//        led_flash(leds[2] , 100 , 1000 , 0);
//    uint8_t rgb[6] = {0 , 0 , 0,0,0,0};
//        rgb[0] = 0;
//        rgb[1] = 0;
//        rgb[2] = 0xFF;
//        set_id_rgb_color(1,rgb);
//        osDelay(100);
//    set_rgb_color(rgb);
//    uint8_t rgb[3] = {0 , 0 , 250};
//    set_id_rgb_color(0,rgb);
//    osDelay(500);
    
    for(;;) {
//        set_id_rgb_color(0 , rgb);
//        set_id_rgb_color(1 , rgb);
//        rgb_SendArray();
//        uint8_t id = 0;
//        serial_servo_read_id(&serial_servo_controller , 0xFE , &id);
//        serial_servo_set_position(&serial_servo_controller , 1 , 600 , 500);
//        printf("id:%d\r\n",id);
//        osDelay(1000);
//        serial_servo_set_position(&serial_servo_controller , 1 , 300 , 500);
//        rgb[0] = 0xFF;
//        rgb[1] = 0;
//        rgb[2] = 0;
//        set_id_rgb_color(1,rgb);
//        buzzer_didi(buzzers[0] , 2000 , 100 , 300 , 1);
//        for(int i = 0 ; i < 4 ; i++){
//            motors[i]->pid_controller.set_point = 0.5;
//        }
//        for(int i = 0 ; i < 4 ; i++){
//            pwm_servo_set_position( pwm_servos[i], 500,500);
//            osDelay(1000);
//            pwm_servo_set_position( pwm_servos[i], 2500,500);
//            osDelay(2000);
//        }
//        for(int i = 0 ; i < 4 ; i++){
//            motors[i]->pid_controller.set_point = 0.0;
//        }
//        struct Pixel rgb = {1,255,0,0};
//        set_rgb_color(1,&rgb);
        HAL_IWDG_Refresh(&hiwdg);
        ATOMIC_SET_BIT(huart1.Instance->CR1, (USART_CR1_RXNEIE));
        osDelay(10);
//        rgb[0] = 0;
//        rgb[1] = 0;
//        rgb[2] = 255;
//        set_id_rgb_color(1,rgb);
//        osDelay(100);
//        rgb[0] = 255;
//        rgb[1] = 0;
//        rgb[2] = 0;
//        set_id_rgb_color(1,rgb);
//        osDelay(500);
    }
}

