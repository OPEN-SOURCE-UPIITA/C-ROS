/**
 * @file led_portting.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 板载LED灯控制实例及接口实现
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "led.h"
#include "gpio.h"
#include "global_conf.h"
#include "packet.h"
#include "lwmem_porting.h"

/* 全系统全局变量 */
LEDObjectTypeDef *leds[LED_NUM];
static osMessageQueueId_t led_ctrl_ququeHandle[LED_NUM]; /* LED1 控制队列Handle */

static void led_set_pin(LEDObjectTypeDef *self, uint32_t level);   /* LED_SYS 灯写 IO 口接口 */
static int put_ctrl_block(LEDObjectTypeDef *self, LEDCtrlTypeDef *p);   /* 控制入队接口 */
static int get_ctrl_block(LEDObjectTypeDef *self, LEDCtrlTypeDef *p);   /* 控制出队接口 */

/**
  * @brief 初始化led相关内存、变量
  * @retval None.
*/
void leds_init(void)
{
	/* 建立 LED1 控制队列 */
	const osMessageQueueAttr_t led_ctrl_quque_attributes[LED_NUM] = {{ .name = "led1_ctrl_quque" },
                                                                      { .name = "led2_ctrl_quque" },
                                                                      { .name = "led3_ctrl_quque" }};

    for(int i = 0 ; i < LED_NUM ; i++){
        led_ctrl_ququeHandle[i] = osMessageQueueNew(5, sizeof(LEDCtrlTypeDef), &led_ctrl_quque_attributes[i]);
        leds[i] = LWMEM_CCM_MALLOC(sizeof(LEDObjectTypeDef)); 
        led_object_init(leds[i]);
        leds[i]->id = i+1;
        leds[i]->set_pin = led_set_pin;
        leds[i]->get_ctrl_block = get_ctrl_block;
        leds[i]->put_ctrl_block = put_ctrl_block;
    }

   // packet_register_callback(&packet_controller, PACKET_FUNC_LED, packet_handler);
}

/**
  * @brief 定时器回调
  * @detials 定时刷新LED灯状态
  * @retval None.
  *
*/
void led_timer_callback(void *argument) {
    for(int i = 0 ; i < LED_NUM ; i++)
    {
        led_task_handler(leds[i], LED_TASK_PERIOD);
    }
}


static void led_set_pin(LEDObjectTypeDef *self, uint32_t level)
{
    if(self->id == 1){
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (GPIO_PinState)(level ^ 1u));
    }else if(self->id == 2){
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (GPIO_PinState)(level ^ 1u));
    }else{
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (GPIO_PinState)(level ^ 0u));
    }
}


/**
  * @brief LED控制队列入队接口
  * @param [in] p 要出队的控制参数结构体指针
  * @retval 0 成功
  * @retval !=0 失败 
  *
*/
static int put_ctrl_block(LEDObjectTypeDef *self, LEDCtrlTypeDef *p) {
    if(self->id == 1){
        return (int)osMessageQueuePut(led_ctrl_ququeHandle[0], p, 0, 0);
    }else if(self->id == 2){
        return (int)osMessageQueuePut(led_ctrl_ququeHandle[1], p, 0, 0);
    }else{
        return (int)osMessageQueuePut(led_ctrl_ququeHandle[2], p, 0, 0);
    }
}

/**
  * @brief LED 控制队列出队接口
  * @param [out] p 出队数据的存储指针
  * @retval 0 成功
  * @retval !=0 失败 
  *
*/
static int get_ctrl_block(LEDObjectTypeDef *self, LEDCtrlTypeDef *p) {
    if(self->id == 1){
        return (int)osMessageQueueGet(led_ctrl_ququeHandle[0], p, 0, 0);
    }else if(self->id == 2){
        return (int)osMessageQueueGet(led_ctrl_ququeHandle[1], p, 0, 0);
    }else{
        return (int)osMessageQueueGet(led_ctrl_ququeHandle[2], p, 0, 0);
    }
}

