#include "global.h"

void DelayUs(uint32_t t)
{
//	uint32_t cycles = SystemCoreClock / 1000000 * t;
	for(uint32_t i = 0; i < t * 7; ++i) {
        __asm volatile("NOP");
	}
}

void I2C_SDA_OUT(void)//SDA输出方向配置
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_Pin; // 指定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
    GPIO_InitStruct.Pull = GPIO_NOPULL; // 无上拉/下拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速
    HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);
}

void I2C_SDA_IN(void)//SDA输入方向配置
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_Pin; // 指定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 输入模式
    GPIO_InitStruct.Pull = GPIO_PULLUP; // 上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速
    HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);
}

//以下为模拟IIC总线函数
void IIC_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_Pin; // 指定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
    GPIO_InitStruct.Pull = GPIO_NOPULL; // 无上拉/下拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速
    HAL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C_SCL_Pin; // 指定引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
    GPIO_InitStruct.Pull = GPIO_NOPULL; // 无上拉/下拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速
    HAL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)1);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
}

void IIC_start()	//起始信号
{
    I2C_SDA_OUT();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)1);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
    DelayUs(4);
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)0); //START:when CLK is high,DATA change form high to low
    DelayUs(4);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0); //钳住I2C总线，准备发送或接收数据
}

void IIC_stop()		//终止信号
{
    I2C_SDA_OUT();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)0); //STOP:when CLK is high DATA change form low to high
    DelayUs(4);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)1); //发送I2C总线结束信号
    DelayUs(4);
}

//主机产生一个应答信号
void IIC_ack()
{
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
    I2C_SDA_OUT();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)0);
    DelayUs(2);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
    DelayUs(2);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
}

//主机不产生应答信号
void IIC_noack()
{
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
    I2C_SDA_OUT();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)1);
    DelayUs(2);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
    DelayUs(2);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
}

//等待从机应答信号
//返回值：1 接收应答失败
//		  0 接收应答成功
uint8_t IIC_wait_ack()
{
    uint8_t tempTime = 0;
    I2C_SDA_IN();
    HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)1);
    DelayUs(1);
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
    DelayUs(1);

    while(HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)) {
        tempTime++;
        if(tempTime > 250) {
            IIC_stop();
            return 1;
        }
    }

    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
    return 0;
}

void IIC_send_byte(uint8_t txd)
{
    uint8_t i = 0;
    I2C_SDA_OUT();
    HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0); //拉低时钟开始数据传输
    for(i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (GPIO_PinState)((txd & 0x80) >> 7)); //读取字节
        txd <<= 1;
        DelayUs(2);
        HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
        DelayUs(2); //发送数据
        HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
        DelayUs(2);
    }
}

//读取一个字节
uint8_t IIC_read_byte(uint8_t ack)
{
    uint8_t i = 0, receive = 0;
    I2C_SDA_IN();
    for(i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)0);
        DelayUs(2);
        HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, (GPIO_PinState)1);
        receive <<= 1; //左移
        if(HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)) {
            receive++;    //连续读取八位
        }
        DelayUs(1);
    }

    if(!ack) {
        IIC_noack();
    } else {
        IIC_ack();
    }

    return receive;//返回读取到的字节
}


int IIC_WriteToMem(uint8_t address, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    IIC_start();
    IIC_send_byte(address);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    IIC_send_byte(reg_addr);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    for(int i = 0; i < length; i++) {
        IIC_send_byte(data[i]);
        if(IIC_wait_ack() != 0) {
						IIC_stop();
            return 1;
        }
    }
		IIC_stop();
		return 0;
}


int IIC_Write(uint8_t address, uint8_t *data, uint8_t length)
{
    IIC_start();
    IIC_send_byte(address);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    for(int i = 0; i < length; i++) {
        IIC_send_byte(data[i]);
        if(IIC_wait_ack() != 0) {
						IIC_stop();
            return 1;
        }
    }
		IIC_stop();
		return 0;
}

int IIC_ReadFromMem(uint8_t address, uint8_t reg_addr, uint8_t *buf, uint8_t length)
{
    IIC_start();
    IIC_send_byte(address);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
    IIC_send_byte(reg_addr);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
	  IIC_start();
    IIC_send_byte(address | 0x01);
    if(IIC_wait_ack() != 0) {
        return 1;
    }
		while(length) {
			*buf = IIC_read_byte((length > 1) ? 1 : 0);
			length--;
			buf++;
    }
		IIC_stop();
		return 0;
}
