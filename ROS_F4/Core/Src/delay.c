#include "delay.h"

/// @brief nus延时
/// @param nus 延时的nus数
void delay_us(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD=nus*168-1; // 计数值加载
	SysTick->VAL=0x00; // 清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; // 开始计数
	do
	{
		temp=SysTick->CTRL; // 读取控制寄存器状态
	}while((temp&0x01)&&!(temp&(1<<16))); // temp&0x01:定时器使能，!(temp&(1<<16)):定时器计数值不为0
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; // 关闭计数
	SysTick->VAL=0x00;// 清空计数器
}

/// @brief nms延时
/// @param nus 延时的ms数
void delay_ms(uint32_t nms)
{
	uint32_t repeat=nms/50;
	uint32_t remain=nms%50;
	while(repeat)
	{
		delay_us(50*1000); // 延时 50 ms
		repeat--;
	}
	if(remain)
	{
		delay_us(remain*1000); // 延时remain ms
	}
}

