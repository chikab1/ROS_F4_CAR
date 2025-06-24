#include <string.h>
#include "jy901.h"
#include "usart.h"

User_USART JY901_data;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SQ stcQ;

const unsigned char header[1]  = {0x51};			//帧头

//				用于修改jy901内容                      //
uint8_t lock[5] = {0xFF,0xAA,0x69,0x88,0xB5};
uint8_t change[5] = {0xFF,0xAA,0x02,0x0E,0x02};
uint8_t change1[5] = {0xFF,0xAA,0x04,0x06,0x00};
uint8_t save[5] = {0xFF,0xAA,0x00,0x00,0x00};
///////////////////////////////////////////////////////

float x_speed = 0.0f;
#define SAMPLE_INTERVAL 0.01f  // 假设每次采样间隔为 10ms，即0.01秒
float last_acc = 0.0f;

extern uint32_t left_count;
extern uint32_t right_count;
extern float left_speed;
extern float right_speed;

//接收结构体初始化
void User_USART_Init(User_USART *Data)
{
    for(uint16_t i=0; i < RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
    Data->frame_head = 0x55;
    Data->Rx_flag = 0;
    Data->Rx_len = 0;
    HAL_UART_Transmit(&huart1, lock, 5,0xff);
    HAL_Delay(200);
	HAL_UART_Transmit(&huart1, change1, 5,0xff);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart1, save, 5,0xff);
}

void JY901_Process(void)
{
    if (JY901_data.Rx_len < RXBUFFER_LEN) return;    //如果位数不对

    for (uint8_t i = 0; i < 4; i++)
    {
        if (JY901_data.RxBuffer[i * 11] != JY901_data.frame_head) return;    					//如果帧头不对
        switch (JY901_data.RxBuffer[i * 11 + 1])
        {
            case 0x51:
                memcpy(&stcAcc, &JY901_data.RxBuffer[2 + i * 11], 8);
                for (uint8_t j = 0; j < 3; j++)
                    JY901_data.acc.a[j] = (float) stcAcc.a[j] / 32768 * 16;                     //官方加速度解算
                break;
            case 0x52:
                memcpy(&stcGyro, &JY901_data.RxBuffer[2 + i * 11], 8);
                for (uint8_t j = 0; j < 3; j++)
                    JY901_data.w.w[j] = (float) stcGyro.w[j] / 32768 * 2000;                    //官方角速度解算
                break;
            case 0x53:
                memcpy(&stcAngle, &JY901_data.RxBuffer[2 + i * 11], 8);
                for (uint8_t j = 0; j < 3; j++)
                    JY901_data.angle.angle[j] = (float) stcAngle.Angle[j] / 32768 * 180;        //官方角度解算
                break;
            case 0x59:
				memcpy(&stcQ, &JY901_data.RxBuffer[2 + i*11], 8);
				for(uint8_t j = 0; j < 4; j++)
					JY901_data.q.q[j] = (float) stcQ.q[j] / 32768;		       					//官方四元数解算
				break;
        }
    }
}

void send_raspi(void)
{
//    char buffer[256];
//    int length = snprintf(buffer, sizeof(buffer),
//        "0x51,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
//        0x51,
//        (int)(JY901_data.acc.a[0] * 1000), // 放大1000倍转为整数
//        (int)(JY901_data.acc.a[1] * 1000),
//        (int)(JY901_data.acc.a[2] * 1000),
//        (int)(JY901_data.w.w[0] * 1000),
//        (int)(JY901_data.w.w[1] * 1000),
//        (int)(JY901_data.w.w[2] * 1000),
//        (int)(JY901_data.q.q[0] * 1000),
//        (int)(JY901_data.q.q[1] * 1000),
//        (int)(JY901_data.q.q[2] * 1000),
//        (int)(JY901_data.q.q[3] * 1000));
//
//    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length,100);
    char buffer[128];

    // 将 m/s 转为 mm/s 并放大 1000 倍为整数发送
    int length = snprintf(buffer, sizeof(buffer),
        "0x52,%d,%d,%d,%d\n",
        left_count,
        right_count,
        (int)(left_speed * 1000.0f),     // m/s -> mm/s
        (int)(right_speed * 1000.0f));   // m/s -> mm/s

    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, length, 100);
}
void update_velocity(void)
{
	x_speed +=	(last_acc + JY901_data.acc.a[0]) * 0.5f * SAMPLE_INTERVAL;
	last_acc = JY901_data.acc.a[0];
}

