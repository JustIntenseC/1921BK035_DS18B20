#include "K1921VK035.h"
#include <stdio.h>
#include <string.h>

volatile uint8_t timerDone = 0;

#define DS18B20_PIN 15

char msg[64] = "K1921VK035 microcontroller - UART1 READY\n";
#define MAX_PACKET_SIZE 256

volatile uint8_t packet_buffer[MAX_PACKET_SIZE];
volatile uint16_t packet_index = 0;
volatile uint8_t packet_received = 0; 

typedef enum{
    STATUS_OFF,
    STATUS_ON
} LED_STATUS;


typedef enum{  // state
    IDLE,
    MEASURING,
    READING,
    ERROR
}   STATE;

typedef enum {
    START_MEASURING,
    MEASURING_DONE,
    READ_DONE,
    SENSOR_ERROR,
    NO_EVENT

}   EVENT;

typedef struct{
    UART_TypeDef *Instance;
    uint32_t BaudRate;
    UART_LCRH_WLEN_Enum WLEN;
    uint32_t  STOPBIT;
    uint32_t RXENABLE;
    uint32_t TXENABLE;
}   UART_AbstractionTypeDef;


uint32_t status = 0;
LED_STATUS Led_Pin_ON(void);
LED_STATUS Led_Pin_OFF(void);
void Led_Pin_Toggle(void);
static void Led_Pin_Init(void);

static void Button_Init(void);


STATE HandleEventDS18B20(STATE curState, EVENT event);

STATE DS18B20_Reset(void);
void DS18B20_WriteBit(uint8_t bit);
void DS18B20_WriteByte(uint8_t);
uint8_t DS18B20_ReadBit(void);
uint8_t DS18B20_ReadByte(void);
void DS18B20_ReadScratchpad(uint8_t *scratchpad);
void DS18B20_ReadROM(uint8_t *rom);
uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t len);

static void USB_UART_Init(void);
void USB_UART_SendByte(uint8_t *byte);
void USB_UART_SendMessage(uint8_t *msg, uint16_t length);
void User_UartInit(UART_AbstractionTypeDef *huart);

static void Timer_Init(void);
void Timer_Delay_Ticks(uint32_t microseconds);

static void GPIO_Init(void);

void Error_Handler();




