#include "main.h"

UART_AbstractionTypeDef huart1;
uint8_t scratchpad[9];
uint8_t rom_address[8];

int main(void){
    
    SystemCoreClockUpdate();
    
    Led_Pin_Init();
    Button_Init();
    GPIO_Init();
    Timer_Init();
    USB_UART_Init();
    float temperature = 0x00;
    char message[256];

    STATE currentState = IDLE;
    EVENT events [] =
    {
        START_MEASURING,
        MEASURING_DONE,
        READ_DONE,

    };
    uint8_t countOfEvents = sizeof(events) / sizeof(events[0]);
    while(1){
        for(int i = 0; i<countOfEvents;i++){
            currentState = HandleEventDS18B20(currentState, events[i]);
        }
        if(currentState == IDLE){
            DS18B20_ReadROM(&rom_address);
            int16_t temperature_raw = (scratchpad[1] << 8) | scratchpad[0];
            temperature = (float)temperature_raw / 16.0f;
            snprintf(message, sizeof(message), "ROM Address: %02X%02X%02X%02X%02X%02X%02X%02X ", 
            rom_address[0], rom_address[1], rom_address[2], rom_address[3],
            rom_address[4], rom_address[5], rom_address[6], rom_address[7]);
            USB_UART_SendMessage(message, strlen(message));
            snprintf(message, sizeof(message), "Temperature DS18B20: %.2f ", temperature);
            USB_UART_SendMessage(message, strlen(message));
            uint8_t CRC = DS18B20_CRC8(&scratchpad, 8);
            if(CRC == scratchpad[8]) snprintf(message, sizeof(message), "CRC8: %d [OK]\n", CRC);
            else snprintf(message, sizeof(message), "CRC8: %d [ERROR]\n", CRC);
            USB_UART_SendMessage(message, strlen(message));            
        }
    }
    return 0;
}

void Led_Pin_Toggle(void){
    GPIOA->DATAOUTTGL_bit.PIN8  = 1;
}

static void Led_Pin_Init(void){

    RCU->HCLKCFG_bit.GPIOAEN = 1;
    RCU->HRSTCFG_bit.GPIOAEN = 1;
    GPIOA->OUTMODE_bit.PIN8 = GPIO_OUTMODE_PIN0_PP;
    GPIOA->DENSET_bit.PIN8 = 1;
    GPIOA->OUTENSET_bit.PIN8 = 1;
 

}
static void Button_Init(void){

    RCU->HCLKCFG_bit.GPIOAEN = 1;
    RCU->HRSTCFG_bit.GPIOAEN = 1;
    GPIOA->DENSET_bit.PIN7 = 1;
    GPIOA->INTTYPESET_bit.PIN7 = 0; 
    GPIOA->INTPOLSET_bit.PIN7 = 0;
    GPIOA->INTENSET_bit.PIN7 = 1;
    NVIC_EnableIRQ(GPIOA_IRQn);

}
void GPIOA_IRQHandler(void){
    GPIOA->INTSTATUS_bit.PIN7 = 1;
    Led_Pin_Toggle();
    status = (GPIOA->DATA & (1<<7)) >> 7;
        if(status == STATUS_ON){
            Led_Pin_OFF();
        }

}
LED_STATUS Led_Pin_ON(void){

    GPIOA->DATAOUTSET_bit.PIN8 = 1;
    return STATUS_ON;

}
LED_STATUS Led_Pin_OFF(void){

    GPIOA->DATAOUTCLR_bit.PIN8 = 1;
    return STATUS_OFF;
}

STATE DS18B20_Reset(void){
    volatile uint32_t timeout = 100000;
    GPIOB->DATAOUTCLR_bit.PIN15 = 1;
    Timer_Delay_Ticks(48000);
    GPIOB->DATAOUTSET_bit.PIN15 = 1;
    while(((GPIOB->DATA & (1 << DS18B20_PIN)) >> DS18B20_PIN == 1) && timeout){
          if(--timeout == 0){ USB_UART_SendMessage("NOT RESPONSE DS18B20\n", strlen("NOT RESPONSE DS18B20\n")); return SENSOR_ERROR; 
        };
    };
    while(((GPIOB->DATA & (1 << DS18B20_PIN)) >> DS18B20_PIN == 0) && timeout){
        if(--timeout == 0){ USB_UART_SendMessage("NOT RESPONSE DS18B20\n", strlen("NOT RESPONSE DS18B20\n")); return SENSOR_ERROR;
        };    
    };
    volatile uint16_t pinStateDS18B20 = ((GPIOB -> DATA) & (1 << DS18B20_PIN)) >> DS18B20_PIN;
    if(pinStateDS18B20 == 1){
        Timer_Delay_Ticks(30000);
    }
    else{
        return ERROR;
    }
}


void DS18B20_WriteBit(uint8_t bit){
    GPIOB->DATAOUTCLR_bit.PIN15 = 1; 
    Timer_Delay_Ticks( bit ? 100 : 6000);
    GPIOB->DATAOUTSET_bit.PIN15 = 1;
    Timer_Delay_Ticks(6000); 

}

void DS18B20_WriteByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20_WriteBit(byte & 0x01);
        byte = byte >> 1;
    }
}

uint8_t DS18B20_ReadBit(void){
    volatile uint8_t bit;
    GPIOB->DATAOUTCLR_bit.PIN15 = 1;
    Timer_Delay_Ticks(100 * 10);
    GPIOB->DATAOUTSET_bit.PIN15 = 1;
    Timer_Delay_Ticks(100 * 15);
    bit  = ((GPIOB->DATA & (1 << DS18B20_PIN)) >> DS18B20_PIN) & 0x01 ;
    Timer_Delay_Ticks(60 * 100);
    return bit;
}

uint8_t DS18B20_ReadByte(void) {
    volatile uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte |= (DS18B20_ReadBit() << i);
    }
    return byte;
}
void DS18B20_ReadROM(uint8_t *rom){
    DS18B20_Reset();
    DS18B20_WriteByte(0x33);
    for(int i=0;i<8;i++){
        rom[i] = DS18B20_ReadByte();
    }
    
}

void DS18B20_ReadScratchpad(uint8_t *scratchpad){
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);
    DS18B20_WriteByte(0x44);
    Timer_Delay_Ticks(750 * 100 * 1000); // 750 ms
    DS18B20_Reset();
    DS18B20_WriteByte(0xCC);
    DS18B20_WriteByte(0xBE);
    for (uint8_t i = 0; i < 9; i++) {
        scratchpad[i] = DS18B20_ReadByte();
    }
    Timer_Delay_Ticks(25000000);
    Led_Pin_ON();
    Timer_Delay_Ticks(25000000);
    Led_Pin_OFF();
    Timer_Delay_Ticks(25000000);
    Led_Pin_ON();
    Timer_Delay_Ticks(25000000);
    Led_Pin_OFF();
}

uint8_t DS18B20_CRC8(const uint8_t *data, uint8_t len){

    uint8_t crc = 0;

    for(int i=0;i<len;i++){
        crc ^=  data[i];
        for(int j=0;j<8;j++){
            if((crc & 0x01) == 0){
                crc >>=1;
            }
            else if((crc & 0x01) == 1){
                crc >>=1; crc ^= 0x8C;
            }
        }
    }

    return crc;

}




void UART0_RX_IRQHandler(void) {
  Led_Pin_Toggle();
  uint8_t byte; byte = UART0->DR & 0xFF;
  UART0->DR = byte;
  UART0->ICR_bit.RXIC = 1;
  byte = 0;
}




static void Timer_Init(void){
    RCU->PCLKCFG_bit.TMR0EN = 1;
    RCU->PRSTCFG_bit.TMR0EN = 1;
    TMR0->CTRL_bit.INTEN= 1;
    NVIC_SetPriority(TMR0_IRQn, 0);
    NVIC_EnableIRQ(TMR0_IRQn);
    TMR0->CTRL_bit.ON = 0 ;
}

void TMR0_IRQHandler(void){
    TMR0->INTSTATUS = 1;
    TMR0->LOAD = 0;
    timerDone = 1;
    TMR0 -> CTRL_bit.ON = 0;
}
void Timer_Delay_Ticks(uint32_t microseconds){
    TMR0->VALUE = 0;
    TMR0 ->LOAD = microseconds;
    TMR0->CTRL_bit.ON = 1;
    while (!timerDone){};
    timerDone = 0;
}


static void USB_UART_Init(void){
    RCU->UARTCFG[1].UARTCFG_bit.DIVEN = 0;
    RCU->UARTCFG[1].UARTCFG_bit.CLKSEL = RCU_UARTCFG_UARTCFG_CLKSEL_PLLCLK;
    RCU->UARTCFG[1].UARTCFG_bit.RSTDIS = 1;
    RCU->UARTCFG[1].UARTCFG_bit.CLKEN = 1;
    huart1.Instance = UART1;
    huart1.BaudRate = 115200U;
    huart1.WLEN = UART_LCRH_WLEN_8bit;
    huart1.STOPBIT = 0U;
    huart1.RXENABLE = 1U;
    huart1.TXENABLE = 1U;
    UART1->IMSC_bit.RXIM = 1;
    NVIC_SetPriority(UART1_RX_IRQn, 0);
    NVIC_EnableIRQ(UART1_RX_IRQn);
    User_UartInit(&huart1);
    USB_UART_SendMessage("K1921VK035 microcontroller - UART1 READY\n", sizeof("K1921VK035 microcontroller - UART1 READY"));
}

void UART1_RX_IRQHandler(void) {
    uint8_t byte = UART1->DR & 0xFF;
    if (packet_index < MAX_PACKET_SIZE) {
        packet_buffer[packet_index] = byte;
        packet_index++;
        if (byte == '\n') {
            Led_Pin_Toggle();
            packet_received =1;
            packet_index = 0;
        }
    } else {
        packet_index = 0;
    }
    UART1->ICR_bit.RXIC = 1;

}





void USB_UART_SendByte(uint8_t *byte){
    uint32_t timeout = 1000000;
    while ((UART1->FR_bit.BUSY == 1) && timeout--) {};
    if (timeout == 0) Error_Handler(); 
    UART1->DR = *byte;
}

void USB_UART_SendMessage(uint8_t *msg, uint16_t length){
    for (int i = 0; i < length; i++) {
        USB_UART_SendByte(&(msg[i]));
        while(UART1->FR_bit.BUSY == 1){};
    }
}

void Error_Handler(void){
    while(1){
        Led_Pin_Toggle();
        Timer_Delay_Ticks(10000000);
    }
}
// C:\Users\a.faizullin\Desktop\PrimeriVoronezh\K1921BK035\niietcm4-k1921vkx_sdk-45893dd9ab5d\projects\NIIET-BB-K1921VK035\blinky\app\main.c

void User_UartInit(UART_AbstractionTypeDef *huart){
    uint32_t integerDivider = 100000000U / (16 * huart->BaudRate);
    huart->Instance->IBRD = integerDivider;
    huart->Instance->FBRD = (uint32_t)((100000000U/ (16.0f * huart ->BaudRate)-integerDivider) * 64.0f + 0.5);
    huart->Instance->LCRH_bit.WLEN = huart->WLEN;
    huart->Instance->CR_bit.RXE = huart->RXENABLE;
    huart->Instance->CR_bit.TXE = huart->TXENABLE;
    huart->Instance->CR_bit.UARTEN = 1;
}

static void GPIO_Init(void){
    RCU->HCLKCFG_bit.GPIOBEN = 1;
    RCU->HRSTCFG_bit.GPIOBEN = 1;
    GPIOB->ALTFUNCSET_bit.PIN8 = 1; // TRANSMIT
    GPIOB->OUTMODE_bit.PIN8 = GPIO_OUTMODE_PIN8_PP;
    GPIOB->DENSET_bit.PIN8 = 1;
    GPIOB->ALTFUNCSET_bit.PIN9 = 1; // RECEIVE
    GPIOB->OUTMODE_bit.PIN9 = GPIO_OUTMODE_PIN9_PP;
    GPIOB->DENSET_bit.PIN9 = 1;

    //DS18B20 SIGNAL
    GPIOB->DENSET_bit.PIN15 = 1;
    GPIOB->OUTMODE_bit.PIN15 = 1;
    GPIOB->OUTENSET_bit.PIN15 = 1;
    GPIOB->DATAOUTSET_bit.PIN15 = 1; // OFF OPEN DRAIN

}

STATE HandleEventDS18B20(STATE curState, EVENT event){
    switch (curState)
    {
    case IDLE:
        if(event == START_MEASURING){
            return  MEASURING;
        }
        break;
    case MEASURING:
         if(DS18B20_Reset() == SENSOR_ERROR){
            return ERROR;
         };
         DS18B20_WriteByte(0xCC);
         DS18B20_WriteByte(0x44);
         Timer_Delay_Ticks(750 * 100 * 1000); // 750 ms
         if(DS18B20_Reset() == ERROR){
            return ERROR;
         };
         DS18B20_WriteByte(0xCC);
         DS18B20_WriteByte(0xBE);
         event = MEASURING_DONE;
         return READING;
    case READING:
        for (uint8_t i = 0; i < 9; i++) {
            scratchpad[i] = DS18B20_ReadByte();
        }
        Timer_Delay_Ticks(25000000);
        Led_Pin_ON();
        Timer_Delay_Ticks(25000000);
        Led_Pin_OFF();
        Timer_Delay_Ticks(25000000);
        Led_Pin_ON();
        Timer_Delay_Ticks(25000000);
        Led_Pin_OFF();
        event = READ_DONE;
        return IDLE;
      case ERROR:
        event = START_MEASURING;
        return MEASURING;
    default:
        break;
    }
}
