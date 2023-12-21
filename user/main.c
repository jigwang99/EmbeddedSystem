#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"

void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void Reset_Pump(void);

static int state = 0;
static int ratio = 0;

void RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}


void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_TX_InitStructure, GPIO_RX_InitStructure;
    GPIO_InitTypeDef GPIOE_Out_InitStructure, GPIOE_In_InitStructure;

    // USART1 Tx(PA9), USART2 Tx(PA2)
    GPIO_TX_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_9;  
    GPIO_TX_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_TX_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_TX_InitStructure);

    // USART1 Rx(PA10), USART2 Rx(PA3)
    GPIO_RX_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10; 
    GPIO_RX_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
    GPIO_Init(GPIOA, &GPIO_RX_InitStructure);

    // RED LED | YELLOW LED | GREEN LED | PUMP 1 | PUMP 2
    GPIOE_Out_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11; 
    GPIOE_Out_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOE_Out_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIOE_Out_InitStructure);

    // Liquid level Sensor
    GPIOE_In_InitStructure.GPIO_Pin = GPIO_Pin_12; 
    GPIOE_In_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOE, &GPIOE_In_InitStructure);
}

// PC -> Board
void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;
    USART_Cmd(USART1, ENABLE);
    
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;

    USART_Init(USART1, &USART1_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

// Board -> Bluetooth
void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;
    USART_Cmd(USART2, ENABLE);

    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Init(USART2, &USART2_InitStructure);
}

void NVIC_Configure(void) 
{
    NVIC_InitTypeDef NVIC_USART1_InitStructure, NVIC_USART2_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_USART1_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_USART1_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_USART1_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_USART1_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_USART1_InitStructure);

    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_USART2_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_USART2_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_USART2_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_USART2_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_USART2_InitStructure);
}

// Putty???? Bluetooth??? ????
void USART1_IRQHandler(void) 
{
    uint16_t word;
    if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET){
        word = USART_ReceiveData(USART1);
        USART_SendData(USART2, word);
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

// Bluetooth?? ???? ????
void USART2_IRQHandler(void) 
{
    uint16_t word;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        word = USART_ReceiveData(USART2);
        if (state <= 1 && word >= '1' && word <= '5') {   // ???????? ??? ?? 
            ratio = word - '0';
            state = 2;
        }
        else if( state == 3 && word == '9'){
          state = 4;
        }
        USART_SendData(USART1, word);
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void delay(void) 
{
    for (int i = 0; i < 1000000; i++) {}
}

void Reset_Pump(void) 
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_10);
    GPIO_ResetBits(GPIOE, GPIO_Pin_11);
}

void Reset_LED(void) 
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_7);
    GPIO_ResetBits(GPIOE, GPIO_Pin_8);
    GPIO_ResetBits(GPIOE, GPIO_Pin_9);
}


int main(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();
    USART2_Init();
    NVIC_Configure();
    volatile uint32_t cnt_1;
    volatile uint32_t cnt_2;
    while (1) { 
        if (state == 0) {   // ??????
            Reset_Pump();
            GPIO_SetBits(GPIOE, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
            state = 1;
        }
        else if (state == 1) {   // Bluetooth ??? ??????
          
        }
        else if (state == 2) {    // Bluetooth ??? ??? ????
            Reset_LED();
            GPIO_SetBits(GPIOE, GPIO_Pin_7);
            state = 3;
        }
        else if (state == 3) {     // ?¬Ù¨ù??? ??? ??? ???? 

        }
        else if (state == 4) {       // ?¬Ù¨ù??? ??? ?? ????1 ???
        Reset_LED();
        GPIO_SetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_10);
        cnt_1 = 0;
        cnt_2 = 0;
        state = 5;
        }
        else if (state == 5) {      // ???????? ???? ??? ????
            cnt_1++;
            delay();
            if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)) {  // ???????? ???? ?? ????2 ???
                Reset_Pump();
                Reset_LED();
                GPIO_SetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_11);
                state = 6;
                }
            }
         else if (state == 6) {    // ????2 ??? ????
             cnt_2++;
             delay();
             if (cnt_2 * (10 - ratio) > cnt_1 * ratio) {  // ??? ??????? ??? ?? ????
                    Reset_Pump();
                    ratio = -1;
                    state = 0;
                }
            } 
            else {}
    }
}

