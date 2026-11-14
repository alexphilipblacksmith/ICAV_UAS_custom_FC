#include "debugUart.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// USART1_TX = DMA2 Stream 7, Channel 4. Set up by hand so no interrupt is needed:
// the stream switches itself off when the transfer is done, which is how we know it's idle.
#define txStream DMA2_Stream7
#define txBufferSize 256

static uint8_t txBuffer[txBufferSize];


void debugUartInit(void){
    __HAL_RCC_DMA2_CLK_ENABLE();
    txStream->CR = 0;
    while(txStream->CR & DMA_SxCR_EN){}
    txStream->PAR = (uint32_t)&USART1->DR;
    txStream->FCR = 0;                      // direct mode
    USART1->CR3 |= USART_CR3_DMAT;
}


bool debugUartIdle(void){
    return (txStream->CR & DMA_SxCR_EN) == 0;
}


int debugUartWrite(const char *data, int len){

    int sent = 0;

    while(sent < len){
        uint32_t start = HAL_GetTick();
        while(!debugUartIdle()){
            if(HAL_GetTick() - start > 50) return sent;   // give up rather than hang
        }

        int chunk = len - sent;
        if(chunk > txBufferSize) chunk = txBufferSize;
        memcpy(txBuffer, data + sent, (size_t)chunk);

        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
        txStream->M0AR = (uint32_t)txBuffer;
        txStream->NDTR = (uint32_t)chunk;
        txStream->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;   // channel 4, memory -> UART
        USART1->SR &= ~USART_SR_TC;
        txStream->CR |= DMA_SxCR_EN;

        sent += chunk;
    }

    return sent;
}
