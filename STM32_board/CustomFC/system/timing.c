#include "timing.h"

static uint32_t cyclesPerMicro = 168;


void timingInit(void){
    cyclesPerMicro = SystemCoreClock / 1000000u;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


uint32_t micros(void){
    return DWT->CYCCNT / cyclesPerMicro;
}


void watchdogStart(uint16_t timeoutMs){

    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;   // don't reset while paused in the debugger

    // LSI is ~32 kHz, /32 gives ~1 ms per count
    IWDG->KR = 0xCCCC;          // start
    IWDG->KR = 0x5555;          // unlock PR and RLR
    IWDG->PR = 3;               // /32
    IWDG->RLR = (timeoutMs > 4095) ? 4095 : timeoutMs;
    while(IWDG->SR != 0){}      // wait for the settings to land
    IWDG->KR = 0xAAAA;          // reload
}


void watchdogKick(void){
    IWDG->KR = 0xAAAA;
}
