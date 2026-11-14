#include "rcInput.h"

// ~24 ms of data at full speed (500 Hz ELRS is ~15 kB/s)
#define rxBufferSize 1024

// Only trust the receiver's LQ value if link stats arrived this recently [ms]
#define linkStatsTimeoutMs 1000

static UART_HandleTypeDef *rcUart = NULL;
static DMA_HandleTypeDef hdmaRcRx;
static uint8_t rxBuffer[rxBufferSize];
static uint16_t rxTail = 0;     // next byte to parse

static crsfParser parser;
static bool rcFrameReceived = false;
static uint32_t lastRcFrameTick = 0;
static bool linkStatsReceived = false;
static uint32_t lastLinkStatsTick = 0;


// USART2_RX = DMA1 Stream 5, Channel 4
static void startDma(void){

    __HAL_RCC_DMA1_CLK_ENABLE();

    hdmaRcRx.Instance = DMA1_Stream5;
    hdmaRcRx.Init.Channel = DMA_CHANNEL_4;
    hdmaRcRx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdmaRcRx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdmaRcRx.Init.MemInc = DMA_MINC_ENABLE;
    hdmaRcRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaRcRx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdmaRcRx.Init.Mode = DMA_CIRCULAR;
    hdmaRcRx.Init.Priority = DMA_PRIORITY_HIGH;
    hdmaRcRx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdmaRcRx);
    __HAL_LINKDMA(rcUart, hdmarx, hdmaRcRx);

    rxTail = 0;
    // The DMA interrupts this turns on are never serviced (NVIC off), which is fine in circular mode
    HAL_UART_Receive_DMA(rcUart, rxBuffer, rxBufferSize);
}


void rcInputInit(UART_HandleTypeDef *huart){

    rcUart = huart;
    crsfParserInit(&parser);
    rcFrameReceived = false;
    linkStatsReceived = false;
    startDma();
}


void rcInputUpdate(void){

    if(rcUart == NULL) return;

    // Restart if the DMA got stopped (e.g. a transfer error)
    if((hdmaRcRx.Instance->CR & DMA_SxCR_EN) == 0){
        HAL_UART_AbortReceive(rcUart);
        HAL_DMA_DeInit(&hdmaRcRx);
        startDma();
        return;
    }

    // The DMA counter counts down, so the write position is size - counter
    uint16_t rxHead = (uint16_t)(rxBufferSize - __HAL_DMA_GET_COUNTER(&hdmaRcRx));
    if(rxHead >= rxBufferSize) rxHead = 0;

    uint32_t now = HAL_GetTick();
    while(rxTail != rxHead){
        uint8_t type = crsfParseByte(&parser, rxBuffer[rxTail]);
        rxTail = (uint16_t)((rxTail + 1) % rxBufferSize);

        if(type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
            rcFrameReceived = true;
            lastRcFrameTick = now;
        }else if(type == CRSF_FRAMETYPE_LINK_STATISTICS){
            linkStatsReceived = true;
            lastLinkStatsTick = now;
        }
    }
}


bool rcInputLinkUp(void){

    uint32_t now = HAL_GetTick();

    if(!rcFrameReceived || (now - lastRcFrameTick) > RC_FRAME_TIMEOUT_MS){
        return false;
    }

    // In case the receiver keeps sending old sticks after losing the link, it still reports 0% LQ
    if(linkStatsReceived && (now - lastLinkStatsTick) <= linkStatsTimeoutMs
       && parser.linkStats.uplinkLinkQuality == 0){
        return false;
    }

    return true;
}


static bool switchOn(uint8_t channel){
    return crsfToMicroseconds(parser.channels[channel]) > RC_SWITCH_ON_US;
}


bool rcInputGetCommand(radioCommand *cmd){

    if(!rcInputLinkUp()){
        cmd->roll = 0.0f;
        cmd->pitch = 0.0f;
        cmd->yaw = 0.0f;
        cmd->throttle = 0.0f;
        cmd->armSwitch = false;
        return false;
    }

    cmd->roll = RC_SIGN_ROLL * crsfToStick(parser.channels[RC_CHANNEL_ROLL], RC_STICK_DEADBAND);
    cmd->pitch = RC_SIGN_PITCH * crsfToStick(parser.channels[RC_CHANNEL_PITCH], RC_STICK_DEADBAND);
    cmd->yaw = RC_SIGN_YAW * crsfToStick(parser.channels[RC_CHANNEL_YAW], RC_STICK_DEADBAND);

    float throttle = crsfToThrottle(parser.channels[RC_CHANNEL_THROTTLE]);
    cmd->throttle = (RC_SIGN_THROTTLE < 0.0f) ? 1.0f - throttle : throttle;

    cmd->armSwitch = switchOn(RC_CHANNEL_ARM);

    return true;
}


uint16_t rcInputGetChannel(uint8_t channel){
    if(channel >= CRSF_NUM_CHANNELS) return CRSF_CHANNEL_CENTRE;
    return parser.channels[channel];
}


const crsfParser *rcInputGetParser(void){
    return &parser;
}
