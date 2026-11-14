#ifndef CRSF_H
#define CRSF_H

/* CRSF parser for the ELRS receiver. No HAL in here so it can be tested on a PC.
 * Frame: [address 0xC8][length][type][payload...][crc8]
 * length counts type + payload + crc. crc8 uses polynomial 0xD5 over type + payload. */

#include <stdint.h>
#include <stdbool.h>

#define CRSF_NUM_CHANNELS 16

// EdgeTX -100% / 0% / +100% = 172 / 992 / 1811 (= 988 / 1500 / 2012 us)
#define CRSF_CHANNEL_MIN    172
#define CRSF_CHANNEL_CENTRE 992
#define CRSF_CHANNEL_MAX    1811

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_SYNC_BYTE                 0xEE   // some receivers start frames with this instead

#define CRSF_FRAMETYPE_LINK_STATISTICS     0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED  0x16

#define CRSF_MAX_FRAME_LENGTH 62

typedef struct{
    uint8_t uplinkRssi1;        // [-dBm], so 70 means -70 dBm
    uint8_t uplinkRssi2;
    uint8_t uplinkLinkQuality;  // [%]
    int8_t  uplinkSnr;          // [dB]
    uint8_t activeAntenna;
    uint8_t rfMode;
    uint8_t uplinkTxPower;
    uint8_t downlinkRssi;
    uint8_t downlinkLinkQuality;
    int8_t  downlinkSnr;
} crsfLinkStatistics;

typedef enum{
    CRSF_WAIT_ADDRESS,
    CRSF_WAIT_LENGTH,
    CRSF_READ_FRAME,
} crsfParserState;

typedef struct{
    uint16_t channels[CRSF_NUM_CHANNELS];   // raw 11-bit values
    crsfLinkStatistics linkStats;

    uint32_t rcFrames;
    uint32_t linkStatsFrames;
    uint32_t crcErrors;
    uint32_t lengthErrors;

    crsfParserState state;
    uint8_t length;
    uint8_t index;
    uint8_t frame[CRSF_MAX_FRAME_LENGTH];
} crsfParser;

void crsfParserInit(crsfParser *parser);

// Feed one byte. Returns the frame type when a good frame has just finished, otherwise 0.
uint8_t crsfParseByte(crsfParser *parser, uint8_t byte);

uint8_t crsfCrc8(const uint8_t *data, uint8_t length);

uint16_t crsfToMicroseconds(uint16_t value);
float crsfToStick(uint16_t value, float deadband);   // -1..1 with a centre deadband (like the sim's radioInput.py)
float crsfToThrottle(uint16_t value);                // 0..1

#endif // CRSF_H
