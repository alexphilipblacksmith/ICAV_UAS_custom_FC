#include "crsf.h"

#define rcChannelsPayloadSize 22    // 16 channels x 11 bits
#define linkStatisticsPayloadSize 10


void crsfParserInit(crsfParser *parser){

    for(int i = 0; i < CRSF_NUM_CHANNELS; i++){
        parser->channels[i] = CRSF_CHANNEL_CENTRE;
    }
    parser->linkStats = (crsfLinkStatistics){0};
    parser->rcFrames = 0;
    parser->linkStatsFrames = 0;
    parser->crcErrors = 0;
    parser->lengthErrors = 0;
    parser->state = CRSF_WAIT_ADDRESS;
    parser->length = 0;
    parser->index = 0;
}


uint8_t crsfCrc8(const uint8_t *data, uint8_t length){

    uint8_t crc = 0;
    for(uint8_t i = 0; i < length; i++){
        crc ^= data[i];
        for(int bit = 0; bit < 8; bit++){
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}


//16 channels x 11 bits, packed LSB first
static void unpackChannels(const uint8_t *payload, uint16_t channels[CRSF_NUM_CHANNELS]){

    for(int ch = 0; ch < CRSF_NUM_CHANNELS; ch++){
        int bitIndex = ch * 11;
        int byteIndex = bitIndex / 8;
        int shift = bitIndex % 8;

        //A channel spans up to 3 bytes; don't read past the end
        uint32_t bits = (uint32_t)payload[byteIndex] | ((uint32_t)payload[byteIndex + 1] << 8);
        if(byteIndex + 2 < rcChannelsPayloadSize){
            bits |= (uint32_t)payload[byteIndex + 2] << 16;
        }

        channels[ch] = (uint16_t)((bits >> shift) & 0x7FF);
    }
}


static void decodeLinkStatistics(const uint8_t *payload, crsfLinkStatistics *stats){

    stats->uplinkRssi1 = payload[0];
    stats->uplinkRssi2 = payload[1];
    stats->uplinkLinkQuality = payload[2];
    stats->uplinkSnr = (int8_t)payload[3];
    stats->activeAntenna = payload[4];
    stats->rfMode = payload[5];
    stats->uplinkTxPower = payload[6];
    stats->downlinkRssi = payload[7];
    stats->downlinkLinkQuality = payload[8];
    stats->downlinkSnr = (int8_t)payload[9];
}


//Checks and decodes a complete frame. Returns its type, or 0 if it was bad
static uint8_t processFrame(crsfParser *parser){

    uint8_t length = parser->length;
    uint8_t type = parser->frame[0];
    const uint8_t *payload = &parser->frame[1];
    uint8_t payloadSize = length - 2;  //minus type and crc

    if(crsfCrc8(parser->frame, length - 1) != parser->frame[length - 1]){
        parser->crcErrors++;
        return 0;
    }

    switch(type){
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            if(payloadSize != rcChannelsPayloadSize){
                parser->lengthErrors++;
                return 0;
            }
            unpackChannels(payload, parser->channels);
            parser->rcFrames++;
            break;

        case CRSF_FRAMETYPE_LINK_STATISTICS:
            if(payloadSize != linkStatisticsPayloadSize){
                parser->lengthErrors++;
                return 0;
            }
            decodeLinkStatistics(payload, &parser->linkStats);
            parser->linkStatsFrames++;
            break;

        default:
            //good frame, just not one we use
            break;
    }

    return type;
}


uint8_t crsfParseByte(crsfParser *parser, uint8_t byte){

    switch(parser->state){

        case CRSF_WAIT_ADDRESS:
            if(byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_SYNC_BYTE){
                parser->state = CRSF_WAIT_LENGTH;
            }
            return 0;

        case CRSF_WAIT_LENGTH:
            //smallest frame is type + crc
            if(byte < 2 || byte > CRSF_MAX_FRAME_LENGTH){
                parser->lengthErrors++;
                //this byte might be the start of the next frame
                parser->state = (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER || byte == CRSF_SYNC_BYTE) ? CRSF_WAIT_LENGTH : CRSF_WAIT_ADDRESS;
                return 0;
            }
            parser->length = byte;
            parser->index = 0;
            parser->state = CRSF_READ_FRAME;
            return 0;

        case CRSF_READ_FRAME:
            parser->frame[parser->index++] = byte;
            if(parser->index < parser->length){
                return 0;
            }
            parser->state = CRSF_WAIT_ADDRESS;
            return processFrame(parser);
    }

    parser->state = CRSF_WAIT_ADDRESS;
    return 0;
}


uint16_t crsfToMicroseconds(uint16_t value){
    //1500 + (value - 992) * 5/8
    return (uint16_t)(1500 + (((int32_t)value - CRSF_CHANNEL_CENTRE) * 5) / 8);
}


static float clip(float value, float min, float max){
    if(value < min) return min;
    if(value > max) return max;
    return value;
}


float crsfToStick(uint16_t value, float deadband){

    float stick;
    if(value >= CRSF_CHANNEL_CENTRE){
        stick = (float)(value - CRSF_CHANNEL_CENTRE) / (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_CENTRE);
    }else{
        stick = -(float)(CRSF_CHANNEL_CENTRE - value) / (float)(CRSF_CHANNEL_CENTRE - CRSF_CHANNEL_MIN);
    }
    stick = clip(stick, -1.0f, 1.0f);

    //Ignore small moves around the centre, and rescale so full stick still gives +-1
    if(deadband <= 0.0f){
        return stick;
    }
    if(stick > -deadband && stick < deadband){
        return 0.0f;
    }
    if(stick > 0.0f){
        return (stick - deadband) / (1.0f - deadband);
    }
    return (stick + deadband) / (1.0f - deadband);
}


float crsfToThrottle(uint16_t value){
    float throttle = ((float)value - CRSF_CHANNEL_MIN) / (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
    return clip(throttle, 0.0f, 1.0f);
}
