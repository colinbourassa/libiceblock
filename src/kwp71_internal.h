#ifndef KWP71_INTERNAL_H
#define KWP71_INTERNAL_H

uint16_t swapshort(const uint16_t source);
int msleep(long msec);

bool kwp71_expectedResponseTitle(uint8_t reqTitle, uint8_t* respTitle);
bool kwp71_5baudInit();
bool kwp71_openSerial(kwp71_info* info, const char* dev);
int16_t kwp71_readSerial(kwp71_info* info, uint8_t* buffer, uint16_t quantity);
int16_t kwp71_writeSerial(kwp71_info* info, const uint8_t* const buffer, uint16_t quantity);
bool kwp71_readIntroduction(kwp71_info* info, kwp71_protocol_variant variant, kwp71_ecuinfo* ecuinfo);

#endif // KWP71_INTERNAL_H

