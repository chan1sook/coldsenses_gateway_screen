#ifndef __COLDSENSES_BLE__
#define __COLDSENSES_BLE__

#include <Arduino.h>
#include "coldsenses_conf.h"

#include <NimBLEDevice.h>

std::string prettyMacAddress(std::string rawMacAddress);

typedef struct
{
    uint32_t ts;
    std::string name;
    std::string rawMacAddress;
    double tempC;
    double humidRH;
    uint16_t battMv;
    uint8_t battPercent;
    uint8_t counter;
    uint8_t flag;
} MiTagData;

class MiTagScanner
{
private:
    BLEScan *_pBLEScan;
    MiTagData _tags[MAX_TAGS_REMEMBER];
    int _tagsCount = 0;

    void _addMiTagData(MiTagData &tagData);
    void _clearMiTagData();
    void _parseRawDataTo(std::string &rawData, MiTagData &to);
#if COLDSENSES_DEBUG_BLE
    std::string _prettyRawData(std::string &rawData);
    void _debugBLEData(std::string &rawData);
#endif

public:
    static NimBLEUUID TARGET_UUID;

    void init();
    void scan();
    int getTagsCount();
    int getActiveTagCount();
    MiTagData *getTagDataAt(int i);
    bool isTagActive(MiTagData *tagData);
    bool isMiTagDataValid(std::string &rawData);
};

#endif