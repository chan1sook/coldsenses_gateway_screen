#include "BLE.h"

std::string prettyMacAddress(std::string rawMacAddress)
{
  String buffer = "";
  for (int i = 0; i < 6; i++)
  {
    if (i > 0)
    {
      buffer += ":";
    }
    uint8_t b = rawMacAddress[i];
    if (b < 0x10)
    {
      buffer += '0';
    }
    buffer += String(b, 16);
  }
  return buffer.c_str();
}

NimBLEUUID MiTagScanner::TARGET_UUID = NimBLEUUID("181a");

void MiTagScanner::init()
{
  BLEDevice::init("");
  this->_pBLEScan = BLEDevice::getScan();
  this->_pBLEScan->setActiveScan(true);
  this->_pBLEScan->setInterval(100);
  this->_pBLEScan->setWindow(99);
}

void MiTagScanner::scan()
{
  BLEScanResults foundDevices = this->_pBLEScan->start(5, false);
  int nDevice = foundDevices.getCount();
  for (int i = 0; i < nDevice; i++)
  {
    NimBLEAdvertisedDevice device = foundDevices.getDevice(i);

    std::string rawData = device.getServiceData(TARGET_UUID);
#if COLDSENSES_DEBUG_BLE
    _debugBLEData(rawData);
#endif

    if (this->isMiTagDataValid(rawData))
    {
      MiTagData data;
      data.name = device.getName();
      data.ts = millis();
      _parseRawDataTo(rawData, data);
      this->_addMiTagData(data);
    }
  }

  Serial.print("Devices found: ");
  Serial.println(nDevice);
  Serial.print("Tags found: ");
  Serial.println(this->_tagsCount);
  Serial.println("Scan done!");

  this->_pBLEScan->clearResults();
}

int MiTagScanner::getTagsCount()
{
  return this->_tagsCount;
}

int MiTagScanner::getActiveTagCount()
{
  int onlines = 0;
  for (int i = 0; i < this->_tagsCount; i++)
  {
    if (isTagActive(getTagDataAt(i)))
    {
      onlines += 1;
    }
  }

  return onlines;
}

MiTagData *MiTagScanner::getTagDataAt(int i)
{
  if (i < 0 || i >= this->_tagsCount)
  {
    return NULL;
  }
  return &(this->_tags[i]);
}

bool MiTagScanner::isTagActive(MiTagData *tagData)
{
  return tagData && millis() - (*tagData).ts <= TAG_ONLINE_TIEMOUT;
}

bool MiTagScanner::isMiTagDataValid(std::string &rawData)
{
  return rawData.length() == 15;
}

void MiTagScanner::_addMiTagData(MiTagData &tagData)
{
  for (int i = 0; i < this->_tagsCount; i++)
  {
    if (tagData.rawMacAddress == this->_tags[i].rawMacAddress)
    {
      this->_tags[i] = tagData;
      return;
    }
  }

  if (this->_tagsCount < MAX_TAGS_REMEMBER)
  {
    this->_tags[this->_tagsCount] = tagData;
    this->_tagsCount += 1;
    return;
  }

  int targetIndex = -1;
  uint32_t tsPrev = 0;
  for (int i = 0; i < this->_tagsCount; i++)
  {
    if (this->_tags[i].ts > tsPrev)
    {
      tsPrev = this->_tags[i].ts;
      targetIndex = i;
    }
  }

  if (targetIndex != -1)
  {
    this->_tags[targetIndex] = tagData;
  }
}

void MiTagScanner::_clearMiTagData()
{
  this->_tagsCount = 0;
}

void MiTagScanner::_parseRawDataTo(std::string &rawData, MiTagData &to)
{
  to.rawMacAddress = rawData.substr(0, 6);

  int tempRaw = rawData[6] + (rawData[7] << 8);
  to.tempC = tempRaw / 100.0;

  int humidRaw = rawData[8] + (rawData[9] << 8);
  to.humidRH = humidRaw / 100.0;

  to.battMv = rawData[10] + (rawData[11] << 8);
  to.battPercent = rawData[12];
  to.counter = rawData[13];
  to.flag = rawData[14];
}

#if COLDSENSES_DEBUG_BLE
std::string MiTagScanner::_prettyRawData(std::string &rawData)
{
  String buffer = "[";
  int len = rawData.length();
  for (int i = 0; i < len; i++)
  {
    if (i > 0)
    {
      buffer += ',';
    }
    buffer += (int)rawData[i];
  }
  buffer += ']';
  return buffer.c_str();
}

void MiTagScanner::_debugBLEData(std::string &rawData)
{
  int len = rawData.length();
  Serial.print("Data: ");
  Serial.println(_prettyRawData(rawData).c_str());
}
#endif