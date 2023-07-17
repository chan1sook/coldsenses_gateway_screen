#include <Arduino.h>
#include <Wire.h>

#include <SPI.h>
#include <TFT_eSPI.h>
#include "FT62XXTouchScreen.h"

#include <WiFi.h>
#include <EEPROM.h>
#include <MQTT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "BLE.h"
#include "ColdsensesEnums.h"

static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;

TFT_eSPI tft = TFT_eSPI();
FT62XXTouchScreen touchScreen = FT62XXTouchScreen(screenHeight, PIN_SDA, PIN_SCL);

#include "esp_freertos_hooks.h"
#include "ui/ui.h"

#define BUFFER_SIZE (screenWidth * screenHeight / 10)

static lv_disp_draw_buf_t disp_buf;
static lv_color_t *screenBuffer1;

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

static void lv_tick_task(void *arg);
static void lv_handler_task(void *arg);
static void update_screen_task(lv_timer_t *timer);
#if COLDSENSES_DEBUG_TICK
static void debug_tick_task(lv_timer_t *timer);
#endif
static void splash_to_home(lv_timer_t *timer);

esp_timer_handle_t ticker_timer;
esp_timer_handle_t handler_timer;
const esp_timer_create_args_t ticker_timer_args = {
    .callback = &lv_tick_task,
    .name = "lv_tick_task"};
const esp_timer_create_args_t handler_timer_args = {
    .callback = &lv_handler_task,
    .name = "lv_handler_task"};

MQTTClient mqttClient;
WiFiClient wifiClient;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

uint32_t blinkLastTs;
uint32_t wifiLastTs;

String deviceMAC;
String wifiSSID = "";
double gpsLatitude = 15.8700;
double gpsLongitude = 100.9925;
String wifiPassword = "";

String inputTemp;
String optionWifiSSID;
String optionWifiPassword;
double optionGpsLat;
double optionGpsLng;

String mqttClientName;
bool isMqttError;

MiTagScanner miTagScanner;
coldsenses_wifi_state wifiState = COLDSENSES_WL_WAITING;
coldsenses_wifi_state prevWifiState = COLDSENSES_WL_WAITING;
coldsenses_tag_state tagState = COLDSENSES_TAG_NOT_SCAN;
coldsenses_mqtt_state mqttState = COLDSENSES_MQTT_DISCONNECTED;
coldsenses_mqtt_state prevMqttState = COLDSENSES_MQTT_DISCONNECTED;

bool spinnerHide = false;
coldsenses_after_alarm_action afterAlarmAction = COLDSENSES_NO_ACTION;
coldsenses_input_target inputTarget = COLDSENSES_NO_TARGET;

ui_coldsenses_tag_holder uiTagHolders[MAX_TAGS_REMEMBER];
ui_coldsenses_option_holder uiWifiSSIDOptionHolder;
ui_coldsenses_option_holder uiWifiPasswordOptionHolder;
ui_coldsenses_option_holder uiGpsLatOptionHolder;
ui_coldsenses_option_holder uiGpsLngOptionHolder;
ui_coldsenses_option_holder uiCheckVersionHolder;

#if COLDSENSES_ALLOW_EEPROM
static void initEEPROM();
static void migrateEEPROMDataVersion();
static void updateEEPROM();
#if COLDSENSES_DEBUG_EEPROM
static void _displayOptionValues();
#endif
#endif

static void beginWifi(String &wifiSSID, String &wifiPassword);
static void beginMqtt();
static void emitMqtt(MiTagData &tagData);

bool isOptionDirty(coldsenses_input_target target);
bool isOptionsDirty();
bool isOptionValid(coldsenses_input_target target);
bool isOptionsValid();
void restoreSaveToOptions();
void applySaveOptions();
void restoreOptionToTemp(coldsenses_input_target target);
void applyValueToOption(String value, coldsenses_input_target target);

static void initUI();
static void instanceTagHolderAt(uint8_t i, ui_coldsenses_tag_holder &holder);
static void instanceOptionHolder(coldsenses_option option, lv_event_cb_t event, ui_coldsenses_option_holder &holder);

void lv_obj_toggle_display(lv_obj_t *obj, bool isShow);
void lv_obj_toggle_clickable(lv_obj_t *obj, bool clickable);

static void ui_event_wifi_ssid_option(lv_event_t *e);
static void ui_event_wifi_password_option(lv_event_t *e);
static void ui_event_gps_lat_option(lv_event_t *e);
static void ui_event_gps_lng_option(lv_event_t *e);
static void ui_event_check_version_option(lv_event_t *e);

void transitionToHomeScreen();
void transitionToOptionScreen();
void transitionToInputScreen();
void transitionToAlertScreen();
void changeAlarmScreen(String message, bool showOK, bool showCancel, bool showSpinner);
void changeInputValueTo(String &value, bool passwordMode, int maxLength);

static void updateStatusBar();
static void updateWifiStatusUI();
static void updateBLEStatusUI();
static void updateMqttStatusUI();
static void updateHomeScreen();
static void updateTagHolderData(MiTagData *tagData, int i);
static void updateOptionScreen();

void setup()
{
  Serial.begin(115200);

  Serial.print("COLDSENSES_GATEWAY V.");
  Serial.println(COLDSENSES_GATEWAY_VERSION);

#if COLDSENSES_ALLOW_EEPROM
#if COLDSENSES_DEBUG_EEPROM
  Serial.println("Read EEPROM");
#endif
  initEEPROM();
#endif

  deviceMAC = WiFi.macAddress();
  deviceMAC.replace(":", "");
  deviceMAC.toUpperCase();

  mqttClientName = "w32s01-gw-";
  mqttClientName += deviceMAC;

  miTagScanner.init();
  beginWifi(wifiSSID, wifiPassword);

  mqttClient.begin(COLDSENSES_MQTT_SERVER_URL, COLDSENSES_MQTT_SERVER_PORT, wifiClient);

  // Init LVGL
  lv_init();
  ESP_ERROR_CHECK(esp_timer_create(&ticker_timer_args, &ticker_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(ticker_timer, portTICK_RATE_MS * 1000));

  // Enable TFT
  tft.begin();
  tft.setRotation(1);

  // Enable Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 1);

  // Start TouchScreen
  touchScreen.begin();

  // Display Buffer
  screenBuffer1 = (lv_color_t *)ps_malloc(BUFFER_SIZE * sizeof(lv_color_t));
  lv_disp_draw_buf_init(&disp_buf, screenBuffer1, NULL, BUFFER_SIZE);

  // Initialize the display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.full_refresh = true;
  lv_disp_drv_register(&disp_drv);

  // Init Touchscreen
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);

  initUI();

  // Init LVGL Update Timer
  ESP_ERROR_CHECK(esp_timer_create(&handler_timer_args, &handler_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(handler_timer, 10 * portTICK_RATE_MS * 1000));

  lv_timer_create(update_screen_task, 100 * portTICK_RATE_MS, NULL);

#if COLDSENSES_DEBUG_TICK
  lv_timer_create(debug_tick_task, 1000 * portTICK_RATE_MS, NULL);
#endif

  lv_timer_t *splashTimer = lv_timer_create(splash_to_home, 3000, NULL);
  lv_timer_set_repeat_count(splashTimer, 1);
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
}

void loop()
{
  timeClient.update();
  mqttClient.loop();

  prevWifiState = wifiState;
  prevMqttState = mqttState;

  if (wifiState != COLDSENSES_WL_CONNECTED && wifiState != COLDSENSES_WL_WAITING && millis() - wifiLastTs > 30000)
  {
    beginWifi(wifiSSID, wifiPassword);
  }

  wl_status_t wifiRawStatus = WiFi.status();
  switch (wifiRawStatus)
  {
  case WL_IDLE_STATUS:
    wifiState = COLDSENSES_WL_WAITING;
    break;
  case WL_CONNECTED:
    wifiState = COLDSENSES_WL_CONNECTED;
    break;
  case WL_DISCONNECTED:
  case WL_CONNECTION_LOST:
    wifiState = COLDSENSES_WL_DISCONNECTED;
    break;
  case WL_CONNECT_FAILED:
  case WL_NO_SSID_AVAIL:
  case WL_NO_SHIELD:
    wifiState = COLDSENSES_WL_CONNECT_FAILED;
  }

  if (prevWifiState != wifiState && wifiState == COLDSENSES_WL_CONNECTED)
  {
    timeClient.begin();
  }

  if (wifiState == COLDSENSES_WL_CONNECTED && (isMqttError || prevWifiState != wifiState))
  {
    beginMqtt();
    isMqttError = false;
    mqttState = COLDSENSES_MQTT_WAITING;
  }

  lwmqtt_err_t mqttError = mqttClient.lastError();
  bool isMqttConnected = mqttClient.connected();
  if (!isMqttError && mqttError != LWMQTT_SUCCESS)
  {
    mqttState = COLDSENSES_MQTT_CONNECT_FAILED;
    isMqttError = true;
  }
  else if (prevMqttState == COLDSENSES_MQTT_WAITING && isMqttConnected)
  {
    mqttState = COLDSENSES_MQTT_CONNECTED;
  }
  else if (prevMqttState == COLDSENSES_MQTT_CONNECTED && !isMqttConnected)
  {
    mqttState = COLDSENSES_MQTT_DISCONNECTED;
  }

  miTagScanner.scan();
  tagState = COLDSENSES_TAG_SCANNED;

  int tagsCount = miTagScanner.getTagsCount();
  for (int i = 0; i < tagsCount; i++)
  {
    MiTagData *tagData = miTagScanner.getTagDataAt(i);
    if (tagData && miTagScanner.isTagActive(tagData))
    {
      emitMqtt(*tagData);
    }
  }

  delay(10);
}

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  TouchPoint touchPos = touchScreen.read();
  if (touchPos.touched)
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchPos.xPos;
    data->point.y = touchPos.yPos;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

static void lv_tick_task(void *arg)
{
  lv_tick_inc(portTICK_RATE_MS);
}

static void lv_handler_task(void *arg)
{
  lv_task_handler();
}

static void update_screen_task(lv_timer_t *timer)
{
  updateStatusBar();

  lv_obj_t *currentScreen = lv_scr_act();
  if (currentScreen == ui_HomeScreen)
  {
    updateHomeScreen();
    return;
  }

  if (currentScreen == ui_OptionScreen)
  {
    updateOptionScreen();
  }
}

#if COLDSENSES_DEBUG_TICK
static void debug_tick_task(lv_timer_t *timer)
{
  // For Debug only
}
#endif

static void splash_to_home(lv_timer_t *timer)
{
  blinkLastTs = millis();
  tagState = COLDSENSES_TAG_NOT_SCAN;
  spinnerHide = false;
  updateWifiStatusUI();
  updateBLEStatusUI();
  updateMqttStatusUI();
  transitionToHomeScreen();
}

#if COLDSENSES_ALLOW_EEPROM
static void initEEPROM()
{
  if (!EEPROM.begin(EEPROM_TOTAL_BYTES))
  {
    Serial.println("EEPROM init error");
    return;
  }

  uint64_t header = EEPROM.readULong64(EEPROM_HEADER_ADDR);

#if COLDSENSES_DEBUG_EEPROM
  Serial.print("EEPROM header: ");
  Serial.println(EEPROM_HEADER_KEY);
  Serial.print("Actual: ");
  Serial.println(header);
  Serial.print("Equal?: ");
  Serial.println(header == EEPROM_HEADER_KEY ? "T" : "F");
#endif

  if (header != EEPROM_HEADER_KEY)
  {
#if COLDSENSES_DEBUG_EEPROM
    Serial.println("Create New EEPROM Save");
#endif
    EEPROM.writeULong64(EEPROM_HEADER_ADDR, EEPROM_HEADER_KEY);
    EEPROM.writeUInt(EEPROM_VERSION_ADDR, COLDSENSES_GATEWAY_VERSION);
    EEPROM.writeString(EEPROM_SSID_ADDR, wifiSSID);
    EEPROM.writeString(EEPROM_WIFIPW_ADDR, wifiPassword);
    EEPROM.writeDouble(EEPROM_GPSLAT_ADDR, gpsLatitude);
    EEPROM.writeDouble(EEPROM_GPSLONG_ADDR, gpsLongitude);
    EEPROM.commit();
#if COLDSENSES_DEBUG_EEPROM
    Serial.println("EEPROM Saved");
#endif
  }

  migrateEEPROMDataVersion();

#if COLDSENSES_DEBUG_EEPROM
  Serial.println("Load EEPROM Save");
#endif

  // Read values
  wifiSSID = EEPROM.readString(EEPROM_SSID_ADDR);
  wifiPassword = EEPROM.readString(EEPROM_WIFIPW_ADDR);
  gpsLatitude = EEPROM.readDouble(EEPROM_GPSLAT_ADDR);
  gpsLongitude = EEPROM.readDouble(EEPROM_GPSLONG_ADDR);

#if COLDSENSES_DEBUG_EEPROM
  Serial.println("EEPROM Values");
  _displayOptionValues();
#endif
}

static void migrateEEPROMDataVersion()
{

#if COLDSENSES_DEBUG_EEPROM
  Serial.println("Check EEPROM Save Version");
#endif
  uint16_t version = EEPROM.readUInt(EEPROM_VERSION_ADDR);
  // TODO if read version < k do this => if(version < k) { ... }
  if (version < COLDSENSES_GATEWAY_VERSION)
  {
    EEPROM.writeUInt(EEPROM_VERSION_ADDR, COLDSENSES_GATEWAY_VERSION);
  }
#if COLDSENSES_DEBUG_EEPROM
  Serial.println("EEPROM Save Updated");
#endif
}

static void updateEEPROM()
{
#if COLDSENSES_DEBUG_EEPROM
  Serial.println("New EEPROM Values");
  _displayOptionValues();
#endif
  EEPROM.writeULong64(EEPROM_HEADER_ADDR, EEPROM_HEADER_KEY);
  EEPROM.writeUInt(EEPROM_VERSION_ADDR, COLDSENSES_GATEWAY_VERSION);
  EEPROM.writeString(EEPROM_SSID_ADDR, wifiSSID);
  EEPROM.writeString(EEPROM_WIFIPW_ADDR, wifiPassword);
  EEPROM.writeDouble(EEPROM_GPSLAT_ADDR, gpsLatitude);
  EEPROM.writeDouble(EEPROM_GPSLONG_ADDR, gpsLongitude);
  EEPROM.commit();

#if COLDSENSES_DEBUG_EEPROM
  bool isSuccess = EEPROM.commit();
  Serial.print("Update EEPROM: ");
  Serial.println(isSuccess ? "T" : "F");
#else
  EEPROM.commit();
#endif
}
#if COLDSENSES_DEBUG_EEPROM
static void _displayOptionValues()
{
  Serial.print("wifiSSID: ");
  Serial.println(wifiSSID);
  Serial.print("wifiPassword: ");
  Serial.println(wifiPassword);
  Serial.print("gpsLatitude: ");
  Serial.println(gpsLatitude);
  Serial.print("gpsLongitude: ");
  Serial.println(gpsLongitude);
}
#endif

#endif

static void beginWifi(String &wifiSSID, String &wifiPassword)
{
#if COLDSENSES_DEBUG_WIFI
  Serial.print("WIFI SSID: ");
  Serial.println(wifiSSID);
  Serial.print("WIFI PW: ");
  Serial.println(wifiPassword);
#endif
  wifiLastTs = millis();
  WiFi.begin(wifiSSID.c_str(), wifiPassword.length() > 0 ? wifiPassword.c_str() : NULL);
}

static void beginMqtt()
{
  mqttClient.connect(mqttClientName.c_str());
}

static void emitMqtt(MiTagData &tagData)
{
  String topic = "push_";
  String macAddress = prettyMacAddress(tagData.rawMacAddress).c_str();
  macAddress.replace(":", "");
  macAddress.toUpperCase();

  topic += macAddress;

  String sharedPayload = String();
  sharedPayload.concat(macAddress);
  sharedPayload.concat(":");
  sharedPayload.concat(mqttClientName);
  sharedPayload.concat(":");
  sharedPayload.concat(timeClient.getEpochTime());

  // HB:macAddress:gateway:gatewayTs:battLv:chargeState:connectivity
  String hbPayload = "HB:";
  hbPayload.concat(sharedPayload);
  hbPayload.concat(":100:-:-");

  // W:macAddress:gateway:gatewayTs:temp:humid:batt:rssi
  String wPayload = "W:";
  wPayload.concat(sharedPayload);
  wPayload.concat(":");
  if (isnan(tagData.tempC))
  {
    wPayload.concat("-");
  }
  else
  {
    wPayload.concat(tagData.tempC);
  }
  wPayload.concat(":");
  if (isnan(tagData.humidRH))
  {
    wPayload.concat("-");
  }
  else
  {
    wPayload.concat(tagData.humidRH);
  }
  wPayload.concat(":100:");
  wPayload.concat(WiFi.RSSI());

  // GEO:macAddress:gateway:gatewayTs:lat:lng
  String geoPayload = "GEO:";
  geoPayload.concat(sharedPayload);
  geoPayload.concat(":");
  geoPayload.concat(String(gpsLatitude, 8));
  geoPayload.concat(":");
  geoPayload.concat(String(gpsLongitude, 8));

  bool hbSentSuccess = mqttClient.publish(topic.c_str(), hbPayload.c_str());
  bool wSentSuccess = mqttClient.publish(topic.c_str(), wPayload.c_str());
  bool geoSentSuccess = mqttClient.publish(topic.c_str(), geoPayload.c_str());

#if COLDSENSES_DEBUG_MQTT >= 2
  Serial.print("HB:");
  Serial.println(hbPayload);
  Serial.print("W:");
  Serial.println(wPayload);
  Serial.print("GEO:");
  Serial.println(geoPayload);
#endif

#if COLDSENSES_DEBUG_MQTT
  Serial.print("OK? [W,HB,GEO]: [");
  Serial.print(wSentSuccess ? "T" : "F");
  Serial.print(",");
  Serial.print(hbSentSuccess ? "T" : "F");
  Serial.print(",");
  Serial.print(geoSentSuccess ? "T" : "F");
  Serial.println("]");
#endif
}

bool isOptionDirty(coldsenses_input_target target)
{
  switch (target)
  {
  case COLDSENSES_TARGET_WIFI_SSID:
    return !wifiSSID.equals(optionWifiSSID);
  case COLDSENSES_TARGET_WIFI_PASSWORD:
    return !wifiPassword.equals(optionWifiPassword);
  case COLDSENSES_TARGET_GPS_LATITUDE:
    return gpsLatitude != optionGpsLat;
  case COLDSENSES_TARGET_GPS_LONGITUDE:
    return gpsLongitude != optionGpsLng;
  case COLDSENSES_NO_TARGET:
  default:
    return false;
  }
}

bool isOptionsDirty()
{
  return isOptionDirty(COLDSENSES_TARGET_WIFI_SSID) || isOptionDirty(COLDSENSES_TARGET_WIFI_PASSWORD) ||
         isOptionDirty(COLDSENSES_TARGET_GPS_LATITUDE) || isOptionDirty(COLDSENSES_TARGET_GPS_LONGITUDE);
}

bool isOptionValid(coldsenses_input_target target)
{
  switch (target)
  {
  case COLDSENSES_TARGET_WIFI_SSID:
    return optionWifiSSID.length() > 0 && optionWifiSSID.length() <= SSID_MAXLENGTH;
  case COLDSENSES_TARGET_WIFI_PASSWORD:
    return optionWifiPassword.length() <= WIFIPW_MAXLENGTH;
  case COLDSENSES_TARGET_GPS_LATITUDE:
    return optionGpsLat >= -90 && optionGpsLat <= 90;
  case COLDSENSES_TARGET_GPS_LONGITUDE:
    return optionGpsLng >= -180 && optionGpsLng <= 180;
  case COLDSENSES_NO_TARGET:
  default:
    return false;
  }
}

bool isOptionsValid()
{
  return isOptionValid(COLDSENSES_TARGET_WIFI_SSID) && isOptionValid(COLDSENSES_TARGET_WIFI_PASSWORD) &&
         isOptionValid(COLDSENSES_TARGET_GPS_LATITUDE) && isOptionValid(COLDSENSES_TARGET_GPS_LONGITUDE);
}

void restoreSaveToOptions()
{
  optionWifiSSID = String(wifiSSID);
  optionWifiPassword = String(wifiPassword);
  optionGpsLat = gpsLatitude;
  optionGpsLng = gpsLongitude;
}

void applySaveOptions()
{

  wifiSSID = String(optionWifiSSID);
  wifiPassword = String(optionWifiPassword);
  gpsLatitude = optionGpsLat;
  gpsLongitude = optionGpsLng;

#if COLDSENSES_ALLOW_EEPROM
  updateEEPROM();
#endif

  // WiFi.disconnect();
  wifiState = COLDSENSES_WL_WAITING;
  beginWifi(wifiSSID, wifiPassword);
}

void restoreOptionToTemp(coldsenses_input_target target)
{
  switch (target)
  {
  case COLDSENSES_TARGET_WIFI_SSID:
    inputTemp = String(optionWifiSSID);
    break;
  case COLDSENSES_TARGET_WIFI_PASSWORD:
    inputTemp = String(optionWifiPassword);
    break;
  case COLDSENSES_TARGET_GPS_LATITUDE:
    inputTemp = String(optionGpsLat, 8);
    break;
  case COLDSENSES_TARGET_GPS_LONGITUDE:
    inputTemp = String(optionGpsLng, 8);
    break;
  default:
    break;
  }
}

void applyValueToOption(String value, coldsenses_input_target target)
{

  switch (target)
  {
  case COLDSENSES_TARGET_WIFI_SSID:
    optionWifiSSID = String(value);
    break;
  case COLDSENSES_TARGET_WIFI_PASSWORD:
    optionWifiPassword = String(value);
    break;
  case COLDSENSES_TARGET_GPS_LATITUDE:
    optionGpsLat = String(value).toDouble();
    break;
  case COLDSENSES_TARGET_GPS_LONGITUDE:
    optionGpsLng = String(value).toDouble();
    break;
  default:
    break;
  }
}

static void initUI()
{
  ui_init();
  SpinnerSpin_Animation(ui_TagSpinner, 0);
  lv_label_set_text(ui_TagCountLabel, "");

  for (int i = 0; i < MAX_TAGS_REMEMBER; i++)
  {
    instanceTagHolderAt(i, uiTagHolders[i]);
    lv_obj_toggle_display(uiTagHolders[i].tag_panel, false);
  }

  instanceOptionHolder(COLDSENSES_OPTION_SET_WIFI_SSID, ui_event_wifi_ssid_option, uiWifiSSIDOptionHolder);
  instanceOptionHolder(COLDSENSES_OPTION_SET_WIFI_PASSWORD, ui_event_wifi_password_option, uiWifiPasswordOptionHolder);
  instanceOptionHolder(COLDSENSES_OPTION_SET_GPS_LATITUDE, ui_event_gps_lat_option, uiGpsLatOptionHolder);
  instanceOptionHolder(COLDSENSES_OPTION_SET_GPS_LONGITUDE, ui_event_gps_lng_option, uiGpsLngOptionHolder);
  instanceOptionHolder(COLDSENSES_OPTION_CHECK_VERSION, ui_event_check_version_option, uiCheckVersionHolder);
  lv_obj_toggle_display(uiCheckVersionHolder.mark_edit_img, false);

  lv_obj_del(ui_TagPanelT);
  lv_obj_del(ui_OptionListPanelHolderT);
}

static void instanceTagHolderAt(uint8_t i, ui_coldsenses_tag_holder &holder)
{
  lv_obj_t *tagPanel = lv_obj_create(ui_TagsHolderPanel);
  lv_obj_set_width(tagPanel, 80);
  lv_obj_set_height(tagPanel, 80);
  int posX = (i >> 1) * 90;
  int posY = (i & 0x1) * 90;
  lv_obj_set_x(tagPanel, posX);
  lv_obj_set_y(tagPanel, posY);
  lv_obj_set_align(tagPanel, LV_ALIGN_TOP_LEFT);
  lv_obj_clear_flag(tagPanel, LV_OBJ_FLAG_SCROLLABLE); /// Flags
  lv_obj_set_style_blend_mode(tagPanel, LV_BLEND_MODE_NORMAL, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_opa(tagPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_left(tagPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(tagPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(tagPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(tagPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_set_style_pad_left(tagPanel, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(tagPanel, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(tagPanel, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(tagPanel, 0, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

  lv_obj_t *tagInnerPanel = lv_obj_create(tagPanel);
  lv_obj_set_width(tagInnerPanel, 65);
  lv_obj_set_height(tagInnerPanel, 55);
  lv_obj_set_align(tagInnerPanel, LV_ALIGN_CENTER);
  lv_obj_clear_flag(tagInnerPanel, LV_OBJ_FLAG_SCROLLABLE); /// Flags
  lv_obj_set_style_radius(tagInnerPanel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(tagInnerPanel, lv_color_hex(0xD3D3D3), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tagInnerPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_border_width(tagInnerPanel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_left(tagInnerPanel, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(tagInnerPanel, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(tagInnerPanel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(tagInnerPanel, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t *tagCRHLabel = lv_label_create(tagInnerPanel);
  lv_obj_set_width(tagCRHLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(tagCRHLabel, LV_SIZE_CONTENT); /// 1
  lv_obj_set_x(tagCRHLabel, 0);
  lv_obj_set_y(tagCRHLabel, 4);
  lv_obj_set_align(tagCRHLabel, LV_ALIGN_RIGHT_MID);
  lv_label_set_text(tagCRHLabel, "C\n%R");
  lv_obj_set_style_text_letter_space(tagCRHLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_line_space(tagCRHLabel, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(tagCRHLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(tagCRHLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t *tagNamelabel = lv_label_create(tagPanel);
  lv_obj_set_width(tagNamelabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(tagNamelabel, LV_SIZE_CONTENT); /// 1
  lv_obj_set_align(tagNamelabel, LV_ALIGN_TOP_MID);
  lv_label_set_text(tagNamelabel, "Name");
  lv_obj_set_style_text_font(tagNamelabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t *tagTempLabel = lv_label_create(tagInnerPanel);
  lv_obj_set_width(tagTempLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(tagTempLabel, LV_SIZE_CONTENT); /// 1
  lv_obj_set_x(tagTempLabel, -6);
  lv_obj_set_y(tagTempLabel, -9);
  lv_obj_set_align(tagTempLabel, LV_ALIGN_CENTER);
  lv_label_set_text(tagTempLabel, "00.0");
  lv_obj_set_style_text_letter_space(tagTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_line_space(tagTempLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(tagTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(tagTempLabel, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t *tagHumidLabel = lv_label_create(tagInnerPanel);
  lv_obj_set_width(tagHumidLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(tagHumidLabel, LV_SIZE_CONTENT); /// 1
  lv_obj_set_x(tagHumidLabel, -6);
  lv_obj_set_y(tagHumidLabel, 12);
  lv_obj_set_align(tagHumidLabel, LV_ALIGN_CENTER);
  lv_label_set_text(tagHumidLabel, "100");
  lv_obj_set_style_text_letter_space(tagHumidLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_line_space(tagHumidLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(tagHumidLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(tagHumidLabel, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t *tagMacLabel = lv_label_create(tagPanel);
  lv_obj_set_width(tagMacLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(tagMacLabel, LV_SIZE_CONTENT); /// 1
  lv_obj_set_align(tagMacLabel, LV_ALIGN_BOTTOM_MID);
  lv_label_set_text(tagMacLabel, "MAC");
  lv_obj_set_style_text_font(tagMacLabel, &lv_font_montserrat_8, LV_PART_MAIN | LV_STATE_DEFAULT);

  holder.temp_label = tagTempLabel;
  holder.humid_label = tagHumidLabel;
  holder.name_label = tagNamelabel;
  holder.mac_label = tagMacLabel;
  holder.tag_panel = tagPanel;
}

static void instanceOptionHolder(coldsenses_option option, lv_event_cb_t event, ui_coldsenses_option_holder &holder)
{
  lv_obj_t *optionListPanelHolder = lv_obj_create(ui_OptionPanelHolder);
  lv_obj_set_width(optionListPanelHolder, 430);
  lv_obj_set_height(optionListPanelHolder, 40);
  lv_obj_set_align(optionListPanelHolder, LV_ALIGN_CENTER);
  lv_obj_clear_flag(optionListPanelHolder, LV_OBJ_FLAG_SCROLLABLE); /// Flags
  lv_obj_set_style_bg_color(optionListPanelHolder, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(optionListPanelHolder, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_left(optionListPanelHolder, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(optionListPanelHolder, 6, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(optionListPanelHolder, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(optionListPanelHolder, 3, LV_PART_MAIN | LV_STATE_DEFAULT);

  String optionText = "";
  switch (option)
  {
  case COLDSENSES_OPTION_SET_WIFI_SSID:
    optionText = "WiFi SSID";
    break;
  case COLDSENSES_OPTION_SET_WIFI_PASSWORD:
    optionText = "WiFi Password";
    break;
  case COLDSENSES_OPTION_SET_GPS_LATITUDE:
    optionText = "GPS Latitude";
    break;
  case COLDSENSES_OPTION_SET_GPS_LONGITUDE:
    optionText = "GPS Longitude";
    break;
  case COLDSENSES_OPTION_CHECK_VERSION:
    optionText = "Check Version";
    break;
  }

  lv_obj_t *optionListLabel = lv_label_create(optionListPanelHolder);
  lv_obj_set_width(optionListLabel, 360);
  lv_obj_set_height(optionListLabel, LV_SIZE_CONTENT); /// 1
  lv_obj_set_x(optionListLabel, 18);
  lv_obj_set_y(optionListLabel, 0);
  lv_obj_set_align(optionListLabel, LV_ALIGN_LEFT_MID);
  lv_label_set_long_mode(optionListLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_label_set_text(optionListLabel, optionText.c_str());
  lv_obj_set_style_text_font(optionListLabel, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_t *optionNextImage = lv_img_create(optionListPanelHolder);
  lv_img_set_src(optionNextImage, &ui_img_chevron_right_png);
  lv_obj_set_width(optionNextImage, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(optionNextImage, LV_SIZE_CONTENT); /// 1
  lv_obj_set_align(optionNextImage, LV_ALIGN_RIGHT_MID);
  lv_obj_add_flag(optionNextImage, LV_OBJ_FLAG_ADV_HITTEST);  /// Flags
  lv_obj_clear_flag(optionNextImage, LV_OBJ_FLAG_SCROLLABLE); /// Flags

  lv_obj_t *optionEditMarkImage = lv_img_create(optionListPanelHolder);
  lv_img_set_src(optionEditMarkImage, &ui_img_asterisk_png);
  lv_obj_set_width(optionEditMarkImage, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_height(optionEditMarkImage, LV_SIZE_CONTENT); /// 1
  lv_obj_set_align(optionEditMarkImage, LV_ALIGN_LEFT_MID);
  lv_obj_add_flag(optionEditMarkImage, LV_OBJ_FLAG_ADV_HITTEST);  /// Flags
  lv_obj_clear_flag(optionEditMarkImage, LV_OBJ_FLAG_SCROLLABLE); /// Flags

  if (event)
  {
    lv_obj_add_event_cb(optionListPanelHolder, event, LV_EVENT_ALL, NULL);
  }

  lv_obj_toggle_display(optionEditMarkImage, false);

  holder.option_panel = optionListPanelHolder;
  holder.mark_edit_img = optionEditMarkImage;
  holder.type = option;
}

void lv_obj_toggle_display(lv_obj_t *obj, bool display)
{
  if (display)
  {
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
  else
  {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
}

void lv_obj_toggle_clickable(lv_obj_t *obj, bool clickable)
{
  if (clickable)
  {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
  }
  else
  {
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
  }
}

static void ui_event_wifi_ssid_option(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    inputTarget = COLDSENSES_TARGET_WIFI_SSID;
    restoreOptionToTemp(inputTarget);
    changeInputValueTo(inputTemp, false, SSID_MAXLENGTH);
    transitionToInputScreen();
  }
}

static void ui_event_wifi_password_option(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    inputTarget = COLDSENSES_TARGET_WIFI_PASSWORD;
    restoreOptionToTemp(inputTarget);
    changeInputValueTo(inputTemp, false, WIFIPW_MAXLENGTH);
    transitionToInputScreen();
  }
}

static void ui_event_gps_lat_option(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    inputTarget = COLDSENSES_TARGET_GPS_LATITUDE;
    restoreOptionToTemp(inputTarget);
    changeInputValueTo(inputTemp, false, 20);
    transitionToInputScreen();
  }
}
static void ui_event_gps_lng_option(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    inputTarget = COLDSENSES_TARGET_GPS_LONGITUDE;
    restoreOptionToTemp(inputTarget);
    changeInputValueTo(inputTemp, false, 20);
    transitionToInputScreen();
  }
}

static void ui_event_check_version_option(lv_event_t *e)
{

  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_CLICKED)
  {
    afterAlarmAction = COLDSENSES_ACTION_AFTER_CHECK_VERSION;
    String text = "Coldsenses Gateway\nVersion: ";
    text += COLDSENSES_GATEWAY_VERSION_FULL;
    text += " (";
    text += COLDSENSES_GATEWAY_VERSION;
    text += ")";
    changeAlarmScreen(text, true, false, false);
    transitionToAlertScreen();
  }
}

void transitionToHomeScreen()
{
  lv_scr_load_anim(ui_HomeScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
  lv_obj_set_parent(ui_StatusPanel, ui_HomeScreen);
  lv_label_set_text(ui_TitleLabel, "Home");
}

void transitionToOptionScreen()
{
  updateOptionScreen();
  lv_scr_load_anim(ui_OptionScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
  lv_obj_set_parent(ui_StatusPanel, ui_OptionScreen);
  lv_label_set_text(ui_TitleLabel, "Options");
}

void transitionToInputScreen()
{
  lv_scr_load_anim(ui_InputScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
  lv_obj_set_parent(ui_StatusPanel, ui_InputScreen);
  lv_label_set_text(ui_TitleLabel, "Options");
}

void transitionToAlertScreen()
{
  lv_scr_load_anim(ui_AlertScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
}

void changeAlarmScreen(String message, bool showOK, bool showCancel, bool showSpinner)
{
  lv_obj_toggle_display(ui_ActionOkButton, showOK);
  lv_obj_toggle_clickable(ui_ActionOkButton, showOK);
  lv_obj_toggle_display(ui_ActionCancelButton, showCancel);
  lv_obj_toggle_clickable(ui_ActionCancelButton, showCancel);

  // TODO handle single show

  lv_obj_toggle_display(ui_AlertSpinner, showSpinner);

  lv_label_set_text(ui_AlertLabel, message.c_str());
}

void changeInputValueTo(String &value, bool passwordMode, int maxLength)
{
  lv_textarea_set_text(ui_InputTextArea, value.c_str());
  lv_textarea_set_password_mode(ui_InputTextArea, passwordMode);
  lv_textarea_set_max_length(ui_InputTextArea, maxLength);
}

static void updateStatusBar()
{
  if (millis() - blinkLastTs > 2000)
  {
    blinkLastTs = millis();
  }
  updateWifiStatusUI();
  updateBLEStatusUI();
  updateMqttStatusUI();
}

static void updateWifiStatusUI()
{
  switch (wifiState)
  {
  case COLDSENSES_WL_DISCONNECTED:
  case COLDSENSES_WL_CONNECT_FAILED:
    lv_obj_toggle_display(ui_WifiStatusImage, false);
    lv_obj_toggle_display(ui_WifiOffStatusImage, true);
    break;
  case COLDSENSES_WL_CONNECTED:
    lv_obj_toggle_display(ui_WifiOffStatusImage, false);
    lv_obj_toggle_display(ui_WifiStatusImage, true);
    break;
  case COLDSENSES_WL_WAITING:
  default:
    lv_obj_toggle_display(ui_WifiOffStatusImage, false);
    bool isImgHidden = millis() - blinkLastTs < 1000;
    lv_obj_toggle_display(ui_WifiStatusImage, !isImgHidden);
  }
}

static void updateBLEStatusUI()
{
  switch (tagState)
  {
  case COLDSENSES_TAG_NOT_SCAN:
    lv_obj_toggle_display(ui_BLEStatusImage, false);
    break;
  case COLDSENSES_TAG_SCANNED:
    lv_obj_toggle_display(ui_BLEStatusImage, true);
    break;
  case COLDSENSES_TAG_WAITING:
  default:
    bool isImgHidden = millis() - blinkLastTs < 1000;
    lv_obj_toggle_display(ui_BLEStatusImage, !isImgHidden);
  }
}

static void updateMqttStatusUI()
{
  switch (mqttState)
  {
  case COLDSENSES_MQTT_DISCONNECTED:
  case COLDSENSES_MQTT_CONNECT_FAILED:
    lv_obj_toggle_display(ui_ServerStatusImage, false);
    lv_obj_toggle_display(ui_ServerOffStatusImage, true);
    break;
  case COLDSENSES_MQTT_CONNECTED:
    lv_obj_toggle_display(ui_ServerOffStatusImage, false);
    lv_obj_toggle_display(ui_ServerStatusImage, true);
    break;
  case COLDSENSES_MQTT_WAITING:
  default:
    lv_obj_toggle_display(ui_ServerOffStatusImage, false);
    bool isImgHidden = millis() - blinkLastTs < 1000;
    lv_obj_toggle_display(ui_ServerStatusImage, !isImgHidden);
    break;
  }
}

static void updateHomeScreen()
{
  if (tagState == COLDSENSES_TAG_SCANNED)
  {
    if (!spinnerHide)
    {
      lv_obj_toggle_display(ui_TagSpinner, false);
      spinnerHide = true;
    }

    int tagsCount = miTagScanner.getTagsCount();
    MiTagData *orderedTagData[tagsCount];
    int orderIndex = 0;
    for (int i = 0; i < tagsCount; i++)
    {
      MiTagData *tagData = miTagScanner.getTagDataAt(i);
      if (tagData && miTagScanner.isTagActive(tagData))
      {
        orderedTagData[orderIndex] = tagData;
        orderIndex += 1;
      }
    }
    for (int i = 0; i < tagsCount; i++)
    {
      MiTagData *tagData = miTagScanner.getTagDataAt(i);
      if (tagData && !miTagScanner.isTagActive(tagData))
      {
        orderedTagData[orderIndex] = tagData;
        orderIndex += 1;
      }
    }

    for (int i = 0; i < MAX_TAGS_REMEMBER; i++)
    {
      updateTagHolderData(i < orderIndex ? orderedTagData[i] : NULL, i);
    }

    String textCountDisplay = String(miTagScanner.getActiveTagCount(), 10);
    textCountDisplay += "/";
    textCountDisplay += String(tagsCount, 10);
    lv_label_set_text(ui_TagCountLabel, textCountDisplay.c_str());
  }
}

static void updateTagHolderData(MiTagData *tagDataRef, int i)
{
  ui_coldsenses_tag_holder holder = uiTagHolders[i];

  if (holder.tag_panel)
  {
    lv_obj_toggle_display(holder.tag_panel, !!tagDataRef);
  }

  if (!tagDataRef)
  {
    return;
  }

  MiTagData tagData = (*tagDataRef);

  if (holder.tag_panel)
  {
    lv_obj_set_style_opa(holder.tag_panel, miTagScanner.isTagActive(tagDataRef) ? 255 : 128, LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (holder.mac_label)
  {
    lv_label_set_text(holder.mac_label, prettyMacAddress(tagData.rawMacAddress).c_str());
  }

  if (holder.name_label)
  {
    lv_label_set_text(holder.name_label, tagData.name.c_str());
  }

  if (holder.temp_label)
  {
    lv_label_set_text(holder.temp_label, String(tagData.tempC, 1).c_str());
  }

  if (holder.humid_label)
  {
    lv_label_set_text(holder.humid_label, String(tagData.humidRH, 0).c_str());
  }
}

static void updateOptionScreen()
{
  bool wifiSSIDDirty = isOptionDirty(COLDSENSES_TARGET_WIFI_SSID);
  lv_obj_toggle_display(uiWifiSSIDOptionHolder.mark_edit_img, wifiSSIDDirty);

  bool wifiPasswordDirty = isOptionDirty(COLDSENSES_TARGET_WIFI_PASSWORD);
  lv_obj_toggle_display(uiWifiPasswordOptionHolder.mark_edit_img, wifiPasswordDirty);

  bool gpsLatDirty = isOptionDirty(COLDSENSES_TARGET_GPS_LATITUDE);
  lv_obj_toggle_display(uiGpsLatOptionHolder.mark_edit_img, gpsLatDirty);

  bool gpsLngDirty = isOptionDirty(COLDSENSES_TARGET_GPS_LONGITUDE);
  lv_obj_toggle_display(uiGpsLngOptionHolder.mark_edit_img, gpsLngDirty);

  bool isDirty = isOptionsDirty();

  lv_obj_toggle_display(ui_OptionResetButton, isDirty);
  lv_obj_toggle_clickable(ui_OptionResetButton, isDirty);

  bool isCanSave = isDirty && isOptionsValid();
  lv_obj_toggle_display(ui_OptionSaveButton, isCanSave);
  lv_obj_toggle_clickable(ui_OptionSaveButton, isCanSave);
}