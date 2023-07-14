#ifndef __COLDSENSES_ENUMS__
#define __COLDSENSES_ENUMS__

#include "ui/ui.h"

typedef enum
{
  COLDSENSES_WL_WAITING,
  COLDSENSES_WL_CONNECTED,
  COLDSENSES_WL_DISCONNECTED,
  COLDSENSES_WL_CONNECT_FAILED,
} coldsenses_wifi_state;

typedef enum
{
  COLDSENSES_TAG_WAITING,
  COLDSENSES_TAG_SCANNED,
  COLDSENSES_TAG_NOT_SCAN,
} coldsenses_tag_state;

typedef enum
{
  COLDSENSES_MQTT_WAITING,
  COLDSENSES_MQTT_CONNECTED,
  COLDSENSES_MQTT_DISCONNECTED,
  COLDSENSES_MQTT_CONNECT_FAILED,
} coldsenses_mqtt_state;

typedef enum
{
  COLDSENSES_OPTION_SET_WIFI_SSID,
  COLDSENSES_OPTION_SET_WIFI_PASSWORD,
  COLDSENSES_OPTION_SET_GPS_LATITUDE,
  COLDSENSES_OPTION_SET_GPS_LONGITUDE,
  COLDSENSES_OPTION_CHECK_VERSION,
} coldsenses_option;

typedef enum
{
  COLDSENSES_NO_TARGET,
  COLDSENSES_TARGET_WIFI_SSID,
  COLDSENSES_TARGET_WIFI_PASSWORD,
  COLDSENSES_TARGET_GPS_LATITUDE,
  COLDSENSES_TARGET_GPS_LONGITUDE,
} coldsenses_input_target;

typedef enum
{
  COLDSENSES_NO_ACTION,
  COLDSENSES_ACTION_SAVE_CONFIRM,
  COLDSENSES_ACTION_SAVE_RESET,
  COLDSENSES_ACTION_SAVE_CANCEL,
  COLDSENSES_ACTION_INPUT_CONFIRM,
  COLDSENSES_ACTION_INPUT_RESET,
  COLDSENSES_ACTION_INPUT_CANCEL,
  COLDSENSES_ACTION_AFTER_CHECK_VERSION,
} coldsenses_after_alarm_action;

typedef struct
{
  lv_obj_t *tag_panel;
  lv_obj_t *mac_label;
  lv_obj_t *name_label;
  lv_obj_t *temp_label;
  lv_obj_t *humid_label;
} ui_coldsenses_tag_holder;

typedef struct
{
  lv_obj_t *option_panel;
  lv_obj_t *mark_edit_img;
  coldsenses_option type;
} ui_coldsenses_option_holder;

#endif