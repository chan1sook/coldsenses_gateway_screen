// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: Coldsenses_gateway_screen

#include "../ui.h"

#ifndef LV_ATTRIBUTE_MEM_ALIGN
    #define LV_ATTRIBUTE_MEM_ALIGN
#endif

// IMAGE DATA: assets\wifi.png
const LV_ATTRIBUTE_MEM_ALIGN uint8_t ui_img_wifi_png_data[] = {
    0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x15,0x00,0x00,0x51,0x00,0x00,0x8E,0x00,0x00,0xC3,0x00,0x00,0xD8,0x00,0x00,0xEB,0x00,0x00,0xFD,0x00,0x00,0xEC,0x00,0x00,0xDA,0x00,0x00,0xC2,0x00,0x00,0x8E,0x00,0x00,0x52,0x00,0x00,0x15,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x49,0x00,0x00,0xAF,0x00,0x00,0xFB,0x00,0x00,0xFF,0x00,0x00,0xEF,0x00,0x00,0xB8,0x00,0x00,0x99,0x00,0x00,0x86,0x00,0x00,0x77,0x00,0x00,0x87,0x00,0x00,0x9C,0x00,0x00,0xB8,0x00,0x00,0xEF,0x00,0x00,0xFF,0x00,0x00,0xFA,0x00,0x00,0xAE,0x00,0x00,0x4A,0x00,0x00,0x02,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x3E,0x00,0x00,0xD1,0x00,0x00,0xFF,0x00,0x00,0xE0,0x00,0x00,0x7F,0x00,0x00,0x2B,0x00,0x00,0x02,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x2C,0x00,0x00,0x7E,
    0x00,0x00,0xE1,0x00,0x00,0xFF,0x00,0x00,0xCF,0x00,0x00,0x3D,0xFF,0xFF,0x00,0x00,0x00,0x99,0x00,0x00,0xFE,0x00,0x00,0xDA,0x00,0x00,0x4F,0x00,0x00,0x02,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x02,0x00,0x00,0x51,0x00,0x00,0xDA,0x00,0x00,0xFE,0x00,0x00,0x98,0x00,0x00,0x94,0x00,0x00,0x74,0x00,0x00,0x06,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x16,0x00,0x00,0x4C,0x00,0x00,0x7B,0x00,0x00,0x8F,0x00,0x00,0x99,0x00,0x00,0x90,0x00,0x00,0x7C,0x00,0x00,0x4C,0x00,0x00,0x16,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x06,0x00,0x00,0x73,0x00,0x00,0x92,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x05,0x00,0x00,0x60,0x00,0x00,0xC0,0x00,0x00,0xFD,0x00,0x00,0xFF,0x00,0x00,0xF7,0x00,0x00,0xE3,0x00,0x00,0xDA,0x00,0x00,0xE3,
    0x00,0x00,0xF8,0x00,0x00,0xFF,0x00,0x00,0xFD,0x00,0x00,0xBF,0x00,0x00,0x5F,0x00,0x00,0x04,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x3F,0x00,0x00,0xDA,0x00,0x00,0xFF,0x00,0x00,0xCC,0x00,0x00,0x72,0x00,0x00,0x2F,0x00,0x00,0x04,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x04,0x00,0x00,0x30,0x00,0x00,0x73,0x00,0x00,0xCD,0x00,0x00,0xFF,0x00,0x00,0xDA,0x00,0x00,0x3E,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x77,0x00,0x00,0xBE,0x00,0x00,0x42,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x43,0x00,0x00,0xBE,0x00,0x00,0x7C,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,
    0xFF,0xFF,0x00,0x00,0x00,0x08,0x00,0x00,0x25,0x00,0x00,0x39,0x00,0x00,0x25,0x00,0x00,0x09,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x08,0x00,0x00,0x68,0x00,0x00,0xC2,0x00,0x00,0xFE,0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00,0xFD,0x00,0x00,0xC1,0x00,0x00,0x68,0x00,0x00,0x08,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x45,0x00,0x00,0xFD,0x00,0x00,0xC6,0x00,0x00,0x75,0x00,0x00,0x4E,0x00,0x00,0x3A,0x00,0x00,0x4E,0x00,0x00,0x76,0x00,0x00,0xC7,0x00,0x00,0xFD,0x00,0x00,0x44,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,
    0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x08,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x09,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x09,0x00,0x00,0x6B,0x00,0x00,0x95,0x00,0x00,0x6B,0x00,0x00,0x06,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x33,0x00,0x00,0xFD,0x00,0x00,0xFF,0x00,0x00,0xFD,0x00,0x00,0x2F,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,
    0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0x00,0x00,0x5A,0x00,0x00,0xE5,0x00,0x00,0x5A,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,0xFF,0xFF,0x00,
};
const lv_img_dsc_t ui_img_wifi_png = {
    .header.always_zero = 0,
    .header.w = 23,
    .header.h = 15,
    .data_size = sizeof(ui_img_wifi_png_data),
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
    .data = ui_img_wifi_png_data
};

