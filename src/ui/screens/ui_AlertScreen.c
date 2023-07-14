// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.6
// Project name: Coldsenses_gateway_screen

#include "../ui.h"

void ui_AlertScreen_screen_init(void)
{
    ui_AlertScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_AlertScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_AlertLabel = lv_label_create(ui_AlertScreen);
    lv_obj_set_width(ui_AlertLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_AlertLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AlertLabel, 0);
    lv_obj_set_y(ui_AlertLabel, -60);
    lv_obj_set_align(ui_AlertLabel, LV_ALIGN_CENTER);
    lv_label_set_long_mode(ui_AlertLabel, LV_LABEL_LONG_CLIP);
    lv_label_set_text(ui_AlertLabel, "Line1\nLine2\nLine3");
    lv_obj_set_style_text_letter_space(ui_AlertLabel, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui_AlertLabel, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_AlertLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_AlertLabel, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_AlertSpinner = lv_spinner_create(ui_AlertScreen, 1000, 90);
    lv_obj_set_width(ui_AlertSpinner, 80);
    lv_obj_set_height(ui_AlertSpinner, 80);
    lv_obj_set_x(ui_AlertSpinner, 0);
    lv_obj_set_y(ui_AlertSpinner, -50);
    lv_obj_set_align(ui_AlertSpinner, LV_ALIGN_BOTTOM_MID);
    lv_obj_clear_flag(ui_AlertSpinner, LV_OBJ_FLAG_CLICKABLE);      /// Flags

    ui_ActionOkButton = lv_btn_create(ui_AlertScreen);
    lv_obj_set_width(ui_ActionOkButton, 62);
    lv_obj_set_height(ui_ActionOkButton, 60);
    lv_obj_set_x(ui_ActionOkButton, 2);
    lv_obj_set_y(ui_ActionOkButton, 0);
    lv_obj_set_align(ui_ActionOkButton, LV_ALIGN_BOTTOM_RIGHT);
    lv_obj_add_flag(ui_ActionOkButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_ActionOkButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_ActionOkButton, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ActionOkButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ActionOkButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_ActionOkButton, lv_color_hex(0xE6E2E6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_ActionOkButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_ActionOkButton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ActionOkImage = lv_img_create(ui_ActionOkButton);
    lv_img_set_src(ui_ActionOkImage, &ui_img_check_png);
    lv_obj_set_width(ui_ActionOkImage, LV_SIZE_CONTENT);   /// 24
    lv_obj_set_height(ui_ActionOkImage, LV_SIZE_CONTENT);    /// 24
    lv_obj_set_align(ui_ActionOkImage, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ActionOkImage, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ActionOkImage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_ActionOkImage, 384);

    ui_ActionCancelButton = lv_btn_create(ui_AlertScreen);
    lv_obj_set_width(ui_ActionCancelButton, 62);
    lv_obj_set_height(ui_ActionCancelButton, 60);
    lv_obj_set_x(ui_ActionCancelButton, -2);
    lv_obj_set_y(ui_ActionCancelButton, 0);
    lv_obj_set_align(ui_ActionCancelButton, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_add_flag(ui_ActionCancelButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_ActionCancelButton, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_ActionCancelButton, 4, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_ActionCancelButton, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ActionCancelButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_ActionCancelButton, lv_color_hex(0xE6E2E6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_ActionCancelButton, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_ActionCancelButton, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ActionCancelImage = lv_img_create(ui_ActionCancelButton);
    lv_img_set_src(ui_ActionCancelImage, &ui_img_x_png);
    lv_obj_set_width(ui_ActionCancelImage, LV_SIZE_CONTENT);   /// 24
    lv_obj_set_height(ui_ActionCancelImage, LV_SIZE_CONTENT);    /// 24
    lv_obj_set_align(ui_ActionCancelImage, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ActionCancelImage, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ActionCancelImage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_ActionCancelImage, 384);

    lv_obj_add_event_cb(ui_ActionOkButton, ui_event_ActionOkButton, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ActionCancelButton, ui_event_ActionCancelButton, LV_EVENT_ALL, NULL);

}