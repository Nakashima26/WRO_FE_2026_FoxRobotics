// PCNT del ESP32: la cuenta la pone Python (sr_set_count).
#pragma once
#include "Arduino.h"

typedef struct sr_pcnt_unit *pcnt_unit_handle_t;
typedef struct sr_pcnt_chan *pcnt_channel_handle_t;

typedef struct {
  int low_limit;
  int high_limit;
  struct { uint32_t accum_count; } flags;
} pcnt_unit_config_t;
typedef struct { uint32_t max_glitch_ns; } pcnt_glitch_filter_config_t;
typedef struct { int edge_gpio_num; int level_gpio_num; } pcnt_chan_config_t;

enum { PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE };
enum { PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_HOLD };

esp_err_t pcnt_new_unit(const pcnt_unit_config_t *, pcnt_unit_handle_t *ret);
inline esp_err_t pcnt_unit_set_glitch_filter(pcnt_unit_handle_t, const pcnt_glitch_filter_config_t *) { return ESP_OK; }
inline esp_err_t pcnt_new_channel(pcnt_unit_handle_t, const pcnt_chan_config_t *, pcnt_channel_handle_t *) { return ESP_OK; }
inline esp_err_t pcnt_channel_set_edge_action(pcnt_channel_handle_t, int, int) { return ESP_OK; }
inline esp_err_t pcnt_channel_set_level_action(pcnt_channel_handle_t, int, int) { return ESP_OK; }
inline esp_err_t pcnt_unit_add_watch_point(pcnt_unit_handle_t, int) { return ESP_OK; }
inline esp_err_t pcnt_unit_enable(pcnt_unit_handle_t) { return ESP_OK; }
inline esp_err_t pcnt_unit_start(pcnt_unit_handle_t) { return ESP_OK; }
esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t);
esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t, int *value);
