#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <lvgl.h>

// Maximum number of independent control sets
#define MAX_SETS 3

// Device modes
typedef enum {
    MODE_BS14 = 1,      // Single control set (default)
    MODE_BS32 = 3,      // Three control sets
    MODE_SUB_48 = 4,    // Single control set (same layout as BS14)
    MODE_SUB_125 = 5    // Single control set (same layout as BS14)
} DeviceMode;

typedef struct {
    // State
    bool breakerstate;
    bool locked;
    bool switchToggled;
    unsigned long lock_press_start;
    unsigned long last_switch_toggle;
    // Widgets
    lv_obj_t *container;
    lv_obj_t *switch_69;
    lv_obj_t *btn_open;
    lv_obj_t *btn_close;
    lv_obj_t *lock_icon_btn;
    lv_obj_t *lock_icon_label;
    lv_obj_t *prohibition_label; // Circle slash symbol over close button
} ControlSet;

extern ControlSet sets[MAX_SETS];
extern int NUM_SETS;  // Dynamic number of sets based on selected mode
extern DeviceMode currentMode;

extern bool deviceConnected;

// Settings
extern bool senseA_selected;

// UI update functions
void update_button_styles(int idx);
void update_lock_icon(int idx);

// UI and event callback prototypes
void create_ui();
void create_mode_selection_menu();
void switch_toggled_cb(lv_event_t *e);
void open_btn_cb(lv_event_t *e);
void close_btn_cb(lv_event_t *e);
void lock_icon_event_cb(lv_event_t *e);
void settings_btn_cb(lv_event_t *e);

// BLE functions
void send_status_to_flutter();
void setup_ble();

// Mode management
void set_device_mode(DeviceMode mode);
DeviceMode load_device_mode();
void save_device_mode(DeviceMode mode);

#endif // MAIN_H
