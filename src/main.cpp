#include <ArduinoJson.h>
#include "main.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

/*
 * BLE BIDIRECTIONAL COMMUNICATION API
 * 
 * Device Name: "BS32-Device"
 * Service UUID: "12345678-1234-1234-1234-123456789abc"
 * 
 * CHARACTERISTICS:
 * 
 * 1. COMMAND (87654321-4321-4321-4321-cba987654321) - Read/Write/Notify
 *    - Send commands TO ESP32, receive acknowledgments FROM ESP32
 *    
 *    Send Commands (JSON):
 *    {"type": "ping"}  // Heartbeat ping
 *    {"type": "status_request"}  // Request full status
 *    {"type": "sense_select", "value": true}  // true=Sense A, false=Sense B
 *    {"setIndex": 0, "property": "breaker", "value": true}  // Control breaker
 *    {"setIndex": 1, "property": "switch", "value": false}  // Control switch  
 *    {"setIndex": 2, "property": "locked", "value": true}   // Control lock
 *    
 *    Receive Acknowledgments (JSON):
 *    {"type": "ack", "command": "breaker", "setIndex": 0, "success": true, "message": "Breaker opened", "timestamp": 12345}
 * 
 * 2. STATUS (11011111-2222-3333-4444-555555555555) - Read/Notify
 *    - 9 bytes: [set0_breaker, set0_switch, set0_locked, set1_breaker, set1_switch, set1_locked, set2_breaker, set2_switch, set2_locked]
 *    - Values: 0=false/off/closed, 1=true/on/open
 * 
 * 3. SENSE (33333333-4444-5555-6666-777777777777) - Read/Write/Notify  
 *    - 1 byte: 0=Sense A selected, 1=Sense B selected
 * 
 * 4. HEARTBEAT (44444444-5555-6666-7777-888888888888) - Read/Notify
 *    - 4 bytes: timestamp (milliseconds since boot) for connection monitoring
 * 
 * 5. LOCK (22222222-3333-4444-5555-666666666666) - Read/Write/Notify [Legacy]
 *    - 1 byte: lock status of first control set (for backward compatibility)
 */

// UUIDs (should match Flutter app)
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define COMMAND_CHAR_UUID   "87654321-4321-4321-4321-cba987654321"
#define STATUS_CHAR_UUID    "11011111-2222-3333-4444-555555555555"
#define LOCK_CHAR_UUID      "22222222-3333-4444-5555-666666666666"
#define SENSE_CHAR_UUID     "33333333-4444-5555-6666-777777777777"
#define HEARTBEAT_CHAR_UUID "44444444-5555-6666-7777-888888888888"

BLEServer* pServer = nullptr;
BLECharacteristic* pCommandChar = nullptr;
BLECharacteristic* pStatusChar = nullptr;
BLECharacteristic* pLockChar = nullptr;
BLECharacteristic* pSenseChar = nullptr;
BLECharacteristic* pHeartbeatChar = nullptr;
bool deviceConnected = false;
bool senseA_selected = true; // Default to Sense A (will be loaded from preferences)
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

// Persistent storage for sense selection
Preferences preferences;
const char* PREF_NAMESPACE = "bs32";
const char* PREF_SENSE_KEY = "sense_select";
const char* PREF_MODE_KEY = "device_mode";

// Device mode management
int NUM_SETS = 1;  // Default to BS14 (1 set)
DeviceMode currentMode = MODE_BS14;
lv_obj_t *mode_selection_menu = nullptr;

// Save sense selection to persistent storage
void save_sense_selection() {
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putBool(PREF_SENSE_KEY, senseA_selected);
    preferences.end();
    Serial.printf("[Storage] Saved sense selection: %s\n", senseA_selected ? "A" : "B");
}

// Load sense selection from persistent storage
void load_sense_selection() {
    preferences.begin(PREF_NAMESPACE, true);
    if (preferences.isKey(PREF_SENSE_KEY)) {
        senseA_selected = preferences.getBool(PREF_SENSE_KEY, true);
        Serial.printf("[Storage] Loaded sense selection: %s\n", senseA_selected ? "A" : "B");
    } else {
        // First time - use default and save it
        senseA_selected = true;
        preferences.end();
        save_sense_selection();
        Serial.println("[Storage] First boot - using default Sense A and saving");
    }
    preferences.end();
}

// Helper function to get mode name string
const char* get_mode_name(DeviceMode mode) {
    switch(mode) {
        case MODE_BS14: return "BS14";
        case MODE_BS32: return "BS32";
        case MODE_SUB_48: return "Sub-48";
        case MODE_SUB_125: return "Sub-125";
        default: return "Unknown";
    }
}

// Mode management functions
DeviceMode load_device_mode() {
    preferences.begin(PREF_NAMESPACE, true);
    DeviceMode mode = MODE_BS14; // Default to BS14
    if (preferences.isKey(PREF_MODE_KEY)) {
        int modeValue = preferences.getInt(PREF_MODE_KEY, (int)MODE_BS14);
        mode = (DeviceMode)modeValue;
        Serial.printf("[Storage] Loaded device mode: %s\n", get_mode_name(mode));
    } else {
        // First time - use default BS14 and save it
        preferences.end();
        save_device_mode(MODE_BS14);
        Serial.println("[Storage] First boot - using default BS14 mode and saving");
        return MODE_BS14;
    }
    preferences.end();
    return mode;
}

void save_device_mode(DeviceMode mode) {
    preferences.begin(PREF_NAMESPACE, false);
    preferences.putInt(PREF_MODE_KEY, (int)mode);
    preferences.end();
    Serial.printf("[Storage] Saved device mode: %s\n", get_mode_name(mode));
}

void set_device_mode(DeviceMode mode) {
    currentMode = mode;
    // Set NUM_SETS based on mode
    if (mode == MODE_BS14) {
        NUM_SETS = 1;
    } else if (mode == MODE_BS32) {
        NUM_SETS = 3;
    } else if (mode == MODE_SUB_48) {
        NUM_SETS = 1;
    } else if (mode == MODE_SUB_125) {
        NUM_SETS = 1;
    } else {
        NUM_SETS = 1; // Default
    }
    save_device_mode(mode);
    Serial.printf("[Mode] Switched to %s mode (%d control sets)\n", 
                 get_mode_name(mode), NUM_SETS);
    // Note: UI recreation is handled by the mode selection menu callbacks
}

// Send JSON state update for a set to Flutter
void send_set_state_to_flutter(int setIndex, const char* property, bool value) {
    if (!deviceConnected) return;
    
    StaticJsonDocument<128> doc;
    doc["setIndex"] = setIndex;
    doc["property"] = property;
    doc["value"] = value;
    doc["source"] = "esp32";
    char buffer[128];
    serializeJson(doc, buffer);
    
    Serial.printf("[BLE TX] Breaker %d: %s = %s\n", setIndex + 1, property, value ? "true" : "false");
    
    pCommandChar->setValue((uint8_t*)buffer, strlen(buffer));
    pCommandChar->notify();
}

void send_status_to_flutter() {
    if (!deviceConnected) return;
    
    // Send comprehensive status for all control sets (dynamic based on NUM_SETS)
    const int statusSize = NUM_SETS * 3; // 3 bytes per set: breaker, switch, locked
    uint8_t status[9]; // Max 3 sets = 9 bytes
    
    for (int i = 0; i < NUM_SETS; i++) {
        status[i*3] = sets[i].breakerstate ? 1 : 0;
        status[i*3+1] = sets[i].switchToggled ? 1 : 0;
        status[i*3+2] = sets[i].locked ? 1 : 0;
    }
    
    // Print status for all sets
    Serial.printf("[BLE TX STATUS] Mode: %s, Sets: %d - ", 
                 currentMode == MODE_BS14 ? "BS14" : "BS32", NUM_SETS);
    for (int i = 0; i < NUM_SETS; i++) {
        Serial.printf("[B:%d S:%d L:%d] ", status[i*3], status[i*3+1], status[i*3+2]);
    }
    Serial.println();
    
    pStatusChar->setValue(status, statusSize);
    pStatusChar->notify();
    
    // Send sense selection
    uint8_t senseVal = senseA_selected ? 0 : 1; // 0=A, 1=B
    pSenseChar->setValue(&senseVal, 1);
    pSenseChar->notify();
}

// Send acknowledgment back to app
void send_ack_to_flutter(const char* command, int setIndex, bool success, const char* message = "") {
    if (!deviceConnected) return;
    
    StaticJsonDocument<200> doc;
    doc["type"] = "ack";
    doc["command"] = command;
    doc["setIndex"] = setIndex;
    doc["success"] = success;
    doc["message"] = message;
    doc["timestamp"] = millis();
    
    char buffer[200];
    serializeJson(doc, buffer);
    pCommandChar->setValue((uint8_t*)buffer, strlen(buffer));
    pCommandChar->notify();
}

// Send heartbeat
void send_heartbeat() {
    if (!deviceConnected) return;
    
    uint32_t timestamp = millis();
    uint8_t heartbeatData[4];
    heartbeatData[0] = (timestamp >> 24) & 0xFF;
    heartbeatData[1] = (timestamp >> 16) & 0xFF;
    heartbeatData[2] = (timestamp >> 8) & 0xFF;
    heartbeatData[3] = timestamp & 0xFF;
    
    pHeartbeatChar->setValue(heartbeatData, 4);
    pHeartbeatChar->notify();
}

// Send mode change notification to Flutter app
void send_mode_change_to_flutter() {
    if (!deviceConnected) return;
    
    StaticJsonDocument<128> doc;
    doc["type"] = "mode_changed";
    doc["mode"] = get_mode_name(currentMode);
    char buffer[128];
    serializeJson(doc, buffer);
    
    Serial.printf("[BLE TX] Mode changed to: %s\n", get_mode_name(currentMode));
    
    pCommandChar->setValue((uint8_t*)buffer, strlen(buffer));
    pCommandChar->notify();
}

// Forward declarations needed before use in callbacks
void update_bluetooth_ui();
void update_sense_buttons();
void update_top_bar_bt_status();
void change_device_mode_and_recreate_ui(DeviceMode newMode);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("[BLE] Device connected - will send status after brief delay");
        // Small delay to ensure notifications are enabled before sending
        delay(100);
        send_status_to_flutter();
        // Update Bluetooth UI if settings modal is open
        update_bluetooth_ui();
        // Update top bar status
        update_top_bar_bt_status();
    }
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        // Update Bluetooth UI if settings modal is open
        update_bluetooth_ui();
        // Update top bar status
        update_top_bar_bt_status();
    }
};

// Status characteristic callbacks - update value when read
class StatusCharCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* pCharacteristic) override {
        // Update characteristic with current status before read
        const int statusSize = NUM_SETS * 3; // 3 bytes per set: breaker, switch, locked
        uint8_t status[9]; // Max 3 sets = 9 bytes
        for (int i = 0; i < NUM_SETS; i++) {
            status[i*3] = sets[i].breakerstate ? 1 : 0;
            status[i*3+1] = sets[i].switchToggled ? 1 : 0;
            status[i*3+2] = sets[i].locked ? 1 : 0;
        }
        pCharacteristic->setValue(status, statusSize);
        Serial.printf("[BLE] Status characteristic read - sending current status (%d sets)\n", NUM_SETS);
    }
};

// Sense characteristic callbacks
class SenseCharCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* pCharacteristic) override {
        // Update characteristic with current sense selection before read
        uint8_t senseVal = senseA_selected ? 0 : 1; // 0=A, 1=B
        pCharacteristic->setValue(&senseVal, 1);
        Serial.printf("[BLE] Sense characteristic read - sending current selection: %s\n", senseA_selected ? "A" : "B");
    }
    
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String valueStr = pCharacteristic->getValue();
        if (valueStr.length() >= 1) {
            uint8_t senseVal = valueStr[0];
            bool selectA = (senseVal == 0); // 0 = Sense A, 1 = Sense B
            
            if (senseA_selected != selectA) {
                senseA_selected = selectA;
                save_sense_selection(); // Save to persistent storage
                
                Serial.printf("[BLE] Sense characteristic written - changed to: %s\n", selectA ? "A" : "B");
                
                // Update UI buttons in real-time if settings modal is open
                update_sense_buttons();
                
                send_status_to_flutter(); // Update status with sense selection
            } else {
                Serial.printf("[BLE] Sense characteristic written - same value (ignored): %s\n", selectA ? "A" : "B");
            }
        }
    }
};

class CommandCharCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        std::string value = std::string(pCharacteristic->getValue().c_str());
        Serial.printf("[BLE] Received command: %s\n", value.c_str());
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, value);
        
        if (error) {
            Serial.printf("[BLE] Failed to parse JSON: %s\n", error.c_str());
            send_ack_to_flutter("unknown", -1, false, "Invalid JSON");
            return;
        }

        // Handle different command types
        if (doc.containsKey("type")) {
            const char* cmdType = doc["type"];
            
            // Heartbeat response
            if (strcmp(cmdType, "ping") == 0) {
                send_ack_to_flutter("ping", -1, true, "pong");
                return;
            }
            
            // Request full status
            if (strcmp(cmdType, "status_request") == 0) {
                send_status_to_flutter();
                send_ack_to_flutter("status_request", -1, true, "Status sent");
                return;
            }
            
            // Sense selection
            if (strcmp(cmdType, "sense_select") == 0 && doc.containsKey("value")) {
                bool selectA = doc["value"]; // true for A, false for B
                senseA_selected = selectA;
                save_sense_selection(); // Save to persistent storage
                
                Serial.printf("[BLE] Sense selection changed to: %s (via BLE) - saved to storage\n", selectA ? "A" : "B");
                
                // Update UI buttons in real-time if settings modal is open
                update_sense_buttons();
                
                send_ack_to_flutter("sense_select", -1, true, selectA ? "Sense A selected" : "Sense B selected");
                send_status_to_flutter(); // Update status with sense selection
                return;
            }
            
            // Mode change request from app
            if (strcmp(cmdType, "mode_change_request") == 0 && doc.containsKey("mode")) {
                const char* modeStr = doc["mode"];
                DeviceMode newMode = currentMode;
                
                if (strcmp(modeStr, "BS14") == 0) {
                    newMode = MODE_BS14;
                } else if (strcmp(modeStr, "BS32") == 0) {
                    newMode = MODE_BS32;
                } else if (strcmp(modeStr, "Sub-48") == 0) {
                    newMode = MODE_SUB_48;
                } else if (strcmp(modeStr, "Sub-125") == 0) {
                    newMode = MODE_SUB_125;
                }
                
                if (newMode != currentMode) {
                    Serial.printf("[BLE] Mode change request from app: %s -> %s\n", get_mode_name(currentMode), get_mode_name(newMode));
                    change_device_mode_and_recreate_ui(newMode);
                    send_ack_to_flutter("mode_change_request", -1, true, "Mode changed");
                    send_status_to_flutter();
                } else {
                    send_ack_to_flutter("mode_change_request", -1, true, "Mode already set");
                }
                return;
            }
        }

        // Handle control set commands - require setIndex for these
        if (!doc.containsKey("setIndex")) {
            Serial.println("[BLE] Command missing setIndex field");
            send_ack_to_flutter("unknown", -1, false, "Missing setIndex");
            return;
        }
        
        int setIndex = doc["setIndex"];
        Serial.printf("[BLE] Processing command for breaker %d (setIndex=%d)\n", setIndex + 1, setIndex);
        
        if (setIndex < 0 || setIndex >= NUM_SETS) {
            Serial.printf("[BLE] Invalid setIndex: %d (valid range: 0-%d)\n", setIndex, NUM_SETS - 1);
            send_ack_to_flutter("invalid", setIndex, false, "Invalid setIndex");
            return;
        }

        bool success = false;
        String message = "";
        
        // Property-based commands
        if (doc.containsKey("property") && doc.containsKey("value")) {
            const char* property = doc["property"];
            bool newValue = doc["value"];
            
            if (strcmp(property, "breaker") == 0) {
                bool oldValue = sets[setIndex].breakerstate;
                sets[setIndex].breakerstate = newValue;
                success = true;
                message = newValue ? "Breaker opened" : "Breaker closed";
                Serial.printf("[BLE] Breaker %d changed from %s to %s\n", 
                    setIndex + 1, oldValue ? "OPEN" : "CLOSED", newValue ? "OPEN" : "CLOSED");
            }
            else if (strcmp(property, "switch") == 0) {
                bool oldValue = sets[setIndex].switchToggled;
                sets[setIndex].switchToggled = newValue;
                success = true;
                message = newValue ? "Switch up" : "Switch down";
                Serial.printf("[BLE] Switch %d changed from %s to %s\n", 
                    setIndex + 1, oldValue ? "UP" : "DOWN", newValue ? "UP" : "DOWN");
                
                // If breaker is closed and switch is toggled down, automatically open the breaker
                if (!sets[setIndex].breakerstate && !sets[setIndex].switchToggled) {
                    sets[setIndex].breakerstate = true;  // Open the breaker
                    Serial.printf("[BLE] Breaker %d automatically opened due to switch toggle down\n", setIndex + 1);
                    send_set_state_to_flutter(setIndex, "breaker", sets[setIndex].breakerstate);
                }
            }
            else if (strcmp(property, "locked") == 0) {
                bool oldValue = sets[setIndex].locked;
                sets[setIndex].locked = newValue;
                success = true;
                message = newValue ? "Locked" : "Unlocked";
                Serial.printf("[BLE] Lock %d changed from %s to %s\n", 
                    setIndex + 1, oldValue ? "LOCKED" : "UNLOCKED", newValue ? "LOCKED" : "UNLOCKED");
            }
            
            if (success) {
                update_button_styles(setIndex);
                update_lock_icon(setIndex);
                send_set_state_to_flutter(setIndex, property, newValue);
                send_status_to_flutter(); // Send complete status update
                send_ack_to_flutter(property, setIndex, true, message.c_str());
                Serial.printf("[BLE] Successfully updated breaker %d, property: %s, value: %s\n", 
                    setIndex + 1, property, newValue ? "true" : "false");
            } else {
                Serial.printf("[BLE] Unknown property: %s for breaker %d\n", property, setIndex + 1);
                send_ack_to_flutter(property, setIndex, false, "Unknown property");
            }
        }
        // Command-based format (legacy support)
        else if (doc.containsKey("command") && doc.containsKey("data")) {
            const char* command = doc["command"];
            JsonObject data = doc["data"].as<JsonObject>();
            
            if (strcmp(command, "breaker") == 0 && data.containsKey("action")) {
                const char* action = data["action"];
                bool newValue = (strcmp(action, "open") == 0);
                sets[setIndex].breakerstate = newValue;
                update_button_styles(setIndex);
                send_set_state_to_flutter(setIndex, "breaker", newValue);
                send_status_to_flutter(); // Send complete status update
                send_ack_to_flutter("breaker", setIndex, true, action);
            }
        } else {
            send_ack_to_flutter("unknown", setIndex, false, "Unknown command format");
        }
    }
};

void setup_ble() {
    BLEDevice::init("Breaker Simulator");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Command characteristic - supports both read and write for bidirectional communication
    pCommandChar = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCommandChar->setCallbacks(new CommandCharCallbacks());
    pCommandChar->addDescriptor(new BLE2902());

    // Status characteristic - comprehensive status for all control sets
    pStatusChar = pService->createCharacteristic(
        STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pStatusChar->setCallbacks(new StatusCharCallbacks());
    pStatusChar->addDescriptor(new BLE2902());

    // Legacy lock characteristic (for compatibility)
    pLockChar = pService->createCharacteristic(
        LOCK_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pLockChar->addDescriptor(new BLE2902());

    // Sense selection characteristic (A/B selection)
    pSenseChar = pService->createCharacteristic(
        SENSE_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pSenseChar->setCallbacks(new SenseCharCallbacks());
    pSenseChar->addDescriptor(new BLE2902());

    // Heartbeat characteristic for connection monitoring
    pHeartbeatChar = pService->createCharacteristic(
        HEARTBEAT_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pHeartbeatChar->addDescriptor(new BLE2902());

    // Initialize characteristic values
    uint8_t initialStatus[9] = {0}; // Will be updated in loop
    pStatusChar->setValue(initialStatus, 9);
    uint8_t initialSense = senseA_selected ? 0 : 1;
    pSenseChar->setValue(&initialSense, 1);

    pService->start();
    
    // Set up BLE advertising with device name
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("BLE Server Started - Device: BS32-Device");
    Serial.printf("Service UUID: %s\n", SERVICE_UUID);
}
#include <stdio.h>
#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "lv_conf.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;


ControlSet sets[MAX_SETS];

/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include "lv_conf.h"


/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
//#include <demos/lv_demos.h>
//#include <examples/lv_examples.h>

// Define the GPIO pin for the physical switch
#define SWITCH_PIN 0

// LVGL software switch event callback (now after includes)
static void lv_switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    if (state) {
        Serial.println("Software switch ON");
    } else {
        Serial.println("Software switch OFF");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500); // Give serial time to initialize
    
    // Load saved sense selection from persistent storage
    load_sense_selection();

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();
#if LVGL_PORT_AVOID_TEARING_MODEt
    auto lcd = board->getLCD();
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif
    assert(board->begin());

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    // Load device mode from preferences
    currentMode = load_device_mode();
    NUM_SETS = (int)currentMode;
    Serial.printf("[Setup] Starting in %s mode (%d control sets)\n", 
                 currentMode == MODE_BS14 ? "BS14" : "BS32", NUM_SETS);

    Serial.println("Creating UI");
    lvgl_port_lock(-1);
    
    // Check if we need to show mode selection menu (first boot or mode not set)
    preferences.begin(PREF_NAMESPACE, true);
    bool hasMode = preferences.isKey(PREF_MODE_KEY);
    preferences.end();
    
    if (!hasMode) {
        // First boot - show mode selection menu
        Serial.println("[Setup] First boot - showing mode selection menu");
        create_mode_selection_menu();
    } else {
        // Mode already selected - create UI directly
        create_ui();
        for (int i = 0; i < NUM_SETS; ++i) {
            update_lock_icon(i);
            update_button_styles(i);
        }
    }
    
    lvgl_port_unlock();
    setup_ble();
}


void loop()
{
    delay(5); // LVGL is handled by ESP Panel port in a background task
    
    // Handle BLE heartbeat
    if (deviceConnected && (millis() - lastHeartbeat > HEARTBEAT_INTERVAL)) {
        send_heartbeat();
        lastHeartbeat = millis();
    }
    
    // Handle disconnected devices (restart advertising)
    static bool wasConnected = false;
    if (wasConnected && !deviceConnected) {
        delay(500); // Give some time after disconnection
        BLEDevice::startAdvertising(); // Restart advertising
        Serial.println("Restarted advertising after disconnection");
    }
    wasConnected = deviceConnected;
}

// --- UI and event logic ported from Arduino sketch ---

static int currentRotation = 0; // 0 = default, 1 = rotated 90, etc.
lv_obj_t *rotate_btn = nullptr;
lv_obj_t *row_container = nullptr;
lv_obj_t *top_bar = nullptr;

// Settings
lv_obj_t *settings_modal = nullptr;
lv_obj_t *bt_status_label = nullptr;
lv_obj_t *btn_disconnect = nullptr;
lv_obj_t *btn_sense_a = nullptr;
lv_obj_t *btn_sense_b = nullptr;
lv_obj_t *current_mode_label = nullptr; // Current mode label in settings
lv_obj_t *top_bar_bt_status = nullptr; // Bluetooth status in top bar
lv_obj_t *top_bar_mode_label = nullptr; // Mode indicator in top bar

// Forward declarations
void updateLayout(int rotation);
void update_bluetooth_ui();
void update_sense_buttons();
void update_top_bar_bt_status();


// Helper function to change mode and recreate UI (used by both button callbacks and BLE)
void change_device_mode_and_recreate_ui(DeviceMode newMode) {
    if (newMode == currentMode) {
        return; // Already in this mode
    }
    
    Serial.printf("[Mode] Changing mode from %s to %s\n", get_mode_name(currentMode), get_mode_name(newMode));
    
    // Update mode
    currentMode = newMode;
    set_device_mode(newMode);
    
    // Reset all breakers to open, unlocked, switch up when switching modes
    for (int i = 0; i < MAX_SETS; ++i) {
        sets[i].breakerstate = true;  // Open
        sets[i].switchToggled = true; // Switch up
        sets[i].locked = false;       // Unlocked
        sets[i].lock_press_start = 0;
        sets[i].last_switch_toggle = 0;
    }
    
    // Recreate UI with new mode
    lvgl_port_lock(-1);
    
    // Delete top bar and row container first
    if (top_bar) {
        lv_obj_del(top_bar);
        top_bar = nullptr;
    }
    if (row_container) {
        lv_obj_del(row_container);
        row_container = nullptr;
    }
    
    // Set black background first to prevent flash
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    lv_obj_clean(lv_scr_act());
    
    // Force immediate refresh to show black screen
    lv_disp_t *disp = lv_disp_get_default();
    lv_refr_now(disp);
    
    // Apply the current rotation to the display
    lv_disp_rot_t lv_rot = LV_DISP_ROT_NONE;
    switch(currentRotation) {
        case 0: lv_rot = LV_DISP_ROT_NONE; break;
        case 1: lv_rot = LV_DISP_ROT_90; break;
        case 2: lv_rot = LV_DISP_ROT_180; break;
        case 3: lv_rot = LV_DISP_ROT_270; break;
    }
    lv_disp_set_rotation(disp, lv_rot);
    
    create_ui(); // This will reset all breakers
    for (int i = 0; i < NUM_SETS; ++i) {
        update_lock_icon(i);
        update_button_styles(i);
    }
    lvgl_port_unlock();
    
    // Notify app of mode change if connected
    send_mode_change_to_flutter();
    delay(50); // Allow BLE notification to be sent
}

void create_top_bar() {
    // Delete existing top bar if it exists
    if (top_bar) {
        lv_obj_del(top_bar);
        top_bar = nullptr;
    }
    
    // Create top bar for settings and rotate buttons
    top_bar = lv_obj_create(lv_layer_top());
    lv_obj_set_size(top_bar, lv_obj_get_width(lv_scr_act()), 50);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x000000), 0); // Black background
    lv_obj_set_style_bg_opa(top_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(top_bar, 0, 0);
    lv_obj_clear_flag(top_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(top_bar);
    
    // Settings button
    lv_obj_t *settings_btn = lv_btn_create(top_bar);
    lv_obj_set_size(settings_btn, 110, 40);
    lv_obj_align(settings_btn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_set_style_bg_color(settings_btn, lv_color_hex(0xFF9800), 0);
    lv_obj_set_style_border_width(settings_btn, 2, 0);
    lv_obj_set_style_border_color(settings_btn, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(settings_btn, 5, 0);
    lv_obj_t *settings_label = lv_label_create(settings_btn);
    lv_label_set_text(settings_label, "SETTINGS");
    lv_obj_set_style_text_font(settings_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(settings_label, lv_color_hex(0x000000), 0);
    lv_obj_center(settings_label);
    lv_obj_add_event_cb(settings_btn, settings_btn_cb, LV_EVENT_CLICKED, NULL);

    // Bluetooth icon (no text), larger, next to settings button
    top_bar_bt_status = lv_label_create(top_bar);
    lv_obj_set_style_text_font(top_bar_bt_status, &lv_font_montserrat_28, 0); // Larger icon
    lv_obj_align_to(top_bar_bt_status, settings_btn, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    update_top_bar_bt_status();

    // Mode label centered: "Mode: <name>"
    top_bar_mode_label = lv_label_create(top_bar);
    lv_obj_set_style_text_font(top_bar_mode_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(top_bar_mode_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(top_bar_mode_label, LV_ALIGN_CENTER, 0, 0);
    char mode_buf[32];
    const char* modeText;
    if (currentMode == MODE_BS14) {
        modeText = "BS14";
    } else if (currentMode == MODE_BS32) {
        modeText = "BS32";
    } else if (currentMode == MODE_SUB_48) {
        modeText = "SUB48";
    } else if (currentMode == MODE_SUB_125) {
        modeText = "SUB125";
    } else {
        modeText = "?";
    }
    snprintf(mode_buf, sizeof(mode_buf), "Mode: %s", modeText);
    lv_label_set_text(top_bar_mode_label, mode_buf);
    
    // Rotate button with text
    rotate_btn = lv_btn_create(top_bar);
    lv_obj_set_size(rotate_btn, 120, 40);
    lv_obj_align(rotate_btn, LV_ALIGN_RIGHT_MID, -5, 0);
    lv_obj_set_style_bg_color(rotate_btn, lv_color_hex(0x2196F3), 0);
    lv_obj_set_style_border_width(rotate_btn, 2, 0);
    lv_obj_set_style_border_color(rotate_btn, lv_color_hex(0x000000), 0);
    lv_obj_set_style_radius(rotate_btn, 5, 0);
    lv_obj_t *rotate_label = lv_label_create(rotate_btn);
    lv_label_set_text(rotate_label, "ROTATE");
    lv_obj_set_style_text_font(rotate_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(rotate_label, lv_color_hex(0x000000), 0);
    lv_obj_center(rotate_label);
    lv_obj_add_event_cb(rotate_btn, [](lv_event_t *e) {
        // Set background FIRST to prevent white flash
        lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
        
        // Increment rotation: 0° -> 90° -> 180° -> 270° -> 0° (360° total)
        currentRotation = (currentRotation + 1) % 4;
        
        lv_disp_t *disp = lv_disp_get_default();
        
        // Map to LVGL rotation values
        lv_disp_rot_t lv_rot = LV_DISP_ROT_NONE;
        switch(currentRotation) {
            case 0: lv_rot = LV_DISP_ROT_NONE; break;   // 0°
            case 1: lv_rot = LV_DISP_ROT_90; break;    // 90°
            case 2: lv_rot = LV_DISP_ROT_180; break;   // 180°
            case 3: lv_rot = LV_DISP_ROT_270; break;   // 270°
        }
        
        lv_disp_set_rotation(disp, lv_rot);
        
        // Update layout immediately
        updateLayout(currentRotation);
        
        // Recreate top bar after rotation
        create_top_bar();
        
        // Force refresh to ensure everything is drawn
        lv_refr_now(disp);
    }, LV_EVENT_CLICKED, NULL);
}

void updateLayout(int rotation) {
    // Remove and recreate all UI controls based on rotation
    // For simplicity, delete and recreate row_container and its children
    if (row_container) {
        lv_obj_del(row_container);
        row_container = nullptr;
    }
    lv_coord_t screen_width = lv_obj_get_width(lv_scr_act());
    lv_coord_t screen_height = lv_obj_get_height(lv_scr_act());
    row_container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(row_container, screen_width, screen_height - 50); // Subtract top bar height
    lv_obj_set_style_bg_opa(row_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_container, 0, 0);
    lv_obj_align(row_container, LV_ALIGN_BOTTOM_MID, 0, 0); // Anchor to bottom to leave room for top bar
    
    // Portrait: use column layout (rotation 1 or 3), Landscape: use row layout (0 or 2)
    if (rotation == 1 || rotation == 3) {
        lv_obj_set_flex_flow(row_container, LV_FLEX_FLOW_COLUMN);
    } else {
        lv_obj_set_flex_flow(row_container, LV_FLEX_FLOW_ROW);
    }
    lv_obj_set_flex_align(row_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(row_container, 0, 0);
    lv_obj_set_style_pad_row(row_container, 0, 0);
    lv_obj_set_style_pad_column(row_container, 0, 0);
    lv_obj_clear_flag(row_container, LV_OBJ_FLAG_SCROLLABLE);

    bool is_portrait = (rotation == 1 || rotation == 3);
    bool is_bs14 = (NUM_SETS == 1); // BS14 mode - make everything larger
    
    for (int i = 0; i < NUM_SETS; ++i) {
        ControlSet *set = &sets[i];
        // Keep existing state, don't reset
        set->lock_press_start = 0;
        set->last_switch_toggle = 0;
        set->container = lv_obj_create(row_container);
        
        // Size container based on orientation and mode
        if (is_bs14) {
            // BS14: Fill entire screen
            if (is_portrait) {
                lv_obj_set_size(set->container, screen_width - 10, screen_height - 50); // Full height minus top bar
            } else {
                lv_obj_set_size(set->container, screen_width - 10, screen_height - 50); // Full width and height minus top bar
            }
        } else {
            // BS32: Original sizing
            if (is_portrait) {
                // Full width, height divided by 3 for portrait
                lv_obj_set_size(set->container, screen_width - 10, (screen_height - 50) / 3 - 5);
            } else {
                // Original landscape sizing
                lv_obj_set_size(set->container, ((screen_width / NUM_SETS) - 5) + 7, screen_height + 20);
            }
        }
        
        lv_obj_set_style_bg_opa(set->container, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(set->container, 3, 0);
        lv_obj_set_style_border_color(set->container, lv_color_hex(0x000000), 0);
        lv_obj_clear_flag(set->container, LV_OBJ_FLAG_SCROLLABLE);
        
        // Set up flex layout for portrait mode (switch left, buttons right)
        // For BS14 portrait, we use absolute positioning, so no flex layout needed
        if (is_portrait && !is_bs14) {
            lv_obj_set_flex_flow(set->container, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(set->container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
            lv_obj_set_style_pad_all(set->container, 10, 0);
        } else if (is_portrait && is_bs14) {
            // BS14 portrait: No flex layout, use absolute positioning
            lv_obj_set_style_pad_all(set->container, 0, 0);
        }

        // Number label (1, 2, 3) - Create it initially, will reposition later
        // Hide number label for BS14 mode
        lv_obj_t *number_label = lv_label_create(set->container);
        if (is_bs14) {
            lv_obj_add_flag(number_label, LV_OBJ_FLAG_HIDDEN); // Hide in BS14 mode
        } else {
            char num_text[4];
            snprintf(num_text, sizeof(num_text), "%d", i + 1);
            lv_label_set_text(number_label, num_text);
            lv_obj_set_style_text_color(number_label, lv_color_hex(0x000000), 0);
            lv_obj_set_style_text_font(number_label, &lv_font_montserrat_32, 0);
            // Will position after lock button is created
        }

        // Switch container - larger for BS14
        lv_obj_t *switch_container = lv_obj_create(set->container);
        if (is_bs14) {
            // BS14: Much larger switch container
            if (is_portrait) {
                // Portrait: Shorter switch container to make room for buttons above/below
                lv_coord_t switch_height = (screen_height - 50) * 0.45 + 15; // 45% height + 15px taller
                lv_obj_set_size(switch_container, (screen_width - 20) * 0.85, switch_height); // 85% width
            } else {
                // Landscape: 15px narrower
                lv_coord_t switch_width = (screen_width - 20) * 0.5 - 15; // 50% width - 15px narrower
                lv_obj_set_size(switch_container, switch_width, (screen_height - 50) * 0.8); // 80% height
            }
        } else {
            // BS32: Original size
            lv_obj_set_size(switch_container, 220, 222);
        }
        if (is_portrait) {
            if (is_bs14) {
                // BS14 portrait: Center horizontally, position in middle vertically (with space for lock above and buttons below)
                lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 100); // 100px from top (moved down 20px from 80px)
            } else {
                lv_obj_align(switch_container, LV_ALIGN_LEFT_MID, 0, 10);
            }
        } else {
            // In landscape, position switch container lower to accommodate top bar
            if (is_bs14) {
                lv_obj_align(switch_container, LV_ALIGN_LEFT_MID, 10, 0); // Centered vertically for BS14
            } else {
                lv_obj_align(switch_container, LV_ALIGN_TOP_LEFT, 0, 64); // Original position for BS32
            }
        }
        lv_obj_set_style_bg_color(switch_container, lv_color_hex(0xFFFF00), 0);
        lv_obj_set_style_border_width(switch_container, is_bs14 ? 8 : 5, 0); // Thicker border for BS14
        lv_obj_set_style_border_color(switch_container, lv_color_hex(0x000000), 0);
        lv_obj_set_style_radius(switch_container, 2, 0);
        lv_obj_set_style_pad_all(switch_container, 0, 0);
        lv_obj_clear_flag(switch_container, LV_OBJ_FLAG_SCROLLABLE);

        // UP label
        lv_obj_t *label_up = lv_label_create(switch_container);
        lv_label_set_text(label_up, "UP");
        lv_obj_set_style_text_color(label_up, lv_color_hex(0x000000), 0);
        lv_obj_set_style_text_font(label_up, is_bs14 ? &lv_font_montserrat_32 : &lv_font_montserrat_24, 0); // Larger font for BS14
        lv_obj_align(label_up, LV_ALIGN_TOP_MID, 0, is_bs14 ? 10 : 5);

        // Middle container for switch and 69 label
        lv_obj_t *middle_container = lv_obj_create(switch_container);
        if (is_bs14) {
            // BS14: Calculate sizes based on switch_container dimensions
            // Use the same calculation we used for switch_container
            lv_coord_t switch_w, switch_h;
            if (is_portrait) {
                switch_w = (screen_width - 20) * 0.85;
                switch_h = (screen_height - 50) * 0.45 + 15; // 45% height + 15px taller (matches switch_container)
            } else {
                switch_w = (screen_width - 20) * 0.5 - 15; // 50% width - 15px narrower (matches switch_container)
                switch_h = (screen_height - 50) * 0.8;
            }
            // Middle container fills most of switch container
            lv_obj_set_size(middle_container, switch_w * 0.95, switch_h * 0.75); // 95% width, 75% height
            lv_obj_align(middle_container, LV_ALIGN_CENTER, 0, 0); // Center it in the yellow container
        } else {
            // BS32: Original size
            lv_obj_set_size(middle_container, 220, 142);
            lv_obj_align(middle_container, LV_ALIGN_LEFT_MID, -40, -12);
        }
        lv_obj_set_style_pad_all(middle_container, 0, 0);
        lv_obj_set_style_border_width(middle_container, 0, 0);
        lv_obj_set_style_bg_opa(middle_container, LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(middle_container, LV_OBJ_FLAG_SCROLLABLE);

        // 69 label
        lv_obj_t *label_69 = lv_label_create(middle_container);
        lv_label_set_text(label_69, "69");
        lv_obj_set_style_text_color(label_69, lv_color_hex(0x000000), 0);
        if (is_bs14) {
            lv_obj_set_style_text_font(label_69, &lv_font_montserrat_48, 0); // Much larger for BS14
            // Position it on the left side of the middle container, centered vertically
            lv_obj_align(label_69, LV_ALIGN_LEFT_MID, 20, 0); // 20px margin from left edge
        } else {
            lv_obj_set_style_text_font(label_69, &lv_font_montserrat_24, 0);
            lv_obj_set_pos(label_69, 58, 59);
        }

        // Custom vertical switch using a button as the knob
        lv_obj_t *vertical_switch_container = lv_obj_create(middle_container);
        if (is_bs14) {
            // BS14: Fixed size for both portrait and landscape - MUST match working repo
            // Black outline: 120px width, 220px height (changed from 200 to match working repo)
            lv_coord_t vswitch_w = 120;
            lv_coord_t vswitch_h = 220;  // CHANGED from 200 to 220 to match working repo
            
            lv_obj_set_size(vertical_switch_container, vswitch_w, vswitch_h);
            // Center it horizontally in the middle container to match knob position
            lv_obj_align(vertical_switch_container, LV_ALIGN_CENTER, 0, 0); // Centered horizontally and vertically
            lv_obj_set_style_radius(vertical_switch_container, vswitch_w / 2, 0); // Fully rounded
        } else {
            // BS32: Original sizing
            if (is_portrait) {
                lv_obj_set_size(vertical_switch_container, 82, 128);
            } else {
                lv_obj_set_size(vertical_switch_container, 72, 128);
            }
            lv_obj_set_pos(vertical_switch_container, 107, 14);
            if (is_portrait) {
                lv_obj_set_style_radius(vertical_switch_container, 41, 0);
            } else {
                lv_obj_set_style_radius(vertical_switch_container, 42, 0);
            }
        }
        lv_obj_set_style_bg_opa(vertical_switch_container, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(vertical_switch_container, is_bs14 ? 8 : 2, 0); // Thicker border for BS14 (8px)
        lv_obj_set_style_border_color(vertical_switch_container, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_all(vertical_switch_container, is_bs14 ? 12 : 5, 0); // More padding for BS14
        lv_obj_clear_flag(vertical_switch_container, LV_OBJ_FLAG_SCROLLABLE);

        // Create the switch "knob"
        set->switch_69 = lv_btn_create(vertical_switch_container);
        if (is_bs14) {
            // BS14: Fixed size for knob in both portrait and landscape
            // Blue knob: 100px width x 100px height (circular)
            lv_coord_t knob_w = 100;
            lv_coord_t knob_h = 100;
            lv_obj_set_size(set->switch_69, knob_w, knob_h);
            lv_obj_set_style_radius(set->switch_69, knob_w / 2, 0); // Fully rounded (circular)
            // Set all styles and event callbacks first...
            // Now set knob position as the very last step, after all parent alignments
            // (This code block should be placed after any lv_obj_align on vertical_switch_container)
        } else {
            // BS32: Original sizing
            if (is_portrait) {
                lv_obj_set_size(set->switch_69, 65, 60);
            } else {
                lv_obj_set_size(set->switch_69, 60, 60);
            }
            lv_obj_set_style_radius(set->switch_69, 30, 0);
            if (set->switchToggled) {
                if (is_portrait) {
                    lv_obj_set_pos(set->switch_69, 20, 30);
                } else {
                    lv_obj_set_pos(set->switch_69, -20, 30);
                }
            } else {
                if (is_portrait) {
                    lv_obj_set_pos(set->switch_69, 20, 58);
                } else {
                    lv_obj_set_pos(set->switch_69, -20, 58);
                }
            }
        }
        lv_obj_set_style_border_width(set->switch_69, is_bs14 ? 2 : 1, 0);
                if (is_bs14) {
                    // Now set knob position as the very last step, after all parent alignments
                    lv_coord_t parent_w = lv_obj_get_width(vertical_switch_container);
                    lv_coord_t parent_h = lv_obj_get_height(vertical_switch_container);
                    lv_coord_t knob_w = 100;
                    lv_coord_t knob_h = 100;
                    lv_coord_t x = (parent_w - knob_w) / 2 - 10;
                    lv_coord_t y = (parent_h - knob_h) / 2 - 50;
                    lv_obj_set_pos(set->switch_69, x, y);
                }
        lv_obj_set_style_border_color(set->switch_69, lv_color_hex(0x000000), 0);
        lv_obj_clear_flag(set->switch_69, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(set->switch_69, lv_color_hex(0x2196F3), 0);
        lv_obj_set_user_data(set->switch_69, (void*)i);
        lv_obj_add_event_cb(set->switch_69, switch_toggled_cb, LV_EVENT_CLICKED, NULL);
        if (set->locked) lv_obj_add_state(set->switch_69, LV_STATE_DISABLED);

        // DOWN label
        lv_obj_t *label_down = lv_label_create(switch_container);
        lv_label_set_text(label_down, "DOWN");
        lv_obj_set_style_text_color(label_down, lv_color_hex(0x000000), 0);
        lv_obj_set_style_text_font(label_down, is_bs14 ? &lv_font_montserrat_32 : &lv_font_montserrat_24, 0); // Larger font for BS14
        lv_obj_align(label_down, LV_ALIGN_BOTTOM_MID, 0, is_bs14 ? -10 : -5);

        // Button container for open/close
        lv_obj_t *tight_container = lv_obj_create(set->container);
        if (is_portrait && is_bs14) {
            // BS14 portrait: Set width to 95% of parent container to ensure buttons fit and are centered
            lv_obj_set_size(tight_container, LV_PCT(95), LV_SIZE_CONTENT);
        } else {
            lv_obj_set_size(tight_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        }
        lv_obj_set_flex_flow(tight_container, LV_FLEX_FLOW_COLUMN);
        if (is_portrait) {
            lv_obj_set_flex_align(tight_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        } else {
            lv_obj_set_flex_align(tight_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        }
        lv_obj_set_style_pad_all(tight_container, is_bs14 ? 10 : 5, 0);
        lv_obj_set_style_pad_row(tight_container, is_bs14 ? 15 : 7, 0);
        lv_obj_set_style_border_width(tight_container, 0, 0);
        lv_obj_set_style_bg_opa(tight_container, LV_OPA_TRANSP, 0);
        if (is_portrait) {
            if (is_bs14) {
                // BS14 portrait: Position buttons below switch container, centered horizontally
                // Calculate position: switch container starts at 100px from top + 45% of screen height + 15px taller
                lv_coord_t switch_bottom = 100 + ((screen_height - 50) * 0.45 + 15);
                lv_obj_align(tight_container, LV_ALIGN_TOP_MID, 0, switch_bottom + 20); // 20px below switch container (moved up from 40px), centered
            } else {
                lv_obj_align(tight_container, LV_ALIGN_RIGHT_MID, -120, 0); // Moved left 120px to be much closer to the yellow switch container
            }
        } else {
            if (is_bs14) {
                lv_obj_align(tight_container, LV_ALIGN_RIGHT_MID, -10, 48); // Right side for BS14 landscape, moved down 15px (33px + 15px = 48px)
            } else {
                lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, -15); // Original position for BS32
            }
        }
        
        // Lock icon button - positioned differently for BS14 portrait
        if (is_portrait && is_bs14) {
            // BS14 portrait: Lock button goes above switch container
            set->lock_icon_btn = lv_btn_create(set->container);
            lv_obj_set_size(set->lock_icon_btn, (screen_width - 20) * 0.85, (screen_height - 50) * 0.10); // Same width as switch container
            lv_obj_align(set->lock_icon_btn, LV_ALIGN_TOP_MID, 0, 10); // At top, centered
            lv_obj_set_style_radius(set->lock_icon_btn, 2, 0);
            lv_obj_set_style_border_width(set->lock_icon_btn, 4, 0);
            lv_obj_set_style_border_color(set->lock_icon_btn, lv_color_hex(0x000000), 0);
            lv_obj_set_user_data(set->lock_icon_btn, (void*)i);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSED, NULL);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSING, NULL);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_RELEASED, NULL);
            set->lock_icon_label = lv_label_create(set->lock_icon_btn);
            lv_obj_set_style_text_font(set->lock_icon_label, &lv_font_montserrat_24, 0); // Smaller font for BS14 (24 instead of 36)
            lv_obj_set_style_text_color(set->lock_icon_label, lv_color_hex(0x000000), 0);
            lv_obj_center(set->lock_icon_label);
        } else if (is_portrait && !is_bs14) {
            // BS32 portrait: Lock button goes in tight_container first
            // Move lock button 10px to the right by adding left padding to container
            lv_obj_set_style_pad_left(tight_container, 15, 0); // 5 (from pad_all) + 10 = 15
            set->lock_icon_btn = lv_btn_create(tight_container);
            lv_obj_set_size(set->lock_icon_btn, 185, 45);
            lv_obj_set_style_radius(set->lock_icon_btn, 2, 0);
            lv_obj_set_style_border_width(set->lock_icon_btn, 3, 0);
            lv_obj_set_style_border_color(set->lock_icon_btn, lv_color_hex(0x000000), 0);
            lv_obj_set_user_data(set->lock_icon_btn, (void*)i);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSED, NULL);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSING, NULL);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_RELEASED, NULL);
            set->lock_icon_label = lv_label_create(set->lock_icon_btn);
            lv_obj_set_style_text_font(set->lock_icon_label, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(set->lock_icon_label, lv_color_hex(0x000000), 0);
            lv_obj_center(set->lock_icon_label);
        }

        // Open button
        set->btn_open = lv_btn_create(tight_container);
        if (is_bs14) {
            // BS14: Much larger buttons
            if (is_portrait) {
                lv_coord_t btn_height = (screen_height - 50) * 0.12 + 25; // 12% height + 25px taller (10px + 15px more)
                lv_obj_set_size(set->btn_open, LV_PCT(100), btn_height); // 100% of container width
            } else {
                lv_coord_t btn_width = (screen_width - 20) * 0.4 + 15; // 40% width + 15px wider
                lv_obj_set_size(set->btn_open, btn_width, (screen_height - 50) * 0.25 + 5); // 25% height + 5px taller
            }
        } else {
            // BS32: Original sizing
            if (is_portrait) {
                lv_obj_set_size(set->btn_open, 205, 68);
            } else {
                lv_obj_set_size(set->btn_open, 220, 68);
            }
        }
        lv_obj_set_style_border_width(set->btn_open, is_bs14 ? 5 : 3, 0);
        lv_obj_set_style_border_color(set->btn_open, lv_color_hex(0x000000), 0);
        lv_obj_set_style_radius(set->btn_open, 2, 0);
        lv_obj_set_user_data(set->btn_open, (void*)i);
        lv_obj_add_event_cb(set->btn_open, open_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_open = lv_label_create(set->btn_open);
        lv_label_set_text(label_open, "OPEN");
        lv_obj_set_style_text_color(label_open, lv_color_hex(0x000000), 0);
        lv_obj_set_style_text_font(label_open, is_bs14 ? &lv_font_montserrat_36 : &lv_font_montserrat_24, 0); // Even larger font for BS14
        lv_obj_center(label_open);

        // Close button
        set->btn_close = lv_btn_create(tight_container);
        if (is_bs14) {
            // BS14: Much larger buttons
            if (is_portrait) {
                lv_coord_t btn_height = (screen_height - 50) * 0.12 + 25; // 12% height + 25px taller (matches open)
                lv_obj_set_size(set->btn_close, LV_PCT(100), btn_height); // 100% of container width
            } else {
                lv_coord_t btn_width = (screen_width - 20) * 0.4 + 15; // 40% width + 15px wider (matches open)
                lv_obj_set_size(set->btn_close, btn_width, (screen_height - 50) * 0.25 + 5); // 25% height + 5px taller
            }
        } else {
            // BS32: Original sizing
            if (is_portrait) {
                lv_obj_set_size(set->btn_close, 205, 68);
            } else {
                lv_obj_set_size(set->btn_close, 220, 68);
            }
        }
        lv_obj_set_style_border_width(set->btn_close, is_bs14 ? 5 : 3, 0);
        lv_obj_set_style_border_color(set->btn_close, lv_color_hex(0x000000), 0);
        lv_obj_set_style_radius(set->btn_close, 2, 0);
        lv_obj_set_user_data(set->btn_close, (void*)i);
        lv_obj_add_event_cb(set->btn_close, close_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_close = lv_label_create(set->btn_close);
        lv_label_set_text(label_close, "CLOSE");
        lv_obj_set_style_text_color(label_close, lv_color_hex(0x000000), 0);
        lv_obj_set_style_text_font(label_close, is_bs14 ? &lv_font_montserrat_36 : &lv_font_montserrat_24, 0); // Even larger font for BS14
        lv_obj_center(label_close);
        
        // For BS32 portrait, move open/close buttons 15px to the left by adding right padding
        if (is_portrait && !is_bs14) {
            lv_obj_set_style_pad_right(tight_container, 15, 0);
        }
        
        // Prohibition symbol (circle slash) over close button - create overlay container
        lv_obj_t *prohibition_overlay = lv_obj_create(set->btn_close);
        lv_obj_set_size(prohibition_overlay, lv_obj_get_width(set->btn_close), lv_obj_get_height(set->btn_close));
        lv_obj_align(prohibition_overlay, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(prohibition_overlay, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(prohibition_overlay, LV_OPA_40, 0);
        lv_obj_set_style_radius(prohibition_overlay, 0, 0);
        lv_obj_add_flag(prohibition_overlay, LV_OBJ_FLAG_HIDDEN);
        
        // Create circle (O) label
        lv_obj_t *circle_label = lv_label_create(prohibition_overlay);
        lv_label_set_text(circle_label, "O");
        lv_obj_set_style_text_color(circle_label, lv_color_hex(0xFF0000), 0);
        lv_obj_set_style_text_font(circle_label, is_bs14 ? &lv_font_montserrat_48 : &lv_font_montserrat_40, 0); // Larger for BS14
        lv_obj_center(circle_label);
        
        // Create slash (/) label
        lv_obj_t *slash_label = lv_label_create(prohibition_overlay);
        lv_label_set_text(slash_label, "/");
        lv_obj_set_style_text_color(slash_label, lv_color_hex(0xFF0000), 0);
        lv_obj_set_style_text_font(slash_label, is_bs14 ? &lv_font_montserrat_48 : &lv_font_montserrat_40, 0); // Larger for BS14
        lv_obj_center(slash_label);

        // Lock icon button (landscape mode only - portrait created above)
        if (!is_portrait) {
            set->lock_icon_btn = lv_btn_create(set->container);
            if (is_bs14) {
                // Match the width and height of open/close buttons
                lv_coord_t btn_width = (screen_width - 20) * 0.4 + 15; // 40% width + 15px wider (matches open/close)
                lv_obj_set_size(set->lock_icon_btn, btn_width, (screen_height - 50) * 0.20); // Same height as open/close buttons
            } else {
                lv_obj_set_size(set->lock_icon_btn, 160, 45);
            }
            lv_obj_set_style_radius(set->lock_icon_btn, 2, 0);
            lv_obj_set_style_border_width(set->lock_icon_btn, is_bs14 ? 5 : 3, 0);
            lv_obj_set_style_border_color(set->lock_icon_btn, lv_color_hex(0x000000), 0);
            lv_obj_set_user_data(set->lock_icon_btn, (void*)i);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSED, NULL);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSING, NULL);
            lv_obj_add_event_cb(set->lock_icon_btn, lock_icon_event_cb, LV_EVENT_RELEASED, NULL);
            if (is_bs14) {
                // Center lock button above open/close buttons (which are in tight_container)
                // Position it above the tight_container
                lv_obj_align_to(set->lock_icon_btn, tight_container, LV_ALIGN_OUT_TOP_MID, 0, -10); // 10px gap above buttons
            } else {
                lv_obj_align(set->lock_icon_btn, LV_ALIGN_TOP_RIGHT, -4, 15); // Original position for BS32
            }
            set->lock_icon_label = lv_label_create(set->lock_icon_btn);
            lv_obj_set_style_text_font(set->lock_icon_label, is_bs14 ? &lv_font_montserrat_24 : &lv_font_montserrat_14, 0); // Smaller font for BS14 (24 instead of 36)
            lv_obj_set_style_text_color(set->lock_icon_label, lv_color_hex(0x000000), 0);
            lv_obj_center(set->lock_icon_label);
            
            // Position number label directly to the left of lock button in landscape
            lv_obj_t *num_label = lv_obj_get_child(set->container, 0); // Number label is first child
            lv_obj_align_to(num_label, set->lock_icon_btn, LV_ALIGN_OUT_LEFT_MID, -10, 0);
        }
        
        // In portrait mode, position number label at the top center of the control set
        if (is_portrait) {
            lv_obj_t *num_label = lv_obj_get_child(set->container, 0); // Number label is first child
            lv_obj_add_flag(num_label, LV_OBJ_FLAG_FLOATING); // Remove from flexbox flow
            lv_obj_align(num_label, LV_ALIGN_TOP_MID, 20, 10); // Top center, moved 20px right, 10px from top
        }
        
        // Update styles based on current state
        update_button_styles(i);
        update_lock_icon(i);
    }
}

void create_mode_selection_menu() {
    Serial.println("Creating mode selection menu");
    
    // Set background FIRST to prevent flash
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    
    // Clear screen after background is set
    lv_obj_clean(lv_scr_act());
    
    // Title - moved down to account for 50px top bar
    lv_obj_t *title = lv_label_create(lv_scr_act());
    lv_label_set_text(title, "Select Device Mode");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0); // Smaller font
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 70); // Moved up a bit
    
    // Helper for button style
    auto set_mode_btn_style = [](lv_obj_t* btn, bool selected) {
        lv_color_t bg = selected ? lv_color_hex(0x9C27B0) : lv_color_hex(0x444444); // Purple or gray
        lv_obj_set_style_bg_color(btn, bg, 0);
        lv_obj_set_style_border_width(btn, 3, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_radius(btn, 8, 0);
    };

    int btn_w = 240, btn_h = 60;
    int y0 = -90, ystep = 70;

    // Create all mode buttons and labels
    lv_obj_t *btns[4];
    lv_obj_t *labels[4];
    const char* names[4] = {"BS14", "BS32", "Sub-48", "Sub-125"};
    static DeviceMode modes[4] = {MODE_BS14, MODE_BS32, MODE_SUB_48, MODE_SUB_125};
    static int selectedModeIdx = -1;
    selectedModeIdx = -1;
    for (int i = 0; i < 4; ++i) {
        btns[i] = lv_btn_create(lv_scr_act());
        lv_obj_set_size(btns[i], btn_w, btn_h);
        lv_obj_align(btns[i], LV_ALIGN_CENTER, 0, y0 + i * ystep);
        bool isSelected = (currentMode == modes[i]);
        set_mode_btn_style(btns[i], isSelected);
        if (isSelected) selectedModeIdx = i;
        labels[i] = lv_label_create(btns[i]);
        lv_label_set_text(labels[i], names[i]);
        lv_obj_set_style_text_font(labels[i], &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(labels[i], lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(labels[i]);
    }

    // Store button pointers and mode info for callback
    struct ModeBtnInfo {
        lv_obj_t* btns[4];
        DeviceMode modes[4];
    };
    static ModeBtnInfo modeBtnInfo;
    for (int i = 0; i < 4; ++i) {
        modeBtnInfo.btns[i] = btns[i];
        modeBtnInfo.modes[i] = modes[i];
        lv_obj_set_user_data(btns[i], (void*)(intptr_t)i);
    }
    // Static callback for mode buttons
    auto mode_btn_cb = [](lv_event_t *e) {
        int idx = (int)(intptr_t)lv_obj_get_user_data(lv_event_get_target(e));
        for (int j = 0; j < 4; ++j) {
            lv_color_t bg = (j == idx) ? lv_color_hex(0x9C27B0) : lv_color_hex(0x444444);
            lv_obj_set_style_bg_color(modeBtnInfo.btns[j], bg, 0);
        }
        selectedModeIdx = idx;
    };
    for (int i = 0; i < 4; ++i) {
        lv_obj_add_event_cb(btns[i], mode_btn_cb, LV_EVENT_CLICKED, NULL);
    }

    // Add a close button at the bottom (blue)
    lv_obj_t *btn_close = lv_btn_create(lv_scr_act());
    lv_obj_set_size(btn_close, 180, 50);
    lv_obj_align(btn_close, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x2196F3), 0); // Blue
    lv_obj_set_style_radius(btn_close, 8, 0);
    lv_obj_set_style_border_width(btn_close, 2, 0);
    lv_obj_set_style_border_color(btn_close, lv_color_hex(0xFFFFFF), 0);
    lv_obj_t *label_close = lv_label_create(btn_close);
    lv_label_set_text(label_close, "CLOSE");
    lv_obj_set_style_text_font(label_close, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(label_close, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(label_close);
    lv_obj_add_event_cb(btn_close, [](lv_event_t *e) {
        if (selectedModeIdx >= 0 && modes[selectedModeIdx] != currentMode) {
            change_device_mode_and_recreate_ui(modes[selectedModeIdx]);
        } else {
            create_ui();
        }
    }, LV_EVENT_CLICKED, NULL);

    // Store reference to menu container (we'll use the screen itself)
    mode_selection_menu = lv_scr_act();
}

void create_ui() {
    // Reset ALL sets to open state, switch up, unlocked (reset all breakers when mode changes)
    // Reset all sets up to MAX_SETS to ensure clean state when switching modes
    for (int i = 0; i < MAX_SETS; ++i) {
        sets[i].breakerstate = true;  // Open (green)
        sets[i].switchToggled = true; // Switch up
        sets[i].locked = false;       // Unlocked
        sets[i].lock_press_start = 0;  // Reset lock press timer
        sets[i].last_switch_toggle = 0; // Reset switch toggle timer
    }
    
    // Set the screen background to black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
    updateLayout(currentRotation);
    
    // Create top bar for settings and rotate buttons
    create_top_bar();
}

// --- Event callbacks and helpers ---

void switch_toggled_cb(lv_event_t *e) {
    int idx = (int)lv_obj_get_user_data(lv_event_get_target(e));
    ControlSet *set = &sets[idx];
    unsigned long currentTime = millis();
    if (currentTime - set->last_switch_toggle < 300) return;
    set->last_switch_toggle = currentTime;
    set->switchToggled = !set->switchToggled;
    
    // If breaker is closed and switch is toggled down, automatically open the breaker
    if (!set->breakerstate && !set->switchToggled) {
        set->breakerstate = true;  // Open the breaker
        Serial.printf("Breaker %d automatically opened due to switch toggle down\n", idx + 1);
        send_set_state_to_flutter(idx, "breaker", set->breakerstate);
        update_button_styles(idx); // Update button visuals immediately
    }
    
    // Move the knob up or down based on state
    bool is_portrait_cb = lv_obj_get_width(lv_scr_act()) < lv_obj_get_height(lv_scr_act());
    bool is_bs14_cb = (NUM_SETS == 1);
    
    if (is_bs14_cb) {
        // BS14: Set knob position as very last step
        lv_obj_t *vertical_switch_container = lv_obj_get_parent(set->switch_69);
        lv_coord_t knob_w = lv_obj_get_width(set->switch_69);
        lv_coord_t knob_h = lv_obj_get_height(set->switch_69);
        lv_coord_t parent_w = lv_obj_get_width(vertical_switch_container);
        lv_coord_t parent_h = lv_obj_get_height(vertical_switch_container);
        lv_coord_t x = (parent_w - knob_w) / 2 - 10;
        lv_coord_t y = (parent_h - knob_h) / 2 - 50;
        lv_obj_set_pos(set->switch_69, x, y);
    } else {
        // BS32: Original hardcoded positions
        if (set->switchToggled) {
            if (is_portrait_cb) {
                lv_obj_set_pos(set->switch_69, 1, 3); // Right 2px in portrait
            } else {
                lv_obj_set_pos(set->switch_69, -1, 3); // Original position in landscape
            }
        } else {
            if (is_portrait_cb) {
                lv_obj_set_pos(set->switch_69, 1, 58); // Right 2px in portrait
            } else {
                lv_obj_set_pos(set->switch_69, -1, 58); // Original position in landscape
            }
        }
    }
    lv_obj_set_style_bg_color(set->switch_69, lv_color_hex(0x2196F3), 0);
    update_button_styles(idx);
    send_set_state_to_flutter(idx, "switch", set->switchToggled);
    send_status_to_flutter(); // Send complete status update
}

void open_btn_cb(lv_event_t *e) {
    int idx = (int)lv_obj_get_user_data(lv_event_get_target(e));
    ControlSet *set = &sets[idx];
    if (set->locked) return;
    set->breakerstate = true;
    update_button_styles(idx);
    send_set_state_to_flutter(idx, "breaker", set->breakerstate);
    send_status_to_flutter(); // Send complete status update
}

void close_btn_cb(lv_event_t *e) {
    int idx = (int)lv_obj_get_user_data(lv_event_get_target(e));
    ControlSet *set = &sets[idx];
    if (set->locked) return;
    if (set->switchToggled && set->breakerstate) {
        set->breakerstate = false;
        update_button_styles(idx);
        send_set_state_to_flutter(idx, "breaker", set->breakerstate);
        send_status_to_flutter(); // Send complete status update
    }
}


void lock_icon_event_cb(lv_event_t *e) {
    int idx = (int)lv_obj_get_user_data(lv_event_get_target(e));
    ControlSet *set = &sets[idx];
    uint32_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        set->lock_press_start = millis();
        if (set->locked) {
            lv_label_set_text(set->lock_icon_label, "HOLD TO UNLOCK");
        } else {
            lv_label_set_text(set->lock_icon_label, "HOLD TO LOCK");
        }
    } else if (code == LV_EVENT_PRESSING) {
        if (set->lock_press_start && (millis() - set->lock_press_start > 400)) {
            set->lock_press_start = 0;
            set->locked = !set->locked;
            update_lock_icon(idx);
            update_button_styles(idx);
            send_set_state_to_flutter(idx, "locked", set->locked);
            send_status_to_flutter(); // Send complete status update
        }
    } else if (code == LV_EVENT_RELEASED) {
        update_lock_icon(idx);
        set->lock_press_start = 0;
    }
}

void update_lock_icon(int idx) {
    ControlSet *set = &sets[idx];
    if (!set->lock_icon_label || !set->lock_icon_btn) return;
    if (set->locked) {
        lv_label_set_text(set->lock_icon_label, "HOLD TO UNLOCK");
        lv_obj_set_style_bg_color(set->lock_icon_btn, lv_color_hex(0xFF9800), 0);
    } else {
        lv_label_set_text(set->lock_icon_label, "HOLD TO LOCK");
        lv_obj_set_style_bg_color(set->lock_icon_btn, lv_color_hex(0x1976D2), 0);
    }
}

void update_button_styles(int idx) {
    ControlSet *set = &sets[idx];
    if (!set->btn_open || !set->btn_close || !set->container || !set->switch_69) return;
    
    // Show/hide prohibition symbol based on switch state
    // Find the prohibition overlay (first child of close button)
    if (set->btn_close) {
        lv_obj_t *prohibition_overlay = lv_obj_get_child(set->btn_close, 1); // Second child (first is the label)
        if (prohibition_overlay) {
            if (!set->switchToggled && set->breakerstate && !set->locked) {
                // Switch is down, breaker open, and unlocked - show prohibition symbol
                lv_obj_clear_flag(prohibition_overlay, LV_OBJ_FLAG_HIDDEN);
            } else {
                // Hide prohibition symbol
                lv_obj_add_flag(prohibition_overlay, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }


    // Set container background color: if switch is down, always green; otherwise use breaker state
    if (!set->switchToggled) {
        // Switch is down - background should always be green
        lv_obj_set_style_bg_color(set->container, lv_color_hex(0x00AA00), 0); // Green
        lv_obj_set_style_bg_color(set->btn_open, lv_color_hex(0x00AA00), 0); // Green open button
        lv_obj_set_style_bg_color(set->btn_close, lv_color_hex(0x8B0000), 0); // Dark brown close button
    } else if (set->breakerstate) {
        // Switch is up and breaker is open
        lv_obj_set_style_bg_color(set->container, lv_color_hex(0x00AA00), 0); // Green for open
        lv_obj_set_style_bg_color(set->btn_open, lv_color_hex(0x00AA00), 0); // Green open button
        lv_obj_set_style_bg_color(set->btn_close, lv_color_hex(0x8B0000), 0); // Dark brown close button
    } else {
        // Switch is up and breaker is closed
        lv_obj_set_style_bg_color(set->container, lv_color_hex(0xAA0000), 0); // Red for closed
        lv_obj_set_style_bg_color(set->btn_open, lv_color_hex(0x005500), 0); // Dark green open button
        lv_obj_set_style_bg_color(set->btn_close, lv_color_hex(0xAA0000), 0); // Red close button
    }

    // Move the knob up or down based on state (in case state changes outside of switch_toggled_cb)
    bool is_portrait_ubs = lv_obj_get_width(lv_scr_act()) < lv_obj_get_height(lv_scr_act());
    bool is_bs14_ubs = (NUM_SETS == 1);
    
    if (is_bs14_ubs) {
        // BS14: Use same positioning logic as toggle callback with proper centering
        lv_obj_t *vertical_switch_container = lv_obj_get_parent(set->switch_69);
        lv_coord_t vswitch_w = lv_obj_get_width(vertical_switch_container);
        lv_coord_t vswitch_h = lv_obj_get_height(vertical_switch_container);
        lv_coord_t knob_w = lv_obj_get_width(set->switch_69);
        lv_coord_t knob_h = lv_obj_get_height(set->switch_69);
        
        // Center horizontally: account for container padding (12px)
        lv_coord_t padding_h = 12; // Padding from container
        lv_coord_t available_w = vswitch_w - (padding_h * 2);
        lv_coord_t x_pos = padding_h + (available_w - knob_w) / 2; // Centered horizontally
        
        if (set->switchToggled) {
            // Top position: padding + small gap
            lv_obj_set_pos(set->switch_69, x_pos, padding_h + 5); // Centered horizontally, near top
        } else {
            // Bottom position: padding from bottom
            lv_coord_t available_h = vswitch_h - (padding_h * 2);
            lv_obj_set_pos(set->switch_69, x_pos, padding_h + available_h - knob_h - 5); // Centered horizontally, near bottom
        }
    } else {
        // BS32: Original hardcoded positions
        if (set->switchToggled) {
            if (is_portrait_ubs) {
                lv_obj_set_pos(set->switch_69, 1, 3); // Right 2px in portrait
            } else {
                lv_obj_set_pos(set->switch_69, -1, 3); // Original position in landscape
            }
        } else {
            if (is_portrait_ubs) {
                lv_obj_set_pos(set->switch_69, 1, 58); // Right 2px in portrait
            } else {
                lv_obj_set_pos(set->switch_69, -1, 58); // Original position in landscape
            }
        }
    }

    // Enable/disable buttons and switch
    if (set->locked) {
        lv_obj_add_state(set->btn_open, LV_STATE_DISABLED);
        lv_obj_add_state(set->btn_close, LV_STATE_DISABLED);
        lv_obj_add_state(set->switch_69, LV_STATE_DISABLED);
    } else {
        lv_obj_clear_state(set->switch_69, LV_STATE_DISABLED);
        lv_obj_clear_state(set->btn_open, LV_STATE_DISABLED); // Always enable open button when unlocked
        if (set->breakerstate) {
            if (!set->switchToggled) {
                lv_obj_add_state(set->btn_close, LV_STATE_DISABLED);
            } else {
                lv_obj_clear_state(set->btn_close, LV_STATE_DISABLED);
            }
        } else {
            lv_obj_clear_state(set->btn_close, LV_STATE_DISABLED);
        }
    }
}

// Settings modal and callbacks
// Update sense button colors based on current selection
void update_sense_buttons() {
    if (btn_sense_a && btn_sense_b) {
        if (senseA_selected) {
            lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0x00AA00), 0); // Green
            lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0xCCCCCC), 0); // Gray
        } else {
            lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0xCCCCCC), 0); // Gray
            lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0x00AA00), 0); // Green
        }
    }
}

void sense_a_btn_cb(lv_event_t *e) {
    Serial.println("Sense A selected");
    senseA_selected = true;
    save_sense_selection(); // Save to persistent storage
    update_sense_buttons(); // Update button colors
    
    // Send BLE notification for sense selection change
    send_status_to_flutter();
}

void sense_b_btn_cb(lv_event_t *e) {
    Serial.println("Sense B selected");
    senseA_selected = false;
    save_sense_selection(); // Save to persistent storage
    update_sense_buttons(); // Update button colors
    
    // Send BLE notification for sense selection change
    send_status_to_flutter();
}

// Update Bluetooth status in top bar
void update_top_bar_bt_status() {
    if (!top_bar_bt_status) {
        return; // UI element not created yet
    }
    
    if (deviceConnected) {
        lv_label_set_text(top_bar_bt_status, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_color(top_bar_bt_status, lv_color_hex(0x00FF00), 0); // Green
    } else {
        lv_label_set_text(top_bar_bt_status, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_color(top_bar_bt_status, lv_color_hex(0xFF0000), 0); // Red
    }
}

// Update Bluetooth UI in settings modal
void update_bluetooth_ui() {
    if (!bt_status_label || !btn_disconnect) {
        return; // UI elements not created yet
    }
    
    if (deviceConnected) {
        lv_label_set_text(bt_status_label, "Status: Connected");
        lv_obj_set_style_text_color(bt_status_label, lv_color_hex(0x000000), 0); // Black
        
        // Enable disconnect button
        lv_obj_clear_state(btn_disconnect, LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(btn_disconnect, lv_color_hex(0xFF4444), 0); // Red
    } else {
        lv_label_set_text(bt_status_label, "Status: Not Connected");
        lv_obj_set_style_text_color(bt_status_label, lv_color_hex(0x000000), 0); // Black
        
        // Disable disconnect button
        lv_obj_add_state(btn_disconnect, LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(btn_disconnect, lv_color_hex(0xCCCCCC), 0); // Gray
    }
}

// Disconnect button callback
void disconnect_btn_cb(lv_event_t *e) {
    if (!deviceConnected || !pServer) {
        Serial.println("[BLE] Cannot disconnect - not connected");
        return;
    }
    
    Serial.println("[BLE] Disconnecting client...");
    
    // Get the connection ID and disconnect
    uint16_t connId = pServer->getConnId();
    if (connId >= 0) {
        pServer->disconnect(connId);
        Serial.printf("[BLE] Disconnecting client with ID: %d\n", connId);
    } else {
        // Fallback: try to disconnect with ID 0
        pServer->disconnect(0);
        Serial.println("[BLE] Disconnecting with default ID 0");
    }
    
    // The onDisconnect callback will be called automatically and update deviceConnected
    // Force update UI immediately
    deviceConnected = false;
    update_bluetooth_ui();
    
    Serial.println("[BLE] Client disconnected");
}

void close_settings_cb(lv_event_t *e) {
    Serial.println("Closing settings");
    if (settings_modal) {
        lv_obj_t *backdrop = (lv_obj_t *)lv_obj_get_user_data(settings_modal);
        if (backdrop) {
            lv_obj_add_flag(backdrop, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_add_flag(settings_modal, LV_OBJ_FLAG_HIDDEN);
    }
}

void settings_btn_cb(lv_event_t *e) {
    Serial.println("=== SETTINGS BUTTON PRESSED ===");
    
    // Create modal dialog if it doesn't exist
    if (!settings_modal) {
        // Create backdrop overlay on top layer
        lv_obj_t *backdrop = lv_obj_create(lv_layer_top());
        lv_obj_set_size(backdrop, LV_PCT(100), LV_PCT(100));
        lv_obj_align(backdrop, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(backdrop, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(backdrop, LV_OPA_50, 0);
        lv_obj_set_style_border_width(backdrop, 0, 0);
        lv_obj_clear_flag(backdrop, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_user_data(backdrop, (void*)0xDEADBEEF); // Mark as backdrop
        
        // Create modal on top layer
        settings_modal = lv_obj_create(lv_layer_top());
        lv_obj_set_size(settings_modal, 450, 450); // Reduced height since close button is now positioned above
        lv_obj_align(settings_modal, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(settings_modal, lv_color_hex(0xDDDDDD), 0);
        lv_obj_set_style_border_width(settings_modal, 3, 0);
        lv_obj_set_style_border_color(settings_modal, lv_color_hex(0x000000), 0);
        lv_obj_set_style_pad_all(settings_modal, 15, 0); // Reduced padding
        lv_obj_clear_flag(settings_modal, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_move_foreground(settings_modal);
        // Store backdrop reference in modal user data
        lv_obj_set_user_data(settings_modal, backdrop);
        
        // Main Title
        lv_obj_t *title = lv_label_create(settings_modal);
        lv_label_set_text(title, "SETTINGS");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(0x000000), 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
        
        // ========== DEVICE MODE SECTION (TOP, LARGEST) ==========
        // Device Mode section title
        lv_obj_t *mode_title = lv_label_create(settings_modal);
        lv_label_set_text(mode_title, "DEVICE MODE");
        lv_obj_set_style_text_font(mode_title, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(mode_title, lv_color_hex(0x000000), 0);
        lv_obj_align(mode_title, LV_ALIGN_TOP_MID, 0, 35);
        
        // Current mode label
        current_mode_label = lv_label_create(settings_modal);
        char mode_text[50];
        const char* modeName;
        if (currentMode == MODE_BS14) {
            modeName = "BS14";
        } else if (currentMode == MODE_BS32) {
            modeName = "BS32";
        } else if (currentMode == MODE_SUB_48) {
            modeName = "SUB48";
        } else if (currentMode == MODE_SUB_125) {
            modeName = "SUB125";
        } else {
            modeName = "Unknown";
        }
        snprintf(mode_text, sizeof(mode_text), "Current: %s", modeName);
        lv_label_set_text(current_mode_label, mode_text);
        lv_obj_set_style_text_font(current_mode_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(current_mode_label, lv_color_hex(0x000000), 0);
        lv_obj_align(current_mode_label, LV_ALIGN_TOP_MID, 0, 55);
        
        // Change Mode button (LARGEST)
        lv_obj_t *btn_change_mode = lv_btn_create(settings_modal);
        lv_obj_set_size(btn_change_mode, 400, 80); // Largest button
        lv_obj_align(btn_change_mode, LV_ALIGN_TOP_MID, 0, 80);
        lv_obj_set_style_bg_color(btn_change_mode, lv_color_hex(0x9C27B0), 0); // Purple
        lv_obj_set_style_border_width(btn_change_mode, 4, 0);
        lv_obj_set_style_border_color(btn_change_mode, lv_color_hex(0x000000), 0);
        lv_obj_set_style_radius(btn_change_mode, 5, 0);
        lv_obj_add_event_cb(btn_change_mode, [](lv_event_t *e) {
            // Close settings modal first
            if (settings_modal) {
                lv_obj_t *backdrop = (lv_obj_t *)lv_obj_get_user_data(settings_modal);
                if (backdrop) {
                    lv_obj_add_flag(backdrop, LV_OBJ_FLAG_HIDDEN);
                }
                lv_obj_add_flag(settings_modal, LV_OBJ_FLAG_HIDDEN);
            }
            // Show mode selection menu
            lvgl_port_lock(-1);
            create_mode_selection_menu();
            lvgl_port_unlock();
        }, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_change_mode = lv_label_create(btn_change_mode);
        lv_label_set_text(label_change_mode, "CHANGE MODE");
        lv_obj_set_style_text_font(label_change_mode, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(label_change_mode, lv_color_hex(0x000000), 0);
        lv_obj_center(label_change_mode);
        
        // ========== SENSE SELECTION SECTION (SMALLER) ==========
        // Sense Selection section title
        lv_obj_t *sense_title = lv_label_create(settings_modal);
        lv_label_set_text(sense_title, "SENSE SELECTION");
        lv_obj_set_style_text_font(sense_title, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(sense_title, lv_color_hex(0x000000), 0);
        lv_obj_align(sense_title, LV_ALIGN_TOP_MID, 0, 175);
        
        // Sense A button (smaller)
        btn_sense_a = lv_btn_create(settings_modal);
        lv_obj_set_size(btn_sense_a, 190, 45); // Smaller, side by side
        lv_obj_align(btn_sense_a, LV_ALIGN_TOP_LEFT, 15, 200);
        lv_obj_set_style_border_width(btn_sense_a, 3, 0);
        lv_obj_set_style_border_color(btn_sense_a, lv_color_hex(0x000000), 0);
        // Set initial color based on current selection
        lv_obj_set_style_bg_color(btn_sense_a, senseA_selected ? lv_color_hex(0x00AA00) : lv_color_hex(0xCCCCCC), 0);
        lv_obj_add_event_cb(btn_sense_a, sense_a_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_a = lv_label_create(btn_sense_a);
        lv_label_set_text(label_a, "SENSE A");
        lv_obj_set_style_text_font(label_a, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(label_a, lv_color_hex(0x000000), 0);
        lv_obj_center(label_a);
        
        // Sense B button (smaller, side by side)
        btn_sense_b = lv_btn_create(settings_modal);
        lv_obj_set_size(btn_sense_b, 190, 45); // Smaller, side by side
        lv_obj_align(btn_sense_b, LV_ALIGN_TOP_RIGHT, -15, 200);
        lv_obj_set_style_border_width(btn_sense_b, 3, 0);
        lv_obj_set_style_border_color(btn_sense_b, lv_color_hex(0x000000), 0);
        // Set initial color based on current selection
        lv_obj_set_style_bg_color(btn_sense_b, senseA_selected ? lv_color_hex(0xCCCCCC) : lv_color_hex(0x00AA00), 0);
        lv_obj_add_event_cb(btn_sense_b, sense_b_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_b = lv_label_create(btn_sense_b);
        lv_label_set_text(label_b, "SENSE B");
        lv_obj_set_style_text_font(label_b, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(label_b, lv_color_hex(0x000000), 0);
        lv_obj_center(label_b);
        
        // ========== BLUETOOTH SECTION ==========
        // Bluetooth section title
        lv_obj_t *bt_title = lv_label_create(settings_modal);
        lv_label_set_text(bt_title, "BLUETOOTH");
        lv_obj_set_style_text_font(bt_title, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(bt_title, lv_color_hex(0x000000), 0);
        lv_obj_align(bt_title, LV_ALIGN_TOP_MID, 0, 260);
        
        // Bluetooth status label
        bt_status_label = lv_label_create(settings_modal);
        lv_obj_set_style_text_font(bt_status_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(bt_status_label, lv_color_hex(0x000000), 0);
        lv_obj_align(bt_status_label, LV_ALIGN_TOP_MID, 0, 280);
        
        // Disconnect button (only enabled when connected)
        btn_disconnect = lv_btn_create(settings_modal);
        lv_obj_set_size(btn_disconnect, 200, 45);
        lv_obj_align(btn_disconnect, LV_ALIGN_TOP_MID, 0, 300);
        lv_obj_set_style_border_width(btn_disconnect, 3, 0);
        lv_obj_set_style_border_color(btn_disconnect, lv_color_hex(0x000000), 0);
        lv_obj_add_event_cb(btn_disconnect, disconnect_btn_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_disconnect = lv_label_create(btn_disconnect);
        lv_label_set_text(label_disconnect, "DISCONNECT");
        lv_obj_set_style_text_font(label_disconnect, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(label_disconnect, lv_color_hex(0x000000), 0);
        lv_obj_center(label_disconnect);
        
        // ========== CLOSE BUTTON ==========
        // Save and Close button - positioned directly below disconnect button
        lv_obj_t *btn_close_settings = lv_btn_create(settings_modal);
        lv_obj_set_size(btn_close_settings, 400, 50);
        lv_obj_align(btn_close_settings, LV_ALIGN_TOP_MID, 0, 360); // Positioned 60px below disconnect button (300 + 45 + 15)
        lv_obj_set_style_border_width(btn_close_settings, 3, 0);
        lv_obj_set_style_border_color(btn_close_settings, lv_color_hex(0x000000), 0);
        lv_obj_add_event_cb(btn_close_settings, close_settings_cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t *label_close = lv_label_create(btn_close_settings);
        lv_label_set_text(label_close, "CLOSE");
        lv_obj_set_style_text_font(label_close, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(label_close, lv_color_hex(0x000000), 0);
        lv_obj_center(label_close);
    }
    
    // Update Bluetooth UI with current status
    update_bluetooth_ui();
    
    // Update sense button colors based on current selection
    update_sense_buttons();
    
    // Update current mode label if it exists
    if (current_mode_label) {
        char mode_text[50];
        snprintf(mode_text, sizeof(mode_text), "Current: %s", 
                currentMode == MODE_BS14 ? "BS14" : "BS32");
        lv_label_set_text(current_mode_label, mode_text);
    }
    
    // Show modal and backdrop
    lv_obj_t *backdrop = (lv_obj_t *)lv_obj_get_user_data(settings_modal);
    if (backdrop) {
        lv_obj_clear_flag(backdrop, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_flag(settings_modal, LV_OBJ_FLAG_HIDDEN);
    Serial.println("Settings modal opened");
}
