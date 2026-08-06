# Headset_BLE_Controller
It's a project based on XIAO ESP32S3 and MPU6050. It detects the movements of your head and transform them into BLE HID signals to control your computers or cellphones.
## Overall Framework
Based on esp-idf.
## Components
### Gesture Detection
### Configuration Software
### BLE HID
### Touch Sensor
Single-channel touch sensor input ported from
`reference/touch_sens_basic/`. `touch_sensor_init(<chan_id>)` brings the
default-V2 sample config + filter up on the given channel, runs the
three-shot baseline scan to set a relative active threshold, and starts
continuous scanning. The touch sensor's `on_active` / `on_inactive` events
are forwarded to a small worker that calls `esp_hidd_send_mouse_value()`
directly, so a touch is a press-and-hold mouse left button and a release
is a release. Default channel in `main.c` is T2 (GPIO2).
