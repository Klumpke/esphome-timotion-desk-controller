#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#ifdef USE_ESP32
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gattc.html
#include <esp_gattc_api.h>

namespace esphome {
namespace timotion_desk_controller {

namespace espbt = esphome::esp32_ble_tracker;

using namespace ble_client;

espbt::ESPBTUUID uuid128_from_string(std::string value);

class TimotionDeskControllerComponent : public Component, public number::Number, public BLEClientNode {
 public:
  TimotionDeskControllerComponent() : Component(){};

  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void dump_config() override;

  void loop() override;

  void use_only_up_down_command(bool use_only_up_down_command) { use_only_up_down_command_ = use_only_up_down_command; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void control(float value) override;

 private:
  bool use_only_up_down_command_ = false;

  espbt::ESPBTUUID output_service_uuid_ = uuid128_from_string("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
  espbt::ESPBTUUID output_char_uuid_ = uuid128_from_string("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
  uint16_t output_handle_{0};

  espbt::ESPBTUUID input_service_uuid_ = uuid128_from_string("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
  espbt::ESPBTUUID input_char_uuid_ = uuid128_from_string("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
  uint16_t input_handle_{0};

  espbt::ESPBTUUID control_service_uuid_ = uuid128_from_string("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
  espbt::ESPBTUUID control_char_uuid_ = uuid128_from_string("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
  uint16_t control_handle_{0};
  uint16_t lastHeight{0};
  uint16_t lastSpeed{0};

  enum class DeskDirection : uint8_t {
    IDLE = 0,
    UP = 1,
    DOWN = 2,
  };

  bool controlled_ = false;
  float target_height_{0.0f};

  bool notify_disable_ = true;
  int not_moving_loop_ = 0;
  float last_move_height_{0.0f};
  uint8_t stalled_loops_{0};
  DeskDirection current_direction_{DeskDirection::IDLE};

  void write_value_(uint16_t handle, uint64_t value);
  void read_value_(uint16_t handle);
  void publish_desk_state_(uint8_t *value, uint16_t value_len);
  void move_desk_();
  bool is_at_target_() const;
  void start_move_torwards_();
  void move_torwards_();
  void stop_move_();
};
}  // namespace timotion_desk_controller
}  // namespace esphome

#endif
