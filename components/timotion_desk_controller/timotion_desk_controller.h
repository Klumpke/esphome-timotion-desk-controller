#pragma once

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
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

class TimotionDeskControllerComponent : public Component, public cover::Cover, public BLEClientNode {
 public:
  TimotionDeskControllerComponent() : Component(){};

  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void dump_config() override;

  void loop() override;

  void use_only_up_down_command(bool use_only_up_down_command) { use_only_up_down_command_ = use_only_up_down_command; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

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

  bool controlled_ = false;
  float position_target_{0.0f};

  bool notify_disable_ = true;
  int not_moving_loop_ = 0;
  float last_move_position_{0.0f};
  uint8_t stalled_loops_{0};

  void write_value_(uint16_t handle, uint64_t value);
  void read_value_(uint16_t handle);
  void publish_cover_state_(uint8_t *value, uint16_t value_len);
  void move_desk_();
  bool is_at_target_() const;
  void start_move_torwards_();
  void move_torwards_();
  void stop_move_();
};
}  // namespace timotion_desk_controller
}  // namespace esphome

#endif
