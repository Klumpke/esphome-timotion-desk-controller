#include "timotion_desk_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <array>
#include <cmath>
#include <string>

namespace esphome {
namespace timotion_desk_controller {

static const char *TAG = "timotion_desk_controller";

static const float DESK_MIN_HEIGHT = 65;
static const float DESK_MAX_HEIGHT = 130;
static const float HEIGHT_EPSILON = 0.5f;
static const uint8_t STALLED_LOOPS_LIMIT = 10;

void TimotionDeskControllerComponent::loop() {}

void TimotionDeskControllerComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Timotion Desk Controller...");
  this->set_interval("update_desk", 200, [this]() { this->move_desk_(); });
}

void TimotionDeskControllerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Timotion Desk Controller:");
  ESP_LOGCONFIG(TAG, "  MAC address        : %s", this->parent()->address_str());
  ESP_LOGCONFIG(TAG, "  Notifications      : %s", this->notify_disable_ ? "disable" : "enable");
  ESP_LOGCONFIG(TAG, "  Min height         : %.1f cm", DESK_MIN_HEIGHT);
  ESP_LOGCONFIG(TAG, "  Max height         : %.1f cm", DESK_MAX_HEIGHT);
}

void TimotionDeskControllerComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                                        esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_WRITE_CHAR_EVT: {
      if (param->write.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error writing char at handle %d, status=%d", param->write.handle, param->write.status);
      }
      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "[%s] Connected successfully!", this->get_name().c_str());
        break;
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->get_name().c_str());
      this->status_set_warning();
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Look for output handle
      this->output_handle_ = 0;
      auto chr_output = this->parent()->get_characteristic(this->output_service_uuid_, this->output_char_uuid_);
      if (chr_output == nullptr) {
        this->status_set_warning();
        std::array<char, 37> output_service_uuid_buf{};
        std::array<char, 37> output_char_uuid_buf{};
        ESP_LOGW(TAG, "No characteristic found at service %s char %s",
                 this->output_service_uuid_.to_str(output_service_uuid_buf),
                 this->output_char_uuid_.to_str(output_char_uuid_buf));
        break;
      }
      this->output_handle_ = chr_output->handle;

      // Register for notification
      auto status_notify = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                             this->parent()->get_remote_bda(), this->output_handle_);
      if (status_notify) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status_notify);
      }

      // Look for input handle
      this->input_handle_ = 0;
      auto chr_input = this->parent()->get_characteristic(this->input_service_uuid_, this->input_char_uuid_);
      if (chr_input == nullptr) {
        this->status_set_warning();
        std::array<char, 37> input_service_uuid_buf{};
        std::array<char, 37> input_char_uuid_buf{};
        ESP_LOGW(TAG, "No characteristic found at service %s char %s",
                 this->input_service_uuid_.to_str(input_service_uuid_buf),
                 this->input_char_uuid_.to_str(input_char_uuid_buf));
        break;
      }
      this->input_handle_ = chr_input->handle;

      // Look for control handle
      this->control_handle_ = 0;
      auto chr_control = this->parent()->get_characteristic(this->control_service_uuid_, this->control_char_uuid_);
      if (chr_control == nullptr) {
        this->status_set_warning();
        std::array<char, 37> control_service_uuid_buf{};
        std::array<char, 37> control_char_uuid_buf{};
        ESP_LOGW(TAG, "No characteristic found at service %s char %s",
                 this->control_service_uuid_.to_str(control_service_uuid_buf),
                 this->control_char_uuid_.to_str(control_char_uuid_buf));
        break;
      }
      this->control_handle_ = chr_control->handle;

      this->set_timeout("desk_init", 5000, [this]() { this->read_value_(this->output_handle_); });

      break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent()->get_conn_id())
        break;
      if (param->read.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Error reading char at handle %d, status=%d", param->read.handle, param->read.status);
        break;
      }
      if (param->read.handle == this->output_handle_) {
        this->status_clear_warning();
        this->publish_desk_state_(param->read.value, param->read.value_len);
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id() || param->notify.handle != this->output_handle_)
        break;
      ESP_LOGV(TAG, "[%s] ESP_GATTC_NOTIFY_EVT: handle=0x%x, value=0x%x", this->get_name().c_str(),
               param->notify.handle, param->notify.value[0]);
      this->publish_desk_state_(param->notify.value, param->notify.value_len);
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->node_state = espbt::ClientState::ESTABLISHED;
      if (param->reg_for_notify.status == ESP_GATT_OK) {
        this->notify_disable_ = false;
      }
      break;
    }

    default:
      break;
  }
}

void TimotionDeskControllerComponent::write_value_(uint16_t handle, uint64_t value) {
  if (handle == 0) {
    ESP_LOGW(TAG, "[%s] Skip write, characteristic handle is not ready yet", this->get_name().c_str());
    return;
  }
  ESP_LOGD(">>>> ", "write_value_");
  uint8_t data[5];
  for (int i = 4; i >= 0; --i) {
    data[4 - i] = (value >> (8 * i)) & 0xFF;
  }

  esp_err_t status = ::esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), handle,
                                                sizeof(data), data, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

  if (status != ESP_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "[%s] Error sending write request for number, status=%d", this->get_name().c_str(), status);
  }
}

void TimotionDeskControllerComponent::read_value_(uint16_t handle) {
  if (handle == 0) {
    ESP_LOGW(TAG, "[%s] Skip read, characteristic handle is not ready yet", this->get_name().c_str());
    return;
  }
  auto status_read = esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), handle,
                                             ESP_GATT_AUTH_REQ_NONE);
  if (status_read) {
    this->status_set_warning();
    ESP_LOGW(TAG, "[%s] Error sending read request for number, status=%d", this->get_name().c_str(), status_read);
  }
}

void TimotionDeskControllerComponent::publish_desk_state_(uint8_t *value, uint16_t value_len) {
  if (value_len < 4) {
    ESP_LOGW(TAG, "[%s] Invalid notification payload length: %u", this->get_name().c_str(), value_len);
    return;
  }

  const uint16_t height = value[3];
  const uint16_t speed = value[1];

  if (this->lastHeight == height && this->lastSpeed == speed) return; 
  this->lastHeight = height;
  this->lastSpeed = speed;

  const float measured_height = clamp(static_cast<float>(height), DESK_MIN_HEIGHT, DESK_MAX_HEIGHT);
  ESP_LOGD(TAG, "publish speed=%u height=%.1f previous=%.1f", speed, measured_height, this->state);

  if (speed == 64) {
    this->current_direction_ = DeskDirection::IDLE;
  } else if (speed == 66) {
    this->current_direction_ = DeskDirection::UP;
  } else if (speed == 65) {
    this->current_direction_ = DeskDirection::DOWN;
  }

  this->publish_state(measured_height);
}

void TimotionDeskControllerComponent::move_desk_() {
  if (this->notify_disable_) {
    if (this->controlled_ || this->current_direction_ != DeskDirection::IDLE) {
      this->read_value_(this->output_handle_);
    }
  }

  if (!this->controlled_) {
    return;
  }

  // Check if target has been reached
  if (this->is_at_target_()) {
    ESP_LOGD(TAG, "Update Desk - target reached");
    this->stop_move_();
    return;
  }

  // Stop if command is active but position is no longer changing.
  if (std::fabs(this->state - this->last_move_height_) <= HEIGHT_EPSILON / 2.0f) {
    this->stalled_loops_++;
    if (this->stalled_loops_ > STALLED_LOOPS_LIMIT) {
      if (this->target_height_ >= DESK_MAX_HEIGHT - HEIGHT_EPSILON) {
        this->publish_state(DESK_MAX_HEIGHT);
      } else if (this->target_height_ <= DESK_MIN_HEIGHT + HEIGHT_EPSILON) {
        this->publish_state(DESK_MIN_HEIGHT);
      }
      ESP_LOGD(TAG, "Update Desk - position stalled, stopping movement");
      this->stop_move_();
      return;
    }
  } else {
    this->stalled_loops_ = 0;
    this->last_move_height_ = this->state;
  }

  if (this->notify_disable_) {
    if (this->current_direction_ == DeskDirection::IDLE) {
      this->not_moving_loop_++;
      if (this->not_moving_loop_ > 4) {
        ESP_LOGD(TAG, "Update Desk - desk not moving");
        this->stop_move_();
      }
    } else {
      this->not_moving_loop_ = 0;
    }
  }

  ESP_LOGD(TAG, "Update Desk - Move from %.1f to %.1f", this->state, this->target_height_);
  this->move_torwards_();
}

void TimotionDeskControllerComponent::control(float value) {
  if (this->notify_disable_) {
    this->read_value_(this->output_handle_);
  }

  if (!std::isfinite(this->state)) {
    if (this->lastHeight == 0) {
      ESP_LOGW(TAG, "Desk height is unknown yet, ignoring target %.1f until first measurement", value);
      return;
    }
    this->publish_state(clamp(static_cast<float>(this->lastHeight), DESK_MIN_HEIGHT, DESK_MAX_HEIGHT));
  }

  if (this->current_direction_ != DeskDirection::IDLE) {
    this->stop_move_();
  }

  this->target_height_ = clamp(value, DESK_MIN_HEIGHT, DESK_MAX_HEIGHT);

  if (std::fabs(this->state - this->target_height_) <= HEIGHT_EPSILON) {
    return;
  }

  this->start_move_torwards_();
}

void TimotionDeskControllerComponent::start_move_torwards_() {
  this->controlled_ = true;
  this->last_move_height_ = this->state;
  this->stalled_loops_ = 0;
  if (this->notify_disable_) {
    this->not_moving_loop_ = 0;
  }

  if (this->target_height_ > this->state) {
    this->current_direction_ = DeskDirection::UP;
  } else {
    this->current_direction_ = DeskDirection::DOWN;
  }

  //   if (false == this->use_only_up_down_command_) {
  //     this->write_value_(this->control_handle_, 0xFE);
  //     this->write_value_(this->control_handle_, 0xFF);
  //   }
}

void TimotionDeskControllerComponent::move_torwards_() {
  //   if (this->use_only_up_down_command_) {
  if (this->current_direction_ == DeskDirection::UP) {
    //   this->write_value_(this->control_handle_, 0x47);
    this->write_value_(this->control_handle_, 0xd9ff01633c);
  } else if (this->current_direction_ == DeskDirection::DOWN) {
    this->write_value_(this->control_handle_, 0xd9ff02603a);
  }
  //   } else {
  //     this->write_value_(this->input_handle_, this->target_height_);
  //   }
}

void TimotionDeskControllerComponent::stop_move_() {
  this->write_value_(this->control_handle_, 0x0000000000); // not needed?
  //   if (false == this->use_only_up_down_command_) {
  //     this->write_value_(this->input_handle_, 0x8001);
  //   }

  this->current_direction_ = DeskDirection::IDLE;
  this->controlled_ = false;
}

bool TimotionDeskControllerComponent::is_at_target_() const {
  switch (this->current_direction_) {
    case DeskDirection::UP:
      return this->state + HEIGHT_EPSILON >= this->target_height_;
    case DeskDirection::DOWN:
      return this->state <= this->target_height_ + HEIGHT_EPSILON;
    case DeskDirection::IDLE:
      if (this->notify_disable_) {
        return !this->controlled_;
      }
      return true;
    default:
      return true;
  }
}

espbt::ESPBTUUID uuid128_from_string(std::string value) {
  esp_bt_uuid_t m_uuid;
  m_uuid.len = ESP_UUID_LEN_128;
  int n = 0;
  for (int i = 0; i < value.length();) {
    if (value.c_str()[i] == '-') {
      i++;
    }
    uint8_t MSB = value.c_str()[i];
    uint8_t LSB = value.c_str()[i + 1];
    if (MSB > '9') {
      MSB -= 7;
    }
    if (LSB > '9') {
      LSB -= 7;
    }
    m_uuid.uuid.uuid128[15 - n++] = ((MSB & 0x0F) << 4) | (LSB & 0x0F);
    i += 2;
  }

  return espbt::ESPBTUUID::from_uuid(m_uuid);
}

}  // namespace timotion_desk_controller
}  // namespace esphome
