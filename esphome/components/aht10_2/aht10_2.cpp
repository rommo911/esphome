// Implementation based on:
//  - AHT10: https://github.com/Thinary/AHT10
//  - Official Datasheet (cn):
//  http://www.aosong.com/userfiles/files/media/aht10%E8%A7%84%E6%A0%BC%E4%B9%A6v1_1%EF%BC%8820191015%EF%BC%89.pdf
//  - Unofficial Translated Datasheet (en):
//  https://wiki.liutyi.info/download/attachments/30507639/Aosong_AHT10_en_draft_0c.pdf
//
// When configured for humidity, the log 'Components should block for at most 20-30ms in loop().' will be generated in
// verbose mode. This is due to technical specs of the sensor and can not be avoided.
//
// According to the datasheet, the component is supposed to respond in more than 75ms. In fact, it can answer almost
// immediately for temperature. But for humidity, it takes >90ms to get a valid data. From experience, we have best
// results making successive requests; the current implementation makes 3 attempts with a delay of 30ms each time.

#include "aht10_2.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cinttypes>

namespace esphome {
namespace aht10_2 {

static const char *const TAG = "aht10_2";
static const uint8_t AHTXX_INIT_CTRL_NOP = 0x00;  // safety margin, normally 3 attempts are enough: 3*30=90ms
static const uint8_t AHT10_CALIBRATE_CMD = {0xE1};
static const uint8_t AHT10_MEASURE_CMD_1 = 0xAC;

static const uint8_t AHT10_MEASURE_CMD[] = {0x33, AHTXX_INIT_CTRL_NOP};
static const uint8_t AHT10_DEFAULT_DELAY = 10;          // ms, for calibration and temperature measurement
static const uint8_t AHT10_HUMIDITY_DELAY = 10;        // ms
static const uint8_t AHT10_ATTEMPTS = 5;               // safety margin, normally 3 attempts are enough: 3*30=90ms
static const uint8_t AHTXX_SOFT_RESET_REG[] = {0xBA};  // safety margin, normally 3 attempts are enough: 3*30=90ms
static const uint8_t AHTXX_SOFT_RESET_REG2 = 0xBA;  // safety margin, normally 3 attempts are enough: 3*30=90ms

#define AHT1X_INIT_CTRL_NORMAL_MODE 0x00  // normal mode on/off       bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CYCLE_MODE 0x20   // cycle mode on/off        bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CMD_MODE 0x40     // command mode  on/off     bit[6:5], for AHT1x only
#define AHTXX_INIT_CTRL_CAL_ON 0x08       // calibration coeff on/off bit[3]
#define AHTXX_INIT (AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_NORMAL_MODE)
static const uint8_t AHT2X_INIT_REG_ADD = 0xBE ;

static const uint8_t AHT2X_INIT_REG[] = { AHTXX_INIT, AHTXX_INIT_CTRL_NOP};

void AHT10_2_Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AHT10...");

   if (!this->write_command(AHTXX_SOFT_RESET_REG2)) {
      ESP_LOGE(TAG, "Communication with AHT10 AHTXX_SOFT_RESET_REG  failed!");
    //this->mark_failed();
    //return;
  }
  else 
  {
    ESP_LOGI(TAG, "Communication with AHT10 AHTXX_SOFT_RESET_REG  SUCCESS!");
  }
  delay(AHT10_DEFAULT_DELAY);

  //if (!this->write_bytes(0, AHTXX_SOFT_RESET_REG, sizeof(AHTXX_SOFT_RESET_REG))) {
    //ESP_LOGE(TAG, "Communication with AHT10 AHTXX_SOFT_RESET_REG failed!");
    // this->mark_failed();
    // return;
  //}

  if (!this->write_bytes(AHT2X_INIT_REG_ADD, AHT2X_INIT_REG, sizeof(AHT2X_INIT_REG))) {
    ESP_LOGE(TAG, "Communication with AHT10 AHT2X_INIT_REG failed!");
    // this->mark_failed();
    // return;
  }
  delay(AHT10_DEFAULT_DELAY);

  /*if (!this->write_bytes(0, AHT10_CALIBRATE_CMD, sizeof(AHT10_CALIBRATE_CMD))) {
    ESP_LOGE(TAG, "Communication with AHT10 AHT10_CALIBRATE_CMD failed!");
    // this->mark_failed();
    // return;
  }*/

  if (!this->write_command(AHT10_CALIBRATE_CMD)) {
      ESP_LOGE(TAG, "Communication with AHT10 AHT10_CALIBRATE_CMD  failed!");
    //this->mark_failed();
    //return;
  }
  else 
  {
    ESP_LOGI(TAG, "Communication with AHT10 AHT10_CALIBRATE_CMD  SUCCESS!");
  }
  delay(AHT10_DEFAULT_DELAY);

  uint8_t data = 0;
  if (this->write(&data, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with AHT10 write failed!");
    // this->mark_failed();
    // return;
  }
  delay(AHT10_DEFAULT_DELAY);
  if (this->read(&data, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with AHT10 read failed!");
    // this->mark_failed();
    // return;
  }
  if (this->read(&data, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Communication with AHT10  read failed!");
    // this->mark_failed();
    // return;
  }
  if ((data & 0x68) != 0x08) {  // Bit[6:5] = 0b00, NORMAL mode and Bit[3] = 0b1, CALIBRATED
    ESP_LOGE(TAG, "AHT10 Bit[6:5] = 0b00, NORMAL mode and Bit[3] = 0b1, CALIBRATED  // calibration failed!");
    // this->mark_failed();
    return;
  }
  setupDone = true;
  ESP_LOGI(TAG, "AHT10 calibrated");
}

void AHT10_2_Component::update() {
  if (setupDone == false) {
    this->setup();
  }

  if (!this->write_bytes(AHT10_MEASURE_CMD_1, AHT10_MEASURE_CMD, sizeof(AHT10_MEASURE_CMD))) {
    ESP_LOGE(TAG, "Communication with AHT10 update write_bytes AHT10_MEASURE_CMD failed test!");
    // this->status_set_warning();
    setupDone = false;
    return;
  }
  uint8_t data[6];
  uint8_t delay_ms = AHT10_DEFAULT_DELAY;
  if (this->humidity_sensor_ != nullptr)
    delay_ms = AHT10_HUMIDITY_DELAY;
  bool success = false;
  for (int i = 0; i < AHT10_ATTEMPTS; ++i) {
    ESP_LOGVV(TAG, "Attempt %d at %6" PRIu32, i, millis());
    delay(delay_ms);
    if (this->read(data, 6) != i2c::ERROR_OK) {
      ESP_LOGD(TAG, "Communication with AHT10 failed, waiting...");
      continue;
    }

    if ((data[0] & 0x80) == 0x80) {  // Bit[7] = 0b1, device is busy
      ESP_LOGD(TAG, "AHT10 is busy, waiting...");
    } else if (data[1] == 0x0 && data[2] == 0x0 && (data[3] >> 4) == 0x0) {
      // Unrealistic humidity (0x0)
      if (this->humidity_sensor_ == nullptr) {
        ESP_LOGVV(TAG, "ATH10 Unrealistic humidity (0x0), but humidity is not required");
        break;
      } else {
        ESP_LOGD(TAG, "ATH10 Unrealistic humidity (0x0), retrying...");
        if (!this->write_bytes(AHT10_MEASURE_CMD_1, AHT10_MEASURE_CMD, sizeof(AHT10_MEASURE_CMD))) {
          ESP_LOGE(TAG, "Communication with AHT10 failed!");
          this->status_set_warning();
          return;
        }
      }
    } else {
      // data is valid, we can break the loop
      ESP_LOGVV(TAG, "Answer at %6" PRIu32, millis());
      success = true;
      break;
    }
  }
  if (!success || (data[0] & 0x80) == 0x80) {
    ESP_LOGE(TAG, "Measurements reading timed-out!");
    this->status_set_warning();
    return;
  }

  uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
  uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;

  float temperature = ((200.0f * (float) raw_temperature) / 1048576.0f) - 50.0f;
  float humidity;
  if (raw_humidity == 0) {  // unrealistic value
    humidity = NAN;
  } else {
    humidity = (float) raw_humidity * 100.0f / 1048576.0f;
  }

  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }
  if (this->humidity_sensor_ != nullptr) {
    if (std::isnan(humidity)) {
      ESP_LOGW(TAG, "Invalid humidity! Sensor reported 0%% Hum");
    }
    this->humidity_sensor_->publish_state(humidity);
  }
  this->status_clear_warning();
}

float AHT10_2_Component::get_setup_priority() const { return setup_priority::DATA; }

void AHT10_2_Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AHT10:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, " AHT10 set to failed!");
  }
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

}  // namespace aht10
}  // namespace esphome
