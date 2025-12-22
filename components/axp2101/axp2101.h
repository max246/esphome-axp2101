#ifndef __AXP2101_H__
#define __AXP2101_H__

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

namespace esphome {
namespace axp2101 {

enum AXP2101Model {
  AXP2101_M5CORE2,
};

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)

#define CURRENT_100MA  (0b0000)
#define CURRENT_190MA  (0b0001)
#define CURRENT_280MA  (0b0010)
#define CURRENT_360MA  (0b0011)
#define CURRENT_450MA  (0b0100)
#define CURRENT_550MA  (0b0101)
#define CURRENT_630MA  (0b0110)
#define CURRENT_700MA  (0b0111)

class AXP2101Component : public PollingComponent, public i2c::I2CDevice {
public:
  void set_battery_voltage_sensor(sensor::Sensor *battery_voltage_sensor) { battery_voltage_sensor_ = battery_voltage_sensor; }
  void set_battery_level_sensor(sensor::Sensor *battery_level_sensor) { battery_level_sensor_ = battery_level_sensor; }
  void set_battery_charging_bsensor(binary_sensor::BinarySensor *battery_charging_bsensor) { battery_charging_bsensor_ = battery_charging_bsensor; }
  void set_brightness(float brightness) { brightness_ = brightness; }
  void set_enable_ALDO3(bool status) { on_boot_ALDO3 = status; }
  void set_enable_DLDO1(bool status) { on_boot_DLDO1 = status; }
  void set_enable_DLDO2(bool status) { on_boot_DLDO2 = status; }
  void set_model(AXP2101Model model) { this->model_ = model; }


  void set_backlight(bool on);
  void set_speaker_enabled(bool on);
  void set_charging_led_mode(std::string mode);

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;



private:
    static std::string get_startup_reason();

protected:
    sensor::Sensor *battery_voltage_sensor_;
    sensor::Sensor *battery_level_sensor_;
    binary_sensor::BinarySensor *battery_charging_bsensor_;
    float brightness_{1.0f};
    float curr_brightness_{-1.0f};
    bool on_boot_ALDO3{false};
    bool on_boot_DLDO1{false};
    bool on_boot_DLDO2{false};
    AXP2101Model model_;

    /** M5Stack Core2 Values
     * LDO2: ILI9342C PWR (Display)
     * LD03: Vibration Motor
     */

    void  update_brightness();
    bool  get_bat_state();
    uint8_t  get_bat_data();

    void  enable_coulomb_counter(void);
    void  disable_coulomb_counter(void);
    void  stop_coulomb_counter(void);
    void  clear_coulomb_counter(void);
    uint32_t get_coulomb_charge_data(void);
    uint32_t get_coulomb_discharge_data(void);
    float get_coulomb_data(void);

    uint16_t get_vbat_data(void) __attribute__((deprecated));
    uint16_t get_icharge_data(void) __attribute__((deprecated));
    uint16_t get_idischarge_data(void) __attribute__((deprecated));
    uint16_t get_temp_data(void) __attribute__((deprecated));
    uint32_t get_power_bat_data(void) __attribute__((deprecated));
    uint16_t get_vin_data(void) __attribute__((deprecated));
    uint16_t get_iin_data(void) __attribute__((deprecated));
    uint16_t get_vusbin_data(void) __attribute__((deprecated));
    uint16_t get_iusbin_data(void) __attribute__((deprecated));
    uint16_t get_vaps_data(void) __attribute__((deprecated));
    uint8_t  get_btn_press(void);

      // -- sleep
    void set_sleep(void);
    void set_deep_sleep(uint64_t time_in_us = 0);
    void set_light_sleep(uint64_t time_in_us = 0);

    void  set_charge_current( uint8_t );
    float get_bat_current();
    float get_vin_voltage();
    float get_vin_current();
    float get_vbus_voltage();
    float get_vbus_current();
    float get_temp_in_AXP2101();
    float get_bat_power();
    float get_bat_charge_current();
    float get_aps_voltage();
    float get_bat_coulomb_input();
    float get_bat_coulomb_out();
    uint8_t get_warning_level(void);
    void set_coulomb_clear();
    void set_LDO2( bool state );
    void set_LDO3( bool state );
    void set_adc_state(bool state);

    void power_off();


    void write_1byte( uint8_t addr ,  uint8_t data );
    uint8_t read_8bit( uint8_t addr );
    uint16_t read_12bit( uint8_t addr);
    uint16_t read_13bit( uint8_t addr);
    uint16_t read_16bit( uint8_t addr );
    uint32_t read_24bit( uint8_t addr );
    uint32_t read_32bit( uint8_t addr );
    void read_buff( uint8_t addr , uint8_t size , uint8_t *buff );
};

}
}

#endif
