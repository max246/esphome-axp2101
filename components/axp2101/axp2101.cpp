#include "axp2101.h"
#include "esp_sleep.h"
#include "esphome/core/log.h"
#include <Esp.h>

#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif

#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif

bool  pmu_flag = 0;
XPowersPMU PMU;

const uint8_t i2c_sda = CONFIG_PMU_SDA;
const uint8_t i2c_scl = CONFIG_PMU_SCL;
const uint8_t pmu_irq_pin = CONFIG_PMU_IRQ;

void setFlag(void)
{
    pmu_flag = true;
}

namespace esphome {
namespace axp2101 {

static const char *TAG = "axp2101.sensor";

void AXP2101Component::setup()
{
    ESP_LOGCONFIG(TAG, "getID:0x%x", PMU.getChipID());

    // Set the minimum common working voltage of the PMU VBUS input,
    // below this value will turn off the PMU
    PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

    // Set the maximum current of the PMU VBUS input,
    // higher than this value will turn off the PMU
    PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);


    // Get the VSYS shutdown voltage
    uint16_t vol = PMU.getSysPowerDownVoltage();
    ESP_LOGCONFIG(TAG, "->  getSysPowerDownVoltage:%u", vol);

    // Set VSY off voltage as 2600mV , Adjustment range 2600mV ~ 3300mV
    PMU.setSysPowerDownVoltage(2600);

    vol = PMU.getSysPowerDownVoltage();
    ESP_LOGCONFIG(TAG, "->  getSysPowerDownVoltage:%u", vol);


    // DC1 IMAX=2A
    // 1500~3400mV,100mV/step,20steps
    PMU.setDC1Voltage(3300);
    ESP_LOGCONFIG(TAG, "DC1  : %s   Voltage:%u mV",  PMU.isEnableDC1()  ? "+" : "-", PMU.getDC1Voltage());

    // DC2 IMAX=2A
    // 500~1200mV  10mV/step,71steps
    // 1220~1540mV 20mV/step,17steps
    PMU.setDC2Voltage(1000);
    ESP_LOGCONFIG(TAG, "DC2  : %s   Voltage:%u mV",  PMU.isEnableDC2()  ? "+" : "-", PMU.getDC2Voltage());

    // DC3 IMAX = 2A
    // 500~1200mV,10mV/step,71steps
    // 1220~1540mV,20mV/step,17steps
    // 1600~3400mV,100mV/step,19steps
    PMU.setDC3Voltage(3300);
    ESP_LOGCONFIG(TAG, "DC3  : %s   Voltage:%u mV",  PMU.isEnableDC3()  ? "+" : "-", PMU.getDC3Voltage());

    // DCDC4 IMAX=1.5A
    // 500~1200mV,10mV/step,71steps
    // 1220~1840mV,20mV/step,32steps
    PMU.setDC4Voltage(1000);
    ESP_LOGCONFIG(TAG, "DC4  : %s   Voltage:%u mV",  PMU.isEnableDC4()  ? "+" : "-", PMU.getDC4Voltage());

    // DC5 IMAX=2A
    // 1200mV
    // 1400~3700mV,100mV/step,24steps
    PMU.setDC5Voltage(3300);
    ESP_LOGCONFIG(TAG, "DC5  : %s   Voltage:%u mV",  PMU.isEnableDC5()  ? "+" : "-", PMU.getDC5Voltage());

    //ALDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO1Voltage(3300);

    //ALDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO2Voltage(3300);

    //ALDO3 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    if (on_boot_ALDO3)
        PMU.setALDO3Voltage(3300);

    //ALDO4 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setALDO4Voltage(3300);

    //BLDO1 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setBLDO1Voltage(3300);

    //BLDO2 IMAX=300mA
    //500~3500mV, 100mV/step,31steps
    PMU.setBLDO2Voltage(3300);

    //CPUSLDO IMAX=30mA
    //500~1400mV,50mV/step,19steps
    PMU.setCPUSLDOVoltage(1000);

    //DLDO1 IMAX=300mA
    //500~3400mV, 100mV/step,29steps
    if (on_boot_DLDO1)    
        PMU.setDLDO1Voltage(3300);

    //DLDO2 IMAX=300mA
    //500~1400mV, 50mV/step,2steps
    if (on_boot_DLDO2)
        PMU.setDLDO2Voltage(3300);


    // PMU.enableDC1();
    PMU.enableDC2();
    PMU.enableDC3();
    PMU.enableDC4();
    PMU.enableDC5();
    PMU.enableALDO1();
    PMU.enableALDO2();
    if (on_boot_ALDO3)
       PMU.enableALDO3(); // This is the speaker
    PMU.enableALDO4();
    PMU.enableBLDO1();
    PMU.enableBLDO2();
    PMU.enableCPUSLDO();
    if (on_boot_DLDO1)    
        PMU.enableDLDO1(); // This is the vibration motor
    if (on_boot_DLDO2)
        PMU.enableDLDO2();


    ESP_LOGCONFIG(TAG, "DC1  : %s   Voltage:%u mV",  PMU.isEnableDC1()  ? "+" : "-", PMU.getDC1Voltage());
    ESP_LOGCONFIG(TAG, "DC2  : %s   Voltage:%u mV",  PMU.isEnableDC2()  ? "+" : "-", PMU.getDC2Voltage());
    ESP_LOGCONFIG(TAG, "DC3  : %s   Voltage:%u mV",  PMU.isEnableDC3()  ? "+" : "-", PMU.getDC3Voltage());
    ESP_LOGCONFIG(TAG, "DC4  : %s   Voltage:%u mV",  PMU.isEnableDC4()  ? "+" : "-", PMU.getDC4Voltage());
    ESP_LOGCONFIG(TAG, "DC5  : %s   Voltage:%u mV",  PMU.isEnableDC5()  ? "+" : "-", PMU.getDC5Voltage());
   // ESP_LOGCONFIG(TAG, "ALDO1: %s   Voltage:%u mV",  PMU.isEnableALDO1()  ? "+" : "-", PMU.getALDO1Voltage());
   // ESP_LOGCONFIG(TAG, "ALDO2: %s   Voltage:%u mV",  PMU.isEnableALDO2()  ? "+" : "-", PMU.getALDO2Voltage());
    if (on_boot_ALDO3)
        ESP_LOGCONFIG(TAG, "ALDO3: %s   Voltage:%u mV",  PMU.isEnableALDO3()  ? "+" : "-", PMU.getALDO3Voltage());
    //ESP_LOGCONFIG(TAG, "ALDO4: %s   Voltage:%u mV",  PMU.isEnableALDO4()  ? "+" : "-", PMU.getALDO4Voltage());

    ESP_LOGCONFIG(TAG, "BLDO1: %s   Voltage:%u mV",  PMU.isEnableBLDO1()  ? "+" : "-", PMU.getBLDO1Voltage());
    ESP_LOGCONFIG(TAG, "BLDO2: %s   Voltage:%u mV",  PMU.isEnableBLDO2()  ? "+" : "-", PMU.getBLDO2Voltage());
    //ESP_LOGCONFIG(TAG, "CPUSLDO: %s Voltage:%u mV",  PMU.isEnableCPUSLDO() ? "+" : "-", PMU.getCPUSLDOVoltage());
    if (on_boot_DLDO1)        
        SP_LOGCONFIG(TAG, "DLDO1: %s   Voltage:%u mV",  PMU.isEnableDLDO1()  ? "+" : "-", PMU.getDLDO1Voltage());
    if (on_boot_DLDO2)    
        ESP_LOGCONFIG(TAG, "DLDO2: %s   Voltage:%u mV",  PMU.isEnableDLDO2()  ? "+" : "-", PMU.getDLDO2Voltage());


    ESP_LOGCONFIG(TAG, "Startup setting for ALDO3 : %s",  on_boot_ALDO3);    
    ESP_LOGCONFIG(TAG, "Startup setting for DLDO1 : %s",  on_boot_DLDO1);    
    ESP_LOGCONFIG(TAG, "Startup setting for DLDO2 : %s",  on_boot_DLDO2);

    // Set the time of pressing the button to turn off
    PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU.getPowerKeyPressOffTime();
    switch (opt) {
    case XPOWERS_POWEROFF_4S:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 4 Second");
        break;
    case XPOWERS_POWEROFF_6S:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 6 Second");
        break;
    case XPOWERS_POWEROFF_8S:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 8 Second");
        break;
    case XPOWERS_POWEROFF_10S:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOffTime: 10 Second");
        break;
    default:
        break;
    }
    // Set the button power-on press time
    PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    opt = PMU.getPowerKeyPressOnTime();
    switch (opt) {
    case XPOWERS_POWERON_128MS:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 128 Ms");
        break;
    case XPOWERS_POWERON_512MS:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 512 Ms");
        break;
    case XPOWERS_POWERON_1S:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 1 Second");
        break;
    case XPOWERS_POWERON_2S:
        ESP_LOGCONFIG(TAG, "PowerKeyPressOnTime: 2 Second");
        break;
    default:
        break;
    }

    bool en;

    // DCDC 120%(130%) high voltage turn off PMIC function
    en = PMU.getDCHighVoltagePowerDownEn();
    ESP_LOGCONFIG(TAG, "getDCHighVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
    // DCDC1 85% low voltage turn off PMIC function
    en = PMU.getDC1LowVoltagePowerDownEn();
    ESP_LOGCONFIG(TAG, "getDC1LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
    // DCDC2 85% low voltage turn off PMIC function
    en = PMU.getDC2LowVoltagePowerDownEn();
    ESP_LOGCONFIG(TAG, "getDC2LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
    // DCDC3 85% low voltage turn off PMIC function
    en = PMU.getDC3LowVoltagePowerDownEn();
    ESP_LOGCONFIG(TAG, "getDC3LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
    // DCDC4 85% low voltage turn off PMIC function
    en = PMU.getDC4LowVoltagePowerDownEn();
    ESP_LOGCONFIG(TAG, "getDC4LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");
    // DCDC5 85% low voltage turn off PMIC function
    en = PMU.getDC5LowVoltagePowerDownEn();
    ESP_LOGCONFIG(TAG, "getDC5LowVoltagePowerDownEn: %s", en ? "ENABLE" : "DISABLE");

    // PMU.setDCHighVoltagePowerDown(true);
    // PMU.setDC1LowVoltagePowerDown(true);
    // PMU.setDC2LowVoltagePowerDown(true);
    // PMU.setDC3LowVoltagePowerDown(true);
    // PMU.setDC4LowVoltagePowerDown(true);
    // PMU.setDC5LowVoltagePowerDown(true);

    // It is necessary to disable the detection function of the TS pin on the board
    // without the battery temperature detection function, otherwise it will cause abnormal charging
    PMU.disableTSPinMeasure();

    PMU.enableTemperatureMeasure();

    // Enable internal ADC detection
    PMU.enableBattDetection();
    PMU.enableVbusVoltageMeasure();
    PMU.enableBattVoltageMeasure();
    PMU.enableSystemVoltageMeasure();


    /*
      The default setting is CHGLED is automatically controlled by the PMU.
    - XPOWERS_CHG_LED_OFF,
    - XPOWERS_CHG_LED_BLINK_1HZ,
    - XPOWERS_CHG_LED_BLINK_4HZ,
    - XPOWERS_CHG_LED_ON,
    - XPOWERS_CHG_LED_CTRL_CHG,
    * */
    set_charging_led_mode("CTRL_CHG");


    // Force add pull-up
    pinMode(pmu_irq_pin, INPUT_PULLUP);
    attachInterrupt(pmu_irq_pin, setFlag, FALLING);


    // Disable all interrupts
    PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    // Clear all interrupt flags
    PMU.clearIrqStatus();
    // Enable the required interrupt function
    PMU.enableIRQ(
        XPOWERS_AXP2101_BAT_INSERT_IRQ    | XPOWERS_AXP2101_BAT_REMOVE_IRQ      |   //BATTERY
        XPOWERS_AXP2101_VBUS_INSERT_IRQ   | XPOWERS_AXP2101_VBUS_REMOVE_IRQ     |   //VBUS
        XPOWERS_AXP2101_PKEY_SHORT_IRQ    | XPOWERS_AXP2101_PKEY_LONG_IRQ       |   //POWER KEY
        XPOWERS_AXP2101_BAT_CHG_DONE_IRQ  | XPOWERS_AXP2101_BAT_CHG_START_IRQ       //CHARGE
        // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
    );

    // Set the precharge charging current
    PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    // Set constant current charge current limit
    PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    // Set stop charging termination current
    PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

    // Set charge cut-off voltage
    PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

    // Set the watchdog trigger event type
    PMU.setWatchdogConfig(XPOWERS_AXP2101_WDT_IRQ_TO_PIN);
    // Set watchdog timeout
    PMU.setWatchdogTimeout(XPOWERS_AXP2101_WDT_TIMEOUT_4S);
    // Enable watchdog to trigger interrupt event
    PMU.enableWatchdog();

    // PMU.disableWatchdog();

    // Enable Button Battery charge
    PMU.enableButtonBatteryCharge();

    // Set Button Battery charge voltage
    PMU.setButtonBatteryChargeVoltage(3300);
}

void AXP2101Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP2101:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->battery_level_sensor_);
  LOG_BINARY_SENSOR("  ", "Battery Charging", this->battery_charging_bsensor_);
}

float AXP2101Component::get_setup_priority() const { return setup_priority::DATA; }

void AXP2101Component::update() {

    if (this->battery_level_sensor_ != nullptr) {
      float vbat = PMU.getBattVoltage();
      ESP_LOGD(TAG, "Got Battery Voltage=%f", vbat);
      this->battery_voltage_sensor_->publish_state(vbat / 1000.);

      // The battery percentage may be inaccurate at first use, the PMU will automatically
      // learn the battery curve and will automatically calibrate the battery percentage
      // after a charge and discharge cycle
      float battery_level;
      if (PMU.isBatteryConnect()) {
        battery_level = PMU.getBatteryPercent();
      } else {
        battery_level = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));
      }

      ESP_LOGD(TAG, "Got Battery Level=%f", battery_level);
      if (battery_level > 100.) {
        battery_level = 100;
      }
      this->battery_level_sensor_->publish_state(battery_level);
    }

    if (this->battery_charging_bsensor_ != nullptr) {
      bool vcharging = PMU.isCharging();

      ESP_LOGD(TAG, "Got Battery Charging=%s", vcharging ? "true" : "false");
      this->battery_charging_bsensor_->publish_state(vcharging);
    }

    update_brightness();
}

void AXP2101Component::write_1byte( uint8_t addr ,  uint8_t data )
{
    this->write_byte(addr, data);
}

uint8_t AXP2101Component::read_8bit( uint8_t addr )
{
    uint8_t data;
    this->read_byte(addr, &data);
    return data;
}

uint16_t AXP2101Component::read_12bit( uint8_t addr)
{
    uint16_t data = 0;
    uint8_t buf[2];
    read_buff(addr,2,buf);
    data = ((buf[0] << 4) + buf[1]); //
    return data;
}

uint16_t AXP2101Component::read_13bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    read_buff(Addr,2,buf);
    Data = ((buf[0] << 5) + buf[1]); //
    return Data;
}

uint16_t AXP2101Component::read_16bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP2101Component::read_24bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP2101Component::read_32bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP2101Component::read_buff( uint8_t Addr , uint8_t Size , uint8_t *Buff )
{
    this->read_bytes(Addr, Buff, Size);
}

// Screen enable
void AXP2101Component::set_backlight(bool on) {
    if (on) {
        PMU.enableBLDO1();  // backlight on
    } else {
        PMU.disableBLDO1(); // backlight off only
    }
}

// SPEAKER on / off
void AXP2101Component::set_speaker_enabled(bool on) {
    if (on) {
        PMU.enableALDO3();  // speaker on
    } else {
        PMU.disableALDO3(); // speaker off
    }
}

// Set charging Led mode
void AXP2101Component::set_charging_led_mode(std::string mode) {
    /*
    - XPOWERS_CHG_LED_OFF,
    - XPOWERS_CHG_LED_BLINK_1HZ,
    - XPOWERS_CHG_LED_BLINK_4HZ,
    - XPOWERS_CHG_LED_ON,
    - XPOWERS_CHG_LED_CTRL_CHG,
    */
    if (mode == "OFF")
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
    else if (mode == "BLINK_1HZ")
        PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
    else if (mode == "BLINK_4HZ")
        PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
    else if (mode == "ON") {
        PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);}
    else if (mode == "CTRL_CHG")
        PMU.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
    else    
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
}


void AXP2101Component::update_brightness()
{
    if (brightness_ == curr_brightness_)
    {
        return;
    }

    ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
    curr_brightness_ = brightness_;

    const uint8_t c_min = 7;
    const uint8_t c_max = 12;
    auto ubri = c_min + static_cast<uint8_t>(brightness_ * (c_max - c_min));

    if (ubri > c_max)
    {
        ubri = c_max;
    }
    switch (this->model_) {
      case AXP2101_M5CORE2:
      {
        uint8_t buf = read_8bit( 0x27 );
        write_1byte( 0x27 , ((buf & 0x80) | (ubri << 3)) );
        break;
      }
    }
}

bool AXP2101Component::get_bat_state()
{
    if( read_8bit(0x01) | 0x20 )
        return true;
    else
        return false;
}

uint8_t AXP2101Component::get_bat_data()
{
    return read_8bit(0x75);
}
//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void);
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void  AXP2101Component::enable_coulomb_counter(void)
{
    write_1byte( 0xB8 , 0x80 );
}

void  AXP2101Component::disable_coulomb_counter(void)
{
    write_1byte( 0xB8 , 0x00 );
}

void  AXP2101Component::stop_coulomb_counter(void)
{
    write_1byte( 0xB8 , 0xC0 );
}

void  AXP2101Component::clear_coulomb_counter(void)
{
    write_1byte( 0xB8 , 0xA0 );
}

uint32_t AXP2101Component::get_coulomb_charge_data(void)
{
    return read_32bit(0xB0);
}

uint32_t AXP2101Component::get_coulomb_discharge_data(void)
{
    return read_32bit(0xB4);
}

float AXP2101Component::get_coulomb_data(void)
{

  uint32_t coin = 0;
  uint32_t coout = 0;

  coin = get_coulomb_charge_data();
  coout = get_coulomb_discharge_data();

  //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
  //Adc rate can be read from 84H ,change this variable if you change the ADC reate
  float ccc = 65536 * 0.5 * (coin - coout) / 3600.0 / 25.0;
  return ccc;

}
//----------coulomb_end_at_here----------

uint16_t AXP2101Component::get_vbat_data(void){

    uint16_t vbat = 0;
    uint8_t buf[2];
    read_buff(0x78,2,buf);
    vbat = ((buf[0] << 4) + buf[1]); // V
    return vbat;
}

uint16_t AXP2101Component::get_vin_data(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    read_buff(0x56,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP2101Component::get_iin_data(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    read_buff(0x58,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP2101Component::get_vusbin_data(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    read_buff(0x5a,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP2101Component::get_iusbin_data(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    read_buff(0x5C,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP2101Component::get_icharge_data(void)
{
    uint16_t icharge = 0;
    uint8_t buf[2];
    read_buff(0x7A,2,buf);
    icharge = ( buf[0] << 5 ) + buf[1] ;
    return icharge;
}

uint16_t AXP2101Component::get_idischarge_data(void)
{
    uint16_t idischarge = 0;
    uint8_t buf[2];
    read_buff(0x7C,2,buf);
    idischarge = ( buf[0] << 5 ) + buf[1] ;
    return idischarge;
}

uint16_t AXP2101Component::get_temp_data(void)
{
    uint16_t temp = 0;
    uint8_t buf[2];
    read_buff(0x5e,2,buf);
    temp = ((buf[0] << 4) + buf[1]);
    return temp;
}

uint32_t AXP2101Component::get_power_bat_data(void)
{
    uint32_t power = 0;
    uint8_t buf[3];
    read_buff(0x70,2,buf);
    power = (buf[0] << 16) + (buf[1] << 8) + buf[2];
    return power;
}

uint16_t AXP2101Component::get_vaps_data(void)
{
    uint16_t vaps = 0;
    uint8_t buf[2];
    read_buff(0x7e,2,buf);
    vaps = ((buf[0] << 4) + buf[1]);
    return vaps;
}

void AXP2101Component::set_sleep(void)
{
    write_1byte(0x31 , read_8bit(0x31) | ( 1 << 3)); // Power off voltag 3.0v
    write_1byte(0x90 , read_8bit(0x90) | 0x07); // GPIO1 floating
    write_1byte(0x82, 0x00); // Disable ADCs
    write_1byte(0x12, read_8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
}

// -- sleep
void AXP2101Component::set_deep_sleep(uint64_t time_in_us)
{
    set_sleep();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)37, 0 /* LOW */);
    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);
}

void AXP2101Component::set_light_sleep(uint64_t time_in_us)
{
    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();
}

// 0 not press, 0x01 long press, 0x02 press
uint8_t AXP2101Component::get_btn_press()
{
    uint8_t state = read_8bit(0x46);
    if(state)
    {
        write_1byte( 0x46 , 0x03 );
    }
    return state;
}

uint8_t AXP2101Component::get_warning_level(void)
{
    return read_8bit(0x47) & 0x01;
}

float AXP2101Component::get_bat_current()
{
    float ADCLSB = 0.5;
    uint16_t current_in = read_13bit( 0x7A );
    uint16_t current_out = read_13bit( 0x7C );
    return ( current_in - current_out ) * ADCLSB;
}

float AXP2101Component::get_vin_voltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t re_data = read_12bit( 0x56 );
    return re_data * ADCLSB;
}

float AXP2101Component::get_vin_current()
{
    float ADCLSB = 0.625;
    uint16_t re_data = read_12bit( 0x58 );
    return re_data * ADCLSB;
}

float AXP2101Component::get_vbus_voltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t re_data = read_12bit( 0x5A );
    return re_data * ADCLSB;
}

float AXP2101Component::get_vbus_current()
{
    float ADCLSB = 0.375;
    uint16_t re_data = read_12bit( 0x5C );
    return re_data * ADCLSB;
}

float AXP2101Component::get_temp_in_AXP2101()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t re_data = read_12bit( 0x5E );
    return OFFSET_DEG_C + re_data * ADCLSB;
}

float AXP2101Component::get_bat_power()
{
    float voltage_LSB = 1.1;
    float current_LCS = 0.5;
    uint32_t re_data = read_24bit( 0x70 );
    return  voltage_LSB * current_LCS * re_data/ 1000.0;
}

float AXP2101Component::get_bat_charge_current()
{
    float ADCLSB = 0.5;
    uint16_t re_data = read_13bit( 0x7A );
    return re_data * ADCLSB;
}

float AXP2101Component::get_aps_voltage()
{
    float ADCLSB = 1.4  / 1000.0;
    uint16_t re_data = read_12bit( 0x7E );
    return re_data * ADCLSB;
}

float AXP2101Component::get_bat_coulomb_input()
{
    uint32_t re_data = read_32bit( 0xB0 );
    return re_data * 65536 * 0.5 / 3600 /25.0;
}

float AXP2101Component::get_bat_coulomb_out()
{
    uint32_t re_data = read_32bit( 0xB4 );
    return re_data * 65536 * 0.5 / 3600 /25.0;
}

void AXP2101Component::set_coulomb_clear()
{
    write_1byte(0xB8,0x20);
}

void AXP2101Component::set_LDO2( bool state )
{
    uint8_t buf = read_8bit(0x12);
    if( state == true )
    {
        buf = (1<<2) | buf;
    }
    else
    {
        buf = ~(1<<2) & buf;
    }
    write_1byte( 0x12 , buf );
}

void AXP2101Component::set_LDO3(bool State)
{
    uint8_t buf = read_8bit(0x12);
    if( State == true )
    {
        buf = (1<<3) | buf;
    }
    else
    {
        buf = ~(1<<3) & buf;
    }
    write_1byte( 0x12 , buf );
}

void AXP2101Component::set_charge_current(uint8_t current)
{
    uint8_t buf = read_8bit(0x33);
    buf = (buf & 0xf0) | (current & 0x07);
    write_1byte(0x33, buf);
}

void AXP2101Component::power_off()
{
    write_1byte(0x32, read_8bit(0x32) | 0x80);
}

void AXP2101Component::set_adc_state(bool state)
{
    write_1byte(0x82, state ? 0xff : 0x00);
}

std::string AXP2101Component::get_startup_reason() {
  esp_reset_reason_t reset_reason = ::esp_reset_reason();
  if (reset_reason == ESP_RST_DEEPSLEEP) {
    esp_sleep_source_t wake_reason = esp_sleep_get_wakeup_cause();
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
      return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
      return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT1)
      return "ESP_SLEEP_WAKEUP_EXT1";
    if (wake_reason == ESP_SLEEP_WAKEUP_TIMER)
      return "ESP_SLEEP_WAKEUP_TIMER";
    if (wake_reason == ESP_SLEEP_WAKEUP_TOUCHPAD)
      return "ESP_SLEEP_WAKEUP_TOUCHPAD";
    if (wake_reason == ESP_SLEEP_WAKEUP_ULP)
      return "ESP_SLEEP_WAKEUP_ULP";
    if (wake_reason == ESP_SLEEP_WAKEUP_GPIO)
      return "ESP_SLEEP_WAKEUP_GPIO";
    if (wake_reason == ESP_SLEEP_WAKEUP_UART)
      return "ESP_SLEEP_WAKEUP_UART";
    return std::string{"WAKEUP_UNKNOWN_REASON"};
  }

  if (reset_reason == ESP_RST_UNKNOWN)
    return "ESP_RST_UNKNOWN";
  if (reset_reason == ESP_RST_POWERON)
    return "ESP_RST_POWERON";
  if (reset_reason == ESP_RST_SW)
    return "ESP_RST_SW";
  if (reset_reason == ESP_RST_PANIC)
    return "ESP_RST_PANIC";
  if (reset_reason == ESP_RST_INT_WDT)
    return "ESP_RST_INT_WDT";
  if (reset_reason == ESP_RST_TASK_WDT)
    return "ESP_RST_TASK_WDT";
  if (reset_reason == ESP_RST_WDT)
    return "ESP_RST_WDT";
  if (reset_reason == ESP_RST_BROWNOUT)
    return "ESP_RST_BROWNOUT";
  if (reset_reason == ESP_RST_SDIO)
    return "ESP_RST_SDIO";
  return std::string{"RESET_UNKNOWN_REASON"};
}

}
}
