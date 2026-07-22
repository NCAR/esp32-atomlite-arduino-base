/*

    IoTwx_Base.ino

    Atmospheric measurement node with
    the Adafruit chips:

      bme680   (thpvoc)
      rg15     (precipitation)
      sen0321  (ozone)
      ms8607   (thp)
      pmsa003i (air quality)
      scd4x    (true co2)
      ltr390   (uva+b)
      sht40    (th)
	  hdc3022  (th) ; high precision
	  tsl2591  (light sensitivity/lux/ir)

    ===
    This code sleeps for 60s, wakes up, takes a measurement,
    transmits and then goes back to sleep.

    copyright (c) 2020-2026 keith maull
    Website    :
    Author     : kmaull-ucar
    Create Time:
    Change Log :
      5/26 == removed seeed/grove code, inserted qwiic/adafruit

*/
#include <WiFi.h>
#include <SPI.h>
#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>
#include "SPIFFS.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_MS8607.h"
#include "Adafruit_PM25AQI.h"
#include "Adafruit_LTR390.h"
#include <SensirionI2cScd4x.h>
#include "Adafruit_SHT4x.h"
#include <Adafruit_HDC302x.h>
#include "IoTwx.h"  /// https://github.com/ncar/esp32-atomlite-arduino-iotwx
#include <SoftwareSerial.h>
#include "rg15arduino.h"
#include "DFRobot_OzoneSensor.h"
#include "SparkFun_Qwiic_Relay.h"
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <ArduinoJson.h>
#include <Adafruit_TSL2591.h>

#define IOTWX_VERSION "2.0.5"
#define FORMAT_LITTLEFS_IF_FAILED true

// POE HAT GPIO PINS
#define SCK 22
#define MISO 23
#define MOSI 33
#define CS 19

#define HDC302X_IIC_ADDR uint8_t(0x44)
#define BMEX80_IIC_ADDR uint8_t(0x76)
#define SEN0321_IIC_ADDR uint8_t(0x73)
#define SEN0321_SAMPLES 20

#define SEALEVELPRESSURE_HPA (1013.25)
#define RELAY_ADDR 0x18  // Alternate address 0x19

Qwiic_Relay aspirator_relay(RELAY_ADDR);

// FILE SCOPE
IoTwx node;
Adafruit_BME680 bme680;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
SensirionI2cScd4x scd4x;
Adafruit_LTR390 ltr = Adafruit_LTR390();
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_HDC302x hdc3022 = Adafruit_HDC302x();
SoftwareSerial atomUART;  // RX, TX
RG15Arduino rg15;
DFRobot_OzoneSensor sen0321;
Adafruit_MS8607 ms8607;
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);

unsigned long last_millis = 0;
unsigned long start_millis = 0;

// DEVICES ATTACHED
bool bme680_attached  = false;
bool pm25aqi_attached = false;
bool scd4x_attached   = false;
bool ltr390_attached  = false;
bool sht4x_attached   = false;
bool rg15_attached    = false;
bool sen0321_attached = false;
bool ms8607_attached  = false;
bool hdc3022_attached = false;
bool tsl2591_attached = false;

// GLOBALS
char *sensor;
char *topic;
char *atom_gpio_config;
int  timezone;
int  reset_interval;
int  publish_interval;
int  use_wifi;
int  max_frequency = 80;
int  aspirated;
int  aspiration_spinup_time;
int  light_sensitivity;
char *transfer_mode;


void publish_tsl2591_measurements() {
	char s[strlen(sensor) + 64];

	uint32_t lum = tsl2591.getFullLuminosity();
	uint16_t ir, full;
	ir = lum >> 16;
	full = lum & 0xFFFF;
 
	strcpy(s, sensor);
	strcat(s, "/tsl2591/ir");
	node.publishMQTTMeasurement(topic, s, ir, 0);

	strcpy(s, sensor);
	strcat(s, "/tsl2591/full");
	node.publishMQTTMeasurement(topic, s, full, 0);

	strcpy(s, sensor);
	strcat(s, "/tsl2591/visible");
	node.publishMQTTMeasurement(topic, s, full - ir, 0);

	strcpy(s, sensor);
	strcat(s, "/tsl2591/lux");
	node.publishMQTTMeasurement(topic, s, tsl2591.calculateLux(full, ir), 0);
}


void publish_ltr390_measurements() {
  char s[strlen(sensor) + 64];

  if (ltr.newDataAvailable()) {
    strcpy(s, sensor);
    strcat(s, "/ltr390/uvs");
    node.publishMQTTMeasurement(topic, s, ltr.readUVS(), 0);
  }
}


void publish_hdc3022_measurements() {
  char s[strlen(sensor) + 64];

  double temp = -999;
  double humidity = -999;

  hdc3022.readTemperatureHumidityOnDemand(temp, humidity, TRIGGERMODE_LP0);

  if (temp != -999) {
    strcpy(s, sensor);
    strcat(s, "/hdc3022/temperature");
    node.publishMQTTMeasurement(topic, s, temp, 0);
  }

  if (humidity != -999) {
    strcpy(s, sensor);
    strcat(s, "/hdc3022/humidity");
    node.publishMQTTMeasurement(topic, s, humidity, 0);
  }

}


void publish_scd4x_measurements() {
  char s[strlen(sensor) + 64];
  uint16_t co2;
  float temperature;
  float humidity;
  uint16_t scd4x_error;

  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  delay(5000);
  scd4x_error = scd4x.readMeasurement(co2, temperature, humidity);

  if (scd4x_error) {
    Serial.print("[error]: scd4x error trying to execute readMeasurement(): ");
  } else if (co2 == 0) {
    Serial.println("[warn]: scd4x invalid sample detected, skipping.");
  } else {
    strcpy(s, sensor);
    strcat(s, "/scd4x/co2");
    node.publishMQTTMeasurement(topic, s, co2, 0);

    strcpy(s, sensor);
    strcat(s, "/scd4x/temperature");
    node.publishMQTTMeasurement(topic, s, temperature, 0);

    strcpy(s, sensor);
    strcat(s, "/scd4x/humidity");
    node.publishMQTTMeasurement(topic, s, humidity, 0);
  }
}


void publish_rg15_measurements() {
  char s[strlen(sensor) + 64];

  delay(5000);

  if (rg15.poll()) {
    strcpy(s, sensor);
    strcat(s, "/rg15/acc");
    node.publishMQTTMeasurement(topic, s, rg15.acc, 0);

    strcpy(s, sensor);
    strcat(s, "/rg15/acc_evt");
    node.publishMQTTMeasurement(topic, s, rg15.eventAcc, 0);

    strcpy(s, sensor);
    strcat(s, "/rg15/acc_tot");
    node.publishMQTTMeasurement(topic, s, rg15.totalAcc, 0);

    strcpy(s, sensor);
    strcat(s, "/rg15/iph");
    node.publishMQTTMeasurement(topic, s, rg15.rInt, 0);
  } else {
    Serial.println("[error]: RG15 measurement timeout, no measurement obtained");
  }
}


void publish_sht4x_measurements() {
  char s[strlen(sensor) + 64];
  sensors_event_t humidity, temp;

  delay(1500);

  sht4.getEvent(&humidity, &temp);

  strcpy(s, sensor);
  strcat(s, "/sht4x/temperature");
  node.publishMQTTMeasurement(topic, s, temp.temperature, 0);

  strcpy(s, sensor);
  strcat(s, "/sht4x/humidity");
  node.publishMQTTMeasurement(topic, s, humidity.relative_humidity, 0);
  //     } else {
  //       Serial.println("[error]: SHT4x sensor data invalid or not ready");
  //     }
  //   } else {
  //     Serial.println("[error]: SHT4x sensor failed to read data");
  //   }
}


void publish_pmsa0031_measurements() {
  PM25_AQI_Data data;
  char s[strlen(sensor) + 64];

  if (!aqi.read(&data)) {
    Serial.println("[warn] could not read from AQI");
    delay(500);
    return;
  }

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/pm10standard");
  node.publishMQTTMeasurement(topic, s, data.pm10_standard, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/pm25standard");
  node.publishMQTTMeasurement(topic, s, data.pm25_standard, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/pm100standard");
  node.publishMQTTMeasurement(topic, s, data.pm100_standard, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/pm10env");
  node.publishMQTTMeasurement(topic, s, data.pm10_env, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/pm25env");
  node.publishMQTTMeasurement(topic, s, data.pm25_env, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/pm100env");
  node.publishMQTTMeasurement(topic, s, data.pm100_env, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/partcount03um");
  node.publishMQTTMeasurement(topic, s, data.particles_03um, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/partcount05um");
  node.publishMQTTMeasurement(topic, s, data.particles_05um, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/partcount10um");
  node.publishMQTTMeasurement(topic, s, data.particles_10um, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/partcount25um");
  node.publishMQTTMeasurement(topic, s, data.particles_25um, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/partcount50um");
  node.publishMQTTMeasurement(topic, s, data.particles_50um, 0);

  strcpy(s, sensor);
  strcat(s, "/pmsa003i/partcount100um");
  node.publishMQTTMeasurement(topic, s, data.particles_100um, 0);
}


void publish_sen0321_measurements() {
  char s[strlen(sensor) + 64];

  int16_t ozoneConcentration = sen0321.readOzoneData(SEN0321_SAMPLES);

  strcpy(s, sensor);
  strcat(s, "/sen031/ozone");
  node.publishMQTTMeasurement(topic, s, ozoneConcentration, 0);
}


void publish_ms8607_measurements() {
  sensors_event_t temp, pressure, humidity;
  char s[strlen(sensor) + 64];

  ms8607.getEvent(&pressure, &temp, &humidity);

  strcpy(s, sensor);
  strcat(s, "/ms8607/temperature");
  node.publishMQTTMeasurement(topic, s, temp.temperature, 0);

  strcpy(s, sensor);
  strcat(s, "/ms8607/humidity");
  node.publishMQTTMeasurement(topic, s, humidity.relative_humidity, 0);

  strcpy(s, sensor);
  strcat(s, "/ms8607/pressure");
  node.publishMQTTMeasurement(topic, s, pressure.pressure, 0);
}


void publish_bme680_measurements() {
  char s[strlen(sensor) + 64];

  if (!bme680.performReading()) {
    Serial.println("[FAIL] Failed to perform BME680 reading.");
    blink_led(LED_FAIL, LED_FAST);
    return;
  }

  strcpy(s, sensor);
  strcat(s, "/bme680/temperature");
  node.publishMQTTMeasurement(topic, s, bme680.temperature, 0);

  strcpy(s, sensor);
  strcat(s, "/bme680/pressure");
  node.publishMQTTMeasurement(topic, s, bme680.pressure, 0);

  strcpy(s, sensor);
  strcat(s, "/bme680/humidity");
  node.publishMQTTMeasurement(topic, s, bme680.humidity, 0);

  strcpy(s, sensor);
  strcat(s, "/bme680/voc");
  node.publishMQTTMeasurement(topic, s, bme680.gas_resistance, 0);

  strcpy(s, sensor);
  strcat(s, "/bme680/altitude");
  node.publishMQTTMeasurement(topic, s, bme680.readAltitude(SEALEVELPRESSURE_HPA), 0);

  delay(2000);
}


void start_aspiration() {
  aspirator_relay.turnRelayOn();
  delay(aspiration_spinup_time * 1000);
}


void stop_aspiration() {
  delay(2000);
  aspirator_relay.turnRelayOff();
}


void setup() {
  File file;
  StaticJsonDocument<1024> doc;
  char uuid[32];
  uint16_t scd4x_error;
  bool i2c_device_connected = false;
  bool rg15_poe_bypass = false;
  String mac = String((uint32_t)ESP.getEfuseMac(), HEX);

  strcpy(uuid, "ESP32P_AtomLite_");
  strcat(uuid, (const char *)mac.c_str());

  // initialize serial
  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.print("[info] This is the IoTwx v");
  Serial.println(IOTWX_VERSION);
  delay(500);
  Serial.println("[info] initializing now ...");

  start_millis = millis();
  init_led();  // set up AtomLite LED

  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("[info] LittleFS mount failed ... HALTING");
    while (1) {
      delay(15000);
      blink_led(LED_FAIL, LED_FAST);
	 	  // reboot
	};
  }

  Serial.println("[info] LittleFS Mounted Successfully");
  file = LittleFS.open("/config.json", FILE_READ);

  if (file) {
    deserializeJson(doc, file);
    file.close();

    String output;
    serializeJson(doc, output);
    Serial.println("[info]: reading serialized json");
    Serial.println(output);

    if (output == "null") {
      Serial.println("[error]: config.json was empty: HALTING");
      while(1) {
        delay(15000);
        blink_led(LED_FAIL, LED_FAST);
        // reboot
      };
    } else if (doc.containsKey("iotwx_local_config")) {
	  // CONSIDER -> if the data already exists, don't store it again
      Serial.println("[info] LOCAL config storing to NVS");
      store_data_to_nvs("iotwx_mq_port", (const char *)doc["iotwx_mq_port"]);
      store_data_to_nvs("iotwx_mq_ip", (const char *)doc["iotwx_mq_ip"]);
      store_data_to_nvs("iotwx_wifi_ssid", (const char *)doc["iotwx_wifi_ssid"]);
      store_data_to_nvs("iotwx_wifi_pwd", (const char *)doc["iotwx_wifi_pwd"]);
      store_data_to_nvs("iotwx_id", (const char *)doc["iotwx_id"]);
      Serial.println("[info]: LOCAL config stored to NVS");

	  // initialize the node
	  node = IoTwx(true);
      Serial.println("[info]: IoTwx node initialized");
    }

	Serial.println("[info]: reading deserialized data");
	timezone = atoi((const char *)doc["iotwx_timezone"]);
	sensor = strdup((const char *)doc["iotwx_sensor"]);
	topic = strdup((const char *)doc["iotwx_topic"]);
	reset_interval = 1000 * 60 * atoi((const char *)doc["iotwx_reset_interval"]);
	publish_interval = atoi((const char *)doc["iotwx_publish_interval"]);
	max_frequency = atoi((const char *)doc["iotwx_max_frequency"]);
	atom_gpio_config = strdup((const char *)doc["iotwx_gpio_config"]);
	use_wifi = atoi((const char *)doc["iotwx_use_wifi"]);
	aspiration_spinup_time = atoi((const char *)doc["iotwx_aspiration_spinup_time"]);
	light_sensitivity = doc["iotwx_light_sensitivity"] ? atoi((const char *)doc["iotwx_light_sensitivity"]) : 1; // default tsl is 1
    
	// set wifi or POE
    node.setWifi(use_wifi == 1);

    if (!use_wifi) {
      byte poe_mac[] = { 0x02, 0xAD, 0x74, 0x7B, 0xED, 0x2B };
      node.setPoEMAC(poe_mac);
      Serial.print("[info]: POE mode with MAC (");
      Serial.print("");
      Serial.println(")");
    }

    Serial.println();

    // initialize the I2C bus
    if (strcmp(atom_gpio_config, "A") == 0) {
      atomUART.begin(9600, SWSERIAL_8N1, 32, 26);
      rg15.setStream(&atomUART);

      Serial.println("[info]: GPIO_config is A\n[info]: OK Found RG15 on Grove, using pins 21,25 for I2C");
      blink_led(LED_OK, LED_SLOW);
      rg15_attached = true;

      // we can allow rg15 connectivity to be a bypass condition
      if (!use_wifi) {
        rg15_poe_bypass = true;
      }
      Serial.print("[info]: M5Stack POE bypass (RG15) : ");
      Serial.println(rg15_poe_bypass);

      // set i2c to other pins on gpio
      Wire.begin(25, 21, 10000);
    } else {
      Serial.println("[info]: GPIO_config is not A, using pins 26,32 (Grove) for I2C connections");

      Wire.begin(26, 32, 10000);
      Serial.println("");
    }

    // set up aspiration fan
    aspirated = aspirator_relay.begin();
    if (aspirated) stop_aspiration();
    if (aspirated) Serial.println("[info]: OK aspiration fan found");

    // check for i2c device connectivity, once then check for rg15 bypass
    do {
      /// Adafruit bme680 TPHVOC >> https://www.adafruit.com/product/3660
      if (!bme680.begin()) {
        Serial.println("[warn]: Could not find Adafruit BME680 sensor. Check your connections and verify the address 0x76 is correct.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        bme680_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit BME680");
        blink_led(LED_OK, LED_SLOW);

        // Set up oversampling and filter initialization
        bme680.setTemperatureOversampling(BME680_OS_8X);
        bme680.setHumidityOversampling(BME680_OS_2X);
        bme680.setPressureOversampling(BME680_OS_4X);
        bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme680.setGasHeater(320, 150);  // 320*C for 150 ms
      }

      /// Adafruit ms8607 TPH >> https://www.adafruit.com/product/4716
      if (bme680_attached || !ms8607.begin()) {
        Serial.println("[warn]: Could not find Adafruit MS8607 sensor or BME680 is attached already on address 0x76. Check your connections and verify the address 0x76 is correct.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        ms8607_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit MS8607");
        blink_led(LED_OK, LED_SLOW);

        ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_12b);
        ms8607.setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096);
      }

      /// Adafruit hdc3022 TH >> https://www.adafruit.com/product/5989
      if (!hdc3022.begin(HDC302X_IIC_ADDR, &Wire)) {
        Serial.println("[warn]: Could not find Adafruit HDC3022 sensor. Check your connections and verify the address 0x44 is correct.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
          hdc3022_attached = true;
          i2c_device_connected = true;

          Serial.println("[info]: OK Found Adafruit HDC3022");
          blink_led(LED_OK, LED_SLOW);
      }

      /// Adafruit sht41 TH >> https://www.adafruit.com/product/5776
      if (hdc3022_attached || !sht4.begin()) {
        Serial.println("[warn]: Could not find Adafruit SHT4X Adafruit Temp/Humidity sensor or HDC3022 is already attached on Address 0x44. Check your connections and verify the address 0x44 is correct.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        sht4.setHeater(SHT4X_NO_HEATER);
        sht4.setPrecision(SHT4X_HIGH_PRECISION);

        sht4x_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit SHT4x");
        blink_led(LED_OK, LED_SLOW);
      }

      /// Adafruit pmsa003i pm25/aqi >> https://www.adafruit.com/product/4632
      if (!aqi.begin_I2C()) {
        Serial.println("[warn]: Could not find Adafruit PMSA003I AQ sensor. Check your connections and verify the address 0x12 is correct.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        pm25aqi_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit PM25AQI");
        blink_led(LED_OK, LED_SLOW);
      }

      /// Adafruit scd41 CO2 >> https://www.adafruit.com/product/5190
      scd4x.begin(Wire, SCD41_I2C_ADDR_62);
      scd4x_error = scd4x.stopPeriodicMeasurement();
      if (scd4x_error) {
        Serial.println("[warn]: Could not find Adafruit SCD4x. Error trying to execute stopPeriodicMeasurement().");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        scd4x_attached = true;
        i2c_device_connected = true;

        Serial.println("[info]: OK Found Adafruit SCD4x");
        blink_led(LED_OK, LED_SLOW);
      }

      /// Adafruit ltr390 uv300-350nm >> https://www.adafruit.com/product/4831
      if (!ltr.begin()) {
        Serial.println("[warn]: Could not find Adafruit LTR390 sensor.");
        blink_led(LED_FAIL, LED_FAST);
      } else {
        Serial.println("[info]: OK Found LTR390 sensor");
		blink_led(LED_OK, LED_SLOW);

        ltr.setResolution(LTR390_RESOLUTION_16BIT);
        ltr.setGain(LTR390_GAIN_3);
        ltr.setMode(LTR390_MODE_UVS);
        ltr.setThresholds(100, 1000);

        ltr390_attached = true;
        i2c_device_connected = true;
      }

	  /// Adafruit TSL2591 light sensor
	  if (!tsl2591.begin())
	  {
		Serial.println("[warn]: Could not find Adafruit TSL2591 sensor.");
        blink_led(LED_FAIL, LED_FAST);
	  } else {
		blink_led(LED_OK, LED_SLOW);
        Serial.println("[info]: OK Found Adafruit TSL2591 sensor");

		switch (light_sensitivity) {
			case 0:
				tsl2591.setGain(TSL2591_GAIN_LOW); // 1x gain  
				tsl2591.setTiming(TSL2591_INTEGRATIONTIME_100MS); // city skies
				Serial.println("[info]: TSL2591 sensivity set to LOW gain + 100ms integration : URBAN SKY MODE");
				break;

			case 1:
				tsl2591.setGain(TSL2591_GAIN_MED); // 25x gain  
				tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS); // suburban skies
				Serial.println("[info]: TSL2591 sensivity set to MED gain + 300ms integration : SUBURBAN SKY MODE");
				break;

			case 2:
				tsl2591.setGain(TSL2591_GAIN_HIGH); // 428x gain  
				tsl2591.setTiming(TSL2591_INTEGRATIONTIME_500MS); // rural skies
				Serial.println("[info]: TSL2591 sensivity set to HI gain + 500ms integration : RURAL SKY MODE");
				break;

			case 3:
				tsl2591.setGain(TSL2591_GAIN_MAX); // 9876x gain  
				tsl2591.setTiming(TSL2591_INTEGRATIONTIME_600MS); // DARK skies
				Serial.println("[info]: TSL2591 sensivity set to MAX gain + 600ms integration : DARK SKY MODE");
				break;
			
			default:
				tsl2591.setGain(TSL2591_GAIN_MED); // 25x gain  
				tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS); // suburban skies
				Serial.println("[info]: TSL2591 sensivity set to MED gain + 300ms integration : SUBURBAN SKY MODE");
				break;

		}

		tsl2591_attached = true;
		i2c_device_connected  = true;
	  }

      /// DF Robot sen0321 ozone >> https://wiki.dfrobot.com/Gravity_IIC_Ozone_Sensor_(0-10ppm)%20SKU_SEN0321
      int retry_count = 0;
      while (true) {
        if (retry_count < 5) {
          if (!sen0321.begin(SEN0321_IIC_ADDR)) {
            delay(1000);
            retry_count++;
          } else {
            sen0321_attached = true;
            i2c_device_connected = true;

            Serial.println("[info]: OK Found DF Robot SEN0321 sensor");
            sen0321.setModes(MEASURE_MODE_PASSIVE);
            break;
          }
        } else {
          Serial.println("[warn]: Could not find DF Robot SEN0321 sensor.");
          break;
        }
      }
    } while (false);

    delay(1000);

    // begin shutdown sequence, downthrottle, shutdown wifi and BT
    btStop();
    Serial.println("[info]: BT disconnected for power reduction");
    setCpuFrequencyMhz(max_frequency);
    Serial.println();
    Serial.print("[info] CPU downthrottled to ");
    Serial.print(max_frequency);
    Serial.println("Mhz for power reduction");
    WiFi.mode(WIFI_OFF);
    Serial.println("[info] Wifi shut off for power reduction");
	  delay(1500);
  } else {
    Serial.println("[halt]: halting > configuration corrupt or missing");
    while (1){
		delay(15000);
		// RESTART
	}
   }
}


void loop() {
  if (millis() - last_millis > publish_interval * 60 * 1000) {
    Serial.println("[info]: measuring");

    last_millis = millis();

    // start measurements
    if (aspirated) start_aspiration();

    // connect to internet -> NOTE: ASPIRATION MUST OCCUR BEFORE THIS; MESSAGES WILL NOT RELAY; UNSURE WHY
    node.establishCommunications();

    // start measurements that are aspiration dependent
    if (bme680_attached)  publish_bme680_measurements();
    if (ms8607_attached)  publish_ms8607_measurements();
    if (sht4x_attached)   publish_sht4x_measurements();
    if (hdc3022_attached) publish_hdc3022_measurements();

    // stop fan
    if (aspirated) stop_aspiration();

    if (pm25aqi_attached) publish_pmsa0031_measurements();
    if (scd4x_attached)   publish_scd4x_measurements();
    if (ltr390_attached)  publish_ltr390_measurements();
    if (rg15_attached)    publish_rg15_measurements();
    if (sen0321_attached) publish_sen0321_measurements();
	if (tsl2591_attached) publish_tsl2591_measurements();

    // configure the timer to wake us up!
    delay(1000);
  }

  if (millis() - start_millis > reset_interval) esp_restart();

  Serial.println("[info]: sleeping");
  esp_sleep_enable_timer_wakeup(publish_interval * 60L * 1000000L);
  esp_light_sleep_start();
}
