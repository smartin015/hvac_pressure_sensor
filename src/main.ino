// https://github.com/espressif/arduino-esp32/blob/master/libraries/Zigbee/examples/Zigbee_Pressure_Flow_Sensor/Zigbee_Pressure_Flow_Sensor.ino

#include <Arduino.h>
#include <Preferences.h>
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"

/* Zigbee pressure sensor configuration */
#define PRESSURE_SENSOR_ENDPOINT_NUMBER 11
#define DIFF_PRESSURE_SENSOR_PIN D2

uint8_t button = BOOT_PIN;

ZigbeePressureSensor zbPressureSensor = ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);
ZigbeeAnalog zbAnalogAvg = ZigbeeAnalog(PRESSURE_SENSOR_ENDPOINT_NUMBER + 1);
ZigbeeAnalog zbAnalogZeroPoint = ZigbeeAnalog(PRESSURE_SENSOR_ENDPOINT_NUMBER + 2);
ZigbeeAnalog zbAnalogSamplesPerKPA = ZigbeeAnalog(PRESSURE_SENSOR_ENDPOINT_NUMBER + 3);


Preferences preferences;
#define PREFS_NS "config"
uint32_t zero_point;
uint32_t samples_per_kpa;

void onZeroPointChange(float v) {
  Serial.printf("Received zero point change: %.1f\r\n", v);
  zero_point = v;
  preferences.begin(PREFS_NS, false);
  preferences.putUInt("zero_point", v);
  preferences.end();
}

void maybeReset() {
  // Checking button for factory reset and reporting
  if (digitalRead(button) == LOW) {  // Push button pressed
    Serial.println("Reset button pressed; waiting for hold");
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      if ((millis() - startTime) > 3000) {
        // If key pressed for more than 3secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        Zigbee.factoryReset();
      }
    }
    //zbPressureSensor.report();
  }
}

void onSamplesPerKPAChange(float v) {
  Serial.printf("Received samples per kPa change: %.1f\r\n", v);
  samples_per_kpa = v;
  preferences.begin(PREFS_NS, false);
  preferences.putUInt("samples_per_kpa", v);
  preferences.end();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // Switch to external antenna
  digitalWrite(WIFI_ANT_CONFIG, HIGH);
  //pinMode(3, OUTPUT); digitalWrite(3, LOW);
  //pinMode(14, OUTPUT); digitalWrite(14, HIGH);


  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 20; i++) {
    maybeReset();
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  digitalWrite(LED_BUILTIN, LOW);
  preferences.begin(PREFS_NS, false);
  zero_point = preferences.getUInt("zero_point", 2048);
  samples_per_kpa = preferences.getUInt("samples_per_kpa", 1024);
  preferences.end();
  Serial.print("Stored values: zero=");
  Serial.print(zero_point);
  Serial.print(", s/kpa=");
  Serial.println(samples_per_kpa);


  pinMode(button, INPUT_PULLUP);
  pinMode(DIFF_PRESSURE_SENSOR_PIN, INPUT);
  zbPressureSensor.setManufacturerAndModel("GreenHouse", "FurnaceFilter");
  zbPressureSensor.setMinMaxValue(-2000, 2000); // value is Pa, reported as hPa
  zbPressureSensor.setTolerance(0.0122); // 4096 samples/5V -> 3276 samples/4V... 4V range of 40 hPa -> 40/3276 => ~0.0122 hPa
  Zigbee.addEndpoint(&zbPressureSensor);

  zbAnalogAvg.addAnalogInput();
  zbAnalogAvg.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogAvg.setAnalogInputDescription("Averaged sensor reading, 0-4095");
  zbAnalogAvg.setAnalogInputResolution(1);
  Zigbee.addEndpoint(&zbAnalogAvg);

  zbAnalogZeroPoint.addAnalogOutput();
  zbAnalogZeroPoint.setAnalogOutputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogZeroPoint.setAnalogOutputDescription("Analog sensor value at 0Pa");
  zbAnalogZeroPoint.setAnalogOutputResolution(1);
  zbAnalogZeroPoint.setAnalogOutputMinMax(0, 4095);
  zbAnalogZeroPoint.onAnalogOutputChange(onZeroPointChange);
  Zigbee.addEndpoint(&zbAnalogZeroPoint);

  zbAnalogSamplesPerKPA.addAnalogOutput();
  zbAnalogSamplesPerKPA.setAnalogOutputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
  zbAnalogSamplesPerKPA.setAnalogOutputDescription("Analog sensor samples per kPa");
  zbAnalogSamplesPerKPA.setAnalogOutputResolution(1);
  zbAnalogSamplesPerKPA.setAnalogOutputMinMax(0, 4095);
  zbAnalogSamplesPerKPA.onAnalogOutputChange(onSamplesPerKPAChange);
  Zigbee.addEndpoint(&zbAnalogSamplesPerKPA);

  delay(2000);
  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  } else {
    Serial.println("Zigbee started successfully!");
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    maybeReset();
    Serial.print(".");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.println();
  delay(1000);

  // Set reporting interval for pressure measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (pressure change in hPa)
  // if min = 1 and max = 0, reporting is sent only when value changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or value changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of delta change
  
  // Report every 10-60 seconds if the value changes by 1
  zbPressureSensor.setReporting(10, 60, 1);
  zbAnalogAvg.setAnalogInputReporting(10, 60, 1);

  zbAnalogZeroPoint.setAnalogOutput(zero_point);
  zbAnalogZeroPoint.reportAnalogOutput();
  zbAnalogSamplesPerKPA.setAnalogOutput(samples_per_kpa);
  zbAnalogSamplesPerKPA.reportAnalogOutput();
}

int32_t pressure_avg = 2970;
void loop() {
  static uint32_t timeCounter = 0;
  // Read pressure sensor every 2s
  if (!(timeCounter++ % 5)) {  // delaying for 100ms x 5= 0.5s
    int32_t pressure_raw = analogRead(DIFF_PRESSURE_SENSOR_PIN);
    pressure_avg = int(
      (0.9 * float(pressure_avg)) + 
      (0.1 * float(pressure_raw))
    );
    // Empirical readings: ~2970 is 0 difference
    // Mouth pressure between 22 and 3907
    // Note that reporting is now in Pascal, not hPa as advertised
    // Datasheet: 1V/kPa
    // Voltage 0=0V, 4095=5V, so 819 = 1V
    // Low value = -20hPa = 2970 - 2*819 = 1332
    // High value = 20hPa = 2970 + 2*819 = 4608
    uint32_t inLow = zero_point - (2.0 * samples_per_kpa);
    uint32_t inHigh = zero_point + (2.0 * samples_per_kpa);
    int32_t pressure_value_pascal = (uint32_t) map(pressure_avg, inLow, inHigh, -2000, 2000);
    Serial.printf("READ analog %d avg %d low %d high %d -> %d Pascal\r\n", pressure_raw, pressure_avg, inLow, inHigh, pressure_value_pascal);
    zbPressureSensor.setPressure(pressure_value_pascal);
    zbAnalogAvg.setAnalogInput(pressure_avg);
  }
  maybeReset();
  delay(100);
}
