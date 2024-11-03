#include <Arduino.h>
#include <FreeRTOS.h>
#include <HADevice.h>
#include <HAMqtt.h>
#include <WiFi.h>
#include <device-types/HASensor.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <string_view>
#include <utility>

WiFiClient client;
HADevice ha_device("wsta");
HAMqtt mqtt(client, ha_device);

constexpr auto kWindDirectionPin = A0;

std::string_view windvane_level_to_direction(int32_t adc_reading) {
  // ADC target levels assume a divider impedance of 3377 ohms, which can be
  // achieved by putting a 5100 ohm and 10000 ohm resistor in parallel.
  constexpr std::array<std::pair<std::string_view, int32_t>, 16> adc_targets{
      // New: I have the divider, then the ADC, then the windvane, then ground.
      {std::make_pair("NE", 2901), std::make_pair("E", 936),
       std::make_pair("SE", 1616), std::make_pair("S", 2195),
       std::make_pair("SW", 3382), std::make_pair("W", 3984),
       std::make_pair("NW", 3893), std::make_pair("N", 3716),
       std::make_pair("NNE", 2705), std::make_pair("ENE", 855),
       std::make_pair("ESE", 693), std::make_pair("SSE", 1204),
       std::make_pair("SSW", 1972), std::make_pair("WSW", 3305),
       std::make_pair("WNW", 3792), std::make_pair("NNW", 3548)}};
  int min_diff = std::numeric_limits<int>::max();
  int min_index = -1;
  for (int i = 0; i < adc_targets.size(); ++i) {
    const int diff = std::abs(adc_targets[i].second - adc_reading);
    if (min_diff > diff) {
      min_diff = diff;
      min_index = i;
    }
  }
  return adc_targets[min_index].first;
}

HASensor ha_windvane("wv");

void sample_windvane(TimerHandle_t) {
  const uint16_t wind_voltage = analogRead(kWindDirectionPin);
  const std::string_view direction = windvane_level_to_direction(wind_voltage);
  ha_windvane.setValue(direction.data());
}

void SetupWindvane() {
  ha_windvane.setName("Windvane");
  ha_windvane.setDeviceClass("enum");
  ha_windvane.setForceUpdate(true);
  ha_windvane.setJsonAttributes(
      "options: \"{{['NE', 'E', 'SE', 'S', 'SW', 'W', 'NW', 'N', 'NNE', "
      "'ENE', 'ESE', 'SSE', 'SSW', 'WSW', 'WNW', 'NNW']}}\" ");

  analogReadResolution(12);
  xTimerStart(xTimerCreate("windvane_reporter", pdMS_TO_TICKS(4997), true,
                           nullptr, sample_windvane),
              portMAX_DELAY);
}

// A rate limited counter. It can be incremented at most once per update_period.
// If an increment is received in less time than the update period from the
// previous, it is a noop. This counter is not thread safe, so anything that
// is flushing the counter needs to ensure that the counter is not being
// incremented at the same time.
struct RateLimitedCounter {
  const int32_t update_period;
  uint32_t next_update = 0;
  int32_t count = 0;

  void Inc() {
    const uint32_t now = millis();
    if (static_cast<int32_t>(next_update - now) > 0) return;
    ++count;
    next_update = now + update_period;
  }

  // Sets the counter value to zero and returns the current value.
  int32_t Flush() { return std::exchange(count, 0); }
};

// Counts each time a pin goes low, subject to a maximum rate. Multiplies
// the count by a scale and reports it to an HASensor periodically.
class ScaledSamplePinCounter {
 public:
  static constexpr int32_t CalcCounterUpdatePeriodMs(float scale,
                                                     float max_value,
                                                     float period_length_s) {
    const float max_ticks_per_period = max_value / scale;
    const float max_ticks_per_s = max_ticks_per_period / period_length_s;
    return 1000 / max_ticks_per_s;
  }

  ScaledSamplePinCounter(int pin, float scale, float max_value, float period_s,
                         HASensor& destination)
      : pin_(pin),
        scale_(scale),
        period_s_(period_s),
        counter_{.update_period =
                     CalcCounterUpdatePeriodMs(scale, max_value, period_s)},
        dest_(destination) {}

  ~ScaledSamplePinCounter() {
    xTimerDelete(flush_timer_, portMAX_DELAY);
    detachInterrupt(pin_);
  }

  // begin() sets up interrupts and a timer to flush the accumulated counter
  // increments to MQTT.
  void begin() {
    // Each time the pin goes low, increment the counter. The counter
    // naturally debounces due to the maximum update frequency.
    pinMode(pin_, INPUT_PULLUP);

    Serial1.printf("attaching interrupted to %d\n", pin_);
    attachInterruptParam(
        pin_,
        +[](void* counter) {
          Serial1.print("interrupt!\n");
          reinterpret_cast<RateLimitedCounter*>(counter)->Inc();
        },
        FALLING, &counter_);

    // Schedule a timer to update the sensor's value with the counter's value.
    flush_timer_ = xTimerCreate(
        "counter_flusher", pdMS_TO_TICKS(period_s_ * 1000), true, this,
        +[](TimerHandle_t timer) {
          reinterpret_cast<ScaledSamplePinCounter*>(pvTimerGetTimerID(timer))
              ->FlushCounterValueToSensor();
        });
    xTimerStart(flush_timer_, portMAX_DELAY);
  }

  void FlushCounterValueToSensor() {
    int32_t ticks;
    taskENTER_CRITICAL();
    ticks = counter_.Flush();
    taskEXIT_CRITICAL();
    const float value = ticks * scale_;
    char buffer[32];
    configASSERT(snprintf(buffer, sizeof(buffer), "%.3f", value) < 32);
    dest_.setValue(buffer);
  }

  const int pin_;
  const float scale_;
  const float period_s_;
  RateLimitedCounter counter_;
  TimerHandle_t flush_timer_;
  HASensor& dest_;
};

HASensor anemometer("an");

constexpr int kWindPin = 14;
// I.e. if the anemometer ticks once per second then the windspeed is 1.73
// mph. We (falsely) assume a linear relationship between ticks and speed,
// and therefore compute the average simply by scaling the speed down by the
// number of seconds over which we're reporting.
constexpr float kWindSpeedPerTickPerSecond = 1.73;  // mph
constexpr float kWindReportPeriodS = 5;
constexpr float kWindScale = kWindSpeedPerTickPerSecond / kWindReportPeriodS;
constexpr float kWindMaxSpeed = 100;

ScaledSamplePinCounter wind_counter(kWindPin, kWindScale, kWindMaxSpeed,
                                    kWindReportPeriodS, anemometer);

void SetupAnemometer() {
  anemometer.setName("Anemometer");
  anemometer.setUnitOfMeasurement("mph");
  anemometer.setDeviceClass("wind_speed");
  anemometer.setStateClass("measurement");
  anemometer.setForceUpdate(true);
}

HASensor rain_sensor("rg");

// In the case of the rain gauge, the total amount of rain is simply the
// number of times the gauge has ticked.
constexpr int kRainPin = 15;
constexpr float kRainSpeedPerTickPerSecond = 0.011;  // inches of rain
constexpr float kRainReportPeriodS = 15 * 60;
constexpr float kRainScale = 1.;
// We will limit our report rate to 12 inches per hour.
// If we get more rain than that the weatherstation will certainly not be
// around any more to report it.
constexpr float kRainMaxValue = 12. / 4;

ScaledSamplePinCounter rain_counter(kRainPin, kRainScale, kRainMaxValue,
                                    kRainReportPeriodS, rain_sensor);

void SetupRainGauge() {
  rain_sensor.setName("Rainfall");
  rain_sensor.setUnitOfMeasurement("in");
  rain_sensor.setDeviceClass("precipitation");
  rain_sensor.setStateClass("total_increasing");
  rain_sensor.setForceUpdate(true);
}

#define SSTR(s) (#s)
#define STR(s) SSTR(s)

void setup() {
  ha_device.setName("Weatherstation");
  ha_device.enableExtendedUniqueIds();

  Serial1.begin();
  WiFi.setHostname("weatherstation");
  Serial1.println("WiFi.begin");
  WiFi.begin(STR(WIFI_SSID), STR(WIFI_PASSWORD));

  while (WiFi.status() != WL_CONNECTED) {
    Serial1.printf("Waiting for connection to %s\n", STR(WIFI_SSID));
    delay(1000);
  }

  Serial1.println("WiFi ready");

  SetupWindvane();
  SetupRainGauge();
  SetupAnemometer();
  ha_device.enableSharedAvailability();
  ha_device.enableLastWill();
  ha_device.setAvailability(true);

  IPAddress mqtt_addr;
  if (!mqtt_addr.fromString(STR(MQTT_HOST))) {
    Serial1.printf("%s is not a valid IP address\n", STR(MQTT_HOST));
    return;
  }
  Serial1.printf("mqtt.begin %s\n", mqtt_addr.toString().c_str());
  mqtt.begin(mqtt_addr, STR(MQTT_USER), STR(MQTT_PASSWORD));

  static auto mqtt_lock = xSemaphoreCreateMutex();
  xTaskCreate(
      [](void*) {
        while (true) {
          xSemaphoreTake(mqtt_lock, portMAX_DELAY);
          mqtt.loop();
          xSemaphoreGive(mqtt_lock);
          delay(5);
        }
      },
      "mqtt_loop", 1024, nullptr, 1, nullptr);

  xSemaphoreTake(mqtt_lock, portMAX_DELAY);
  while (!mqtt.isConnected()) {
    xSemaphoreGive(mqtt_lock);
    delay(1000);
    xSemaphoreTake(mqtt_lock, portMAX_DELAY);
  }
  xSemaphoreGive(mqtt_lock);

  Serial1.printf("MQTT up\n");

  // Something in the MQTT or LWIP code causes our interrupt handlers to be
  // deleted if we set them up before starting MQTT. If we set them up after
  // then it's all good.
  wind_counter.begin();
  rain_counter.begin();
};

void loop() { delay(10000); }