/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/

///////////////////////////////////////////////////////////////////////
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

///////////////////////////////////////////////////////////////////////
#define WIFI_SSID "ssid"
#define WIFI_PASS "password"

///////////////////////////////////////////////////////////////////////
#define MQTT_HOST IPAddress(192, 168, 1, 30)
#define MQTT_PORT 1883

///////////////////////////////////////////////////////////////////////
#define DEBOUNCE 20
#define BUTTON_PIN 34
#define BELL_PIN 22

///////////////////////////////////////////////////////////////////////
AsyncMqttClient mqtt_client;
TimerHandle_t mqtt_reconnect_timer;
TimerHandle_t wifi_reconnect_timer;

///////////////////////////////////////////////////////////////////////
static void wifi_connect()
{
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void mqtt_connect()
{
  mqtt_client.connect();
}

static void WiFiEvent(WiFiEvent_t event)
{
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        mqtt_connect();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStop(mqtt_reconnect_timer, 0);
        xTimerStart(wifi_reconnect_timer, 0);
        break;
    }
}

static void on_mqtt_disconnect(AsyncMqttClientDisconnectReason reason)
{
  if ( WiFi.isConnected() )
    xTimerStart(mqtt_reconnect_timer, 0);
}

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BELL_PIN, OUTPUT);
  digitalWrite(BELL_PIN, LOW);

  mqtt_reconnect_timer = xTimerCreate("mqtt_timer", pdMS_TO_TICKS(2000), pdFALSE, NULL, reinterpret_cast<TimerCallbackFunction_t>(mqtt_connect));
  wifi_reconnect_timer = xTimerCreate("wifi_timer", pdMS_TO_TICKS(2000), pdFALSE, NULL, reinterpret_cast<TimerCallbackFunction_t>(wifi_connect));

  WiFi.onEvent(WiFiEvent);

  mqtt_client.onDisconnect(on_mqtt_disconnect);
  mqtt_client.setServer(MQTT_HOST, MQTT_PORT);

  wifi_connect();
}

static uint8_t count;
void loop()
{
  delay(1);
  uint8_t is_down = (digitalRead(BUTTON_PIN) == HIGH);
  int8_t increment = 0;
  if (count == 0)
    increment = (is_down ? +1 :  0);
  else if (count == DEBOUNCE)
    increment = (is_down ?  0 : -1);
  else
    increment = (is_down ? +1 : -1);
  if (increment != 0)
  {
    count += increment;
    if (count == DEBOUNCE)
    {
      digitalWrite(BELL_PIN, HIGH);
      mqtt_client.publish("doorbell", 0, false, "1");
    }
    else if (count == 0)
    {
      digitalWrite(BELL_PIN, LOW);
      mqtt_client.publish("doorbell", 0, false, "0");
    }
  }
}
