#ifndef _TRACKDOCK_MQTT_CLIENT_H_
#define _TRACKDOCK_MQTT_CLIENT_H_

#include <string>

void InitTrackDockMqttClient();
void TrackDockMqtt_PublishState();
void TrackDockMqtt_PublishTelemetry();
void TrackDockMqtt_PublishCargoJson(const std::string& cargo_json);
void TrackDockMqtt_PublishDialog(const char* text);
void TrackDockMqtt_PublishEvent(const char* text);

#endif // _TRACKDOCK_MQTT_CLIENT_H_