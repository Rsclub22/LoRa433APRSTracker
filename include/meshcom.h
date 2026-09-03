// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCom position beacon
// ============================================================

#pragma once

#include <Arduino.h>
#include "config_file.h"

namespace MeshCom {

// Build and transmit one MeshCom position frame. Retunes the radio to the
// MeshCom profile, transmits, and returns it to APRS.
bool sendPosition(const TrackerConfig &cfg, const char *callsign,
                  float lat, float lon, float altMeters, int8_t drive);

// Build and transmit one MeshCom private message to a target callsign.
// Same retune, same return to APRS.
bool sendMessage(const TrackerConfig &cfg, const char *callsign,
                 const char *target, const char *text, int8_t drive);

}  // namespace MeshCom
