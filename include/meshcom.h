// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCom position beacon
// ============================================================

#pragma once

#include <Arduino.h>
#include "config_file.h"

namespace MeshCom {

// Derive this node's message-id space from the callsign, and pick a starting
// counter that differs from the last boot's. Both matter: a receiving node
// deduplicates on the message id alone, so ids that repeat are dropped
// without a trace. Call once from setup().
void begin(const char *callsign, uint16_t seed);

// Build and transmit one MeshCom position frame. Retunes the radio to the
// MeshCom profile, transmits, and returns it to APRS.
// track marks the position as movement-driven rather than a timer tick,
// the way upstream does for anything sent off its POSINFO interval.
bool sendPosition(const TrackerConfig &cfg, const char *callsign,
                  float lat, float lon, float altMeters, int8_t drive,
                  bool track = false);

// Build and transmit one MeshCom private message to a target callsign.
// Same retune, same return to APRS.
bool sendMessage(const TrackerConfig &cfg, const char *callsign,
                 const char *target, const char *text, int8_t drive);

}  // namespace MeshCom
