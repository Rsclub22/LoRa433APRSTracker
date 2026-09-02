// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCom position beacon
// ============================================================

#include "meshcom.h"

#include <hardware/watchdog.h>

#include "pins.h"
#include "radio.h"

namespace {

static uint32_t meshComMsgId = 1;

static uint8_t meshComModulation(const TrackerConfig &cfg) {
    const float bw = cfg.meshComBandwidth;
    const int sf = cfg.meshComSf;
    const int cr = cfg.meshComCr;

    uint8_t mod = 3;  // medium
    if (sf == 12 && cr == 8 && fabsf(bw - 125.0f) < 0.01f) mod = 4;
    if (sf == 12 && cr == 6 && fabsf(bw - 125.0f) < 0.01f) mod = 5;
    if (sf == 10 && cr == 6 && fabsf(bw - 125.0f) < 0.01f) mod = 6;
    if (sf == 11 && cr == 5 && fabsf(bw - 250.0f) < 0.01f) mod = 7;
    if (sf == 11 && cr == 6 && fabsf(bw - 250.0f) < 0.01f) mod = 8;
    return mod;
}

static bool meshComPayload(char *out, size_t outLen, const TrackerConfig &cfg,
                           float lat, float lon, float altMeters) {
    if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) return false;

    char symTable = '/';
    char symCode = '#';
    if (strlen(cfg.symbol) >= 2) {
        symTable = cfg.symbol[0];
        symCode = cfg.symbol[1];
    }

    char latHem = lat >= 0 ? 'N' : 'S';
    char lonHem = lon >= 0 ? 'E' : 'W';
    float absLat = fabsf(lat);
    float absLon = fabsf(lon);
    int latDeg = (int)absLat;
    int lonDeg = (int)absLon;
    float latMin = (absLat - latDeg) * 60.0f;
    float lonMin = (absLon - lonDeg) * 60.0f;
    float latAprs = latDeg * 100.0f + latMin;
    float lonAprs = lonDeg * 100.0f + lonMin;

    int altFeet = (int)lroundf(altMeters * 3.2808399f);
    if (altFeet < 0) altFeet = 0;

    snprintf(out, outLen, "%07.2f%c%c%08.2f%c%c/A=%06d",
             latAprs, latHem, symTable, lonAprs, lonHem, symCode, altFeet);
    return true;
}

}  // namespace

bool MeshCom::sendPosition(const TrackerConfig &cfg, const char *callsign,
                           float lat, float lon, float altMeters, int8_t drive) {
    if (!callsign || !callsign[0]) return false;

    char payload[96];
    if (!meshComPayload(payload, sizeof(payload), cfg, lat, lon, altMeters)) return false;

    char aprs[140];
    snprintf(aprs, sizeof(aprs), "%s>*!%s", callsign, payload);

    uint8_t frame[255];
    size_t n = 0;

    frame[n++] = 0x21;  // position
    uint32_t msgId = meshComMsgId++;
    frame[n++] = (uint8_t)(msgId & 0xFF);
    frame[n++] = (uint8_t)((msgId >> 8) & 0xFF);
    frame[n++] = (uint8_t)((msgId >> 16) & 0xFF);
    frame[n++] = (uint8_t)((msgId >> 24) & 0xFF);
    frame[n++] = (uint8_t)((cfg.meshComMaxHop & 0x0F) | 0x10);

    size_t aprsLen = strnlen(aprs, sizeof(aprs));
    if (n + aprsLen + 10 >= sizeof(frame)) return false;
    memcpy(frame + n, aprs, aprsLen);
    n += aprsLen;

    frame[n++] = 0x00;
    uint8_t hw = (uint8_t)cfg.meshComHardwareId;
    frame[n++] = hw;
    frame[n++] = meshComModulation(cfg);

    uint16_t fcs = 0;
    for (size_t i = 0; i < n; i++) fcs = (uint16_t)(fcs + frame[i]);
    frame[n++] = (uint8_t)((fcs >> 8) & 0xFF);
    frame[n++] = (uint8_t)(fcs & 0xFF);

    frame[n++] = 2;
    frame[n++] = (uint8_t)(0x80 | hw);
    frame[n++] = 0x23;  // '#'
    frame[n++] = 0x7E;

    char err[64];
    bool ok = false;
    if (TrackerRadio::setMode(RADIO_MODE_MESHCOM, cfg, err, sizeof(err))) {
        TrackerRadio::setDrive(drive);
        bool needPa = cfg.hasPa || TrackerRadio::hasModulePa();
        if (needPa) { digitalWrite(PIN_PA, HIGH); delay(250); watchdog_update(); }
        ok = TrackerRadio::send(frame, n);
        if (needPa) { delay(100); digitalWrite(PIN_PA, LOW); }
    }
    if (!TrackerRadio::setMode(RADIO_MODE_APRS, cfg, err, sizeof(err))) {
        Serial.printf("\x1b[1;5;31mRADIO STUCK OFF-PROFILE: %s\x1b[0m\r\n", err);
    }
    return ok;
}
