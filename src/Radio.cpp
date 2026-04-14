#include "Radio.h"
 
Radio::Radio(uint8_t csPin, uint8_t intPin):
    _rfm9x(csPin, intPin),
    _ready(false),
    _txIntervalMs(250),
    _lastTxTime(0)
{}
 
bool Radio::begin(float frequencyMHz, int8_t txPowerDbm) {
    if (!_rfm9x.init()) {
        return false;
    }

    if (!_rfm9x.setFrequency(frequencyMHz)) {
        return false;
    }

    _rfm9x.setTxPower(txPowerDbm, false); // false = use PA_BOOST pin

    _rfm9x.setModemConfig(RH_RF95::Bw125Cr45Sf128);

    _ready = true;
    return true;
}
 
bool Radio::sendTelemetry(const TelemetryPacket &pkt) {
    if (!_ready) {
        return false;
    }

    unsigned long now = millis();
    if (now - _lastTxTime < _txIntervalMs) {
        return false;
    }

    TelemetryPacket out = pkt;
    out.crc = _computeCRC(reinterpret_cast<const uint8_t *>(&out),
                          sizeof(TelemetryPacket) - 1);

    bool ok = _rfm9x.send(reinterpret_cast<const uint8_t *>(&out),
                         sizeof(TelemetryPacket));

    if (ok) {
        _lastTxTime = now;
    }

    return ok;
}
 
void Radio::setTxInterval(unsigned long ms) {
    _txIntervalMs = ms;
}
 
bool Radio::isReady() const {
    return _ready;
}
 
int16_t Radio::lastRssi() const {
    return _rfm9x.lastRssi();
}
 
uint8_t Radio::_computeCRC(const uint8_t *data, size_t len) const {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}
