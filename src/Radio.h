#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <RH_RF95.h>
#include <States.h>

/**
 * AHHHHHHHHHHHHHHHHHHHH
 * PLEASE CHANGE ME IF NEEDED
 */
struct __attribute__((packed)) TelemetryPacket {
    uint32_t time_ms;

    // ADXL345 (low-g)
    float accel_x;
    float accel_y;
    float accel_z;

    // ADXL375 (high-g)
    float accel_x_high_g;
    float accel_y_high_g;
    float accel_z_high_g;

    // Barometer
    float pressure;
    float temperature;
    float altitude;

    // Airbrake state
    float airbrake_pct;
    int8_t airbrake_dir;

    // Flight state
    uint8_t state;       // cast of States enum

    uint8_t crc;
};

class Radio {
public:
    /**
     * @param csPin   SPI chip-select for the RFM95 module
     * @param intPin  Interrupt (DIO0 / G0) pin from the RFM95 module
     */
    Radio(uint8_t csPin, uint8_t intPin);

    /**
     * @brief Initialise the RFM95 module.
     *
     * @param frequencyMHz  default 915.0
     * @param txPowerDbm    Transmit power 5-23 dBm (default 20)
     * @return true on success
     */
    bool begin(float frequencyMHz = 915.0, int8_t txPowerDbm = 20);

    /**
     * @return true if a packet was actually dispatched this call
     */
    bool sendTelemetry(const TelemetryPacket &pkt);

    void setTxInterval(unsigned long ms);

    /**
     * @brief Check whether the driver initialised successfully.
     */
    bool isReady() const;

    /**
     * @brief Last measured RSSI from the driver (useful for ground-side).
     */
    int16_t lastRssi() const;
 
private:
    RH_RF95  _rfm9x;
    bool     _ready;

    unsigned long _txIntervalMs;
    unsigned long _lastTxTime;

    uint8_t _computeCRC(const uint8_t *data, size_t len) const;
};

#endif
