struct ControlPacket {
    uint32_t time_ms;       // 4 bytes
    float pressure;         // 4 bytes
    float temperature;      // 4 bytes
    float accel_z_low_g;    // 4 bytes (ADXL345 for coast phase)
    float accel_z_high_g;   // 4 bytes (ADXL375 for boost phase)
    uint8_t flight_state;   // 1 byte  (STM32 tells Teensy when motor burnout happens)
} __attribute__((packed));