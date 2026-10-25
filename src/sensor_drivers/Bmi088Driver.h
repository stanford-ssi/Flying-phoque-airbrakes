#ifndef Bmi088Driver_h
#define Bmi088Driver_h

#include <BMI088.h> // Bolder Flight Systems library

struct ImuReading {
    float accelX, accelY, accelZ; // m/s^2 // need to remember to check axis
    float gyroX, gyroY, gyroZ; // rad/s
    uint32_t timeUs; // time measured BEFORE reads
};

class Bmi088Driver {
public:
    enum class Status {Ok, NotInitialized, InvalidPins, AccelerometerInitFailed, GyroscopeInitFailed, ConfigurationFailed};

    Bmi088Driver(SPIClass& bus, uint8_t accelCs, uint8_t gyroCs);
    Bmi088Driver(const Bmi088Driver&) = delete;
    Bmi088Driver& operator=(const Bmi088Driver&) = delete;

    bool begin();
    bool read(ImuReading& output);
    Status lastStatus() const { return status_; } // this line and below is for getting status of Bmi
    static const char* statusText(Status status);

private:
    Bmi088Accel accel_;
    Bmi088Gyro gyro_;
    const uint8_t accelCs_, gyroCs_;
    bool ready_ = false;
    Status status_ = Status::NotInitialized;
    bool fail(Status status); // this plus the two lines above are for simply declaring that something didn't work
};

#endif