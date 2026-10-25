#include "Bmi088Driver.h"

Bmi088Driver::Bmi088Driver(SPIClass& bus, uint8_t accelCs, uint8_t gyroCs)
    : accel_(bus, accelCs), gyro_(bus, gyroCs),
      accelCs_(accelCs), gyroCs_(gyroCs) {}

bool Bmi088Driver::begin() {
    ready_ = false;
    status_ = Status::NotInitialized;
    if (accelCs_ == gyroCs_) return fail(Status::InvalidPins);

    // this is to make either sensor unselected at the start so that there are no issues with communication
    digitalWrite(accelCs_, HIGH);
    digitalWrite(gyroCs_, HIGH);
    pinMode(accelCs_, OUTPUT);
    pinMode(gyroCs_, OUTPUT);

    delay(31); // accel and gyro power on // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

    int accelStatus = accel_.begin();
    if (accelStatus < 0) accelStatus = accel_.begin(); // trying again since first intiation might be activating accelerometer (or if == -1)
    if (accelStatus < 0) return fail(Status::AccelerometerInitFailed);
    if (gyro_.begin() < 0) return fail(Status::GyroscopeInitFailed);

    if (!accel_.setRange(Bmi088Accel::RANGE_24G) || // 24 Gs
        !gyro_.setRange(Bmi088Gyro::RANGE_2000DPS) || // 2000 degrees per second
        !accel_.setOdr(Bmi088Accel::ODR_100HZ_BW_40HZ) || // 100 Hz updating rate
        !gyro_.setOdr(Bmi088Gyro::ODR_100HZ_BW_32HZ)) { // 100 Hz updating rate
        return fail(Status::ConfigurationFailed);
    }
    delay(50); // Update readings after changing settings
    ready_ = true;
    status_ = Status::Ok;
    return true;
}

bool Bmi088Driver::read(ImuReading& output) {
    if (!ready_) return false;

    output.timeUs = micros();
    accel_.readSensor();
    gyro_.readSensor();

    output.accelX = accel_.getAccelX_mss();
    output.accelY = accel_.getAccelY_mss();
    output.accelZ = accel_.getAccelZ_mss();
    output.gyroX = gyro_.getGyroX_rads();
    output.gyroY = gyro_.getGyroY_rads();
    output.gyroZ = gyro_.getGyroZ_rads();
    return true;
}

bool Bmi088Driver::fail(Status status) {
    ready_ = false;
    status_ = status;
    return false;
}

const char* Bmi088Driver::statusText(Status status) {
    switch (status) {
    case Status::Ok: return "Initialized (reads are not communication-verified) ";
    case Status::NotInitialized: return "Call begin() first";
    case Status::InvalidPins: return "Accelerometer and gyro CS must differ";
    case Status::AccelerometerInitFailed: return "Library accelerometer initialization failed";
    case Status::GyroscopeInitFailed: return "Library gyro initialization failed";
    case Status::ConfigurationFailed: return "Library range/rate configuration failed";
    }
    return "Unknown status";
}
