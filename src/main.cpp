#define I2C_BUFFER_LENGTH 128
#define DEBUG_MODE 0  // Set to 0 for flight, 1 for bench testing

#include <Arduino.h>
#include <Globals.h>
#include <Logging.h>
#include <PhysicsConstants.h>
#include <PinDefinitions.h>
#include <SD.h>
#include <SPI.h>
#include <StateMachine.h>
#include <Wire.h>
#include <control/ControlStruct.h>
#include <sensor_drivers/Adxl.h>
#include <sensor_drivers/BNO.h>
#include <sensor_drivers/Lps22.h>

#define SIMULATION_MODE 0
#define USE_BNO080 1
#define USE_TELEMETRY 0

// Sensors
Adxl adxl345 = Adxl(0x1D, ADXL345);
Adxl adxl375 = Adxl(0x53, ADXL375);
Lps22 lps22 = Lps22(0x5C);
BNO bno080;

// Actuators
Bilda airbrake_servo_1;
Bilda airbrake_servo_2;
Igniter primaryIgniter = Igniter(PinDefs.IGNITER_0, PinDefs.IGNITER_SENSE_0);
Igniter backupIgniter = Igniter(PinDefs.IGNITER_1, PinDefs.IGNITER_SENSE_1);

// Peripherals
Logging logging(DEBUG_MODE, true, PinDefs.SD_CS);
StatusIndicator statusIndicator =
    StatusIndicator(PinDefs.STATUS_LED_RED, PinDefs.STATUS_LED_GREEN, PinDefs.STATUS_LED_BLUE);

// Shared state
SensorData_t sensors;
BrakeState_t BrakeState;
FlightState_t FlightState;
I2CControl_t I2CControl;
States state = States::BOOT;
int failed_sensors = 0;

// Timers
int last_sd_write = 0;
unsigned long last_loop_time = 0;

static struct {
  float pressure = 0;
  float temperature = 0;
} RefCalibration;

static void calibrateSensors(Lps22 &lps) {
  for (int i = 0; i < 10; i++) {
    float p, t;
    lps.readPressure(&p);
    lps.readTemperature(&t);
    RefCalibration.pressure += p;
    RefCalibration.temperature += t;
  }
  RefCalibration.pressure /= 10.0f;
  RefCalibration.temperature = (RefCalibration.temperature / 10.0f) + CELSIUS_TO_KELVIN;
}

template <typename T>  // generic sensor
static bool initSensor(T &sensor, const char *name, int maxAttempts = 10) {
  int attempts = 0;
  while (!sensor.begin()) {
    logging.log(name);
    Serial1.print(F("Waiting for "));
    Serial1.print(name);
    Serial1.println(F("..."));
    delay(1000);
    if (++attempts >= maxAttempts) {
      failed_sensors++;
      return false;
    }
  }
  Serial1.print(name);
  Serial1.println(F(" initialized"));
  return true;
}

static float altitudeDelta(float p, float T) {
#if SIMULATION_MODE
  float p_ref = SIM_PRESSURE_REF;
  float T_ref = SIM_TEMPERATURE_REF;
#else
  float p_ref = RefCalibration.pressure;
  float T_ref = RefCalibration.temperature;
#endif
  float Tbar = 0.5f * (T_ref + T);
  return (Rd * Tbar / g0) * log(p_ref / p);
}

static float readSensors() {
#if SIMULATION_MODE
  Serial1.println("DATAREQUEST");
  String line = Serial1.readStringUntil('\n');
  line.trim();

  int lastIndex = 0;
  int index = 0;
  float values[6];

  for (int i = 0; i < (int)line.length(); i++) {
    if (line[i] == ',' || i == (int)line.length() - 1) {
      String part = line.substring(lastIndex, (i == (int)line.length() - 1) ? i + 1 : i);
      values[index++] = part.toFloat();
      lastIndex = i + 1;
    }
  }

  if (index >= 6) {
    sensors.accel_x = values[1];
    sensors.accel_y = values[2];
    sensors.accel_z = values[3];
    sensors.pressure = values[4];
    sensors.temperature = values[5];
    sensors.accel_x_high_g = sensors.accel_x;
    sensors.accel_y_high_g = sensors.accel_y;
    sensors.accel_z_high_g = sensors.accel_z;
  }
#else
  adxl345.readAccelerometer(&sensors.accel_x, &sensors.accel_y, &sensors.accel_z);
  adxl375.readAccelerometer(&sensors.accel_x_high_g, &sensors.accel_y_high_g, &sensors.accel_z_high_g);
  lps22.readPressure(&sensors.pressure);
  lps22.readTemperature(&sensors.temperature);
#endif

  float altitude = altitudeDelta(sensors.pressure, sensors.temperature + CELSIUS_TO_KELVIN);

#if USE_BNO080
  if (bno080.dataAvailable()) {
    bno080.getLinearAccelerometer(&sensors.bno_x, &sensors.bno_y, &sensors.bno_z);
    bno080.getRotationVector(&sensors.bno_i, &sensors.bno_j, &sensors.bno_k, &sensors.bno_real);
  }
#endif

  return altitude;
}

static void sendControlPacket(float altitude) {
  if (!USE_CONTROL || I2CControl.fallback || (millis() - I2CControl.lastSend < CONTROL_INTERVAL_MS)) return;

  ControlPacket controlPacket;
  controlPacket.time_ms = millis();
  controlPacket.pressure = sensors.pressure;
  controlPacket.temperature = sensors.temperature;
  controlPacket.accel_z_low_g = sensors.accel_y;  // Y-axis is thrust axis
  controlPacket.accel_z_high_g = sensors.accel_y_high_g;
  controlPacket.baro_altitude = altitude;
  controlPacket.flight_state = static_cast<uint8_t>(state);
  controlPacket.crc = computeControlCRC((uint8_t *)&controlPacket, sizeof(controlPacket) - 1);

  Wire.beginTransmission(CTRL_TEENSY_ADDR);
  Wire.write((uint8_t *)&controlPacket, sizeof(controlPacket));
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    I2CControl.failCount++;
    if (I2CControl.failCount >= I2C_FAIL_THRESHOLD) {
      I2CControl.fallback = true;
      Serial1.println(F("Control Teensy I2C failed, switching to fallback"));
    }
  } else {
    I2CControl.failCount = 0;

    uint8_t bytesRead = Wire.requestFrom(CTRL_TEENSY_ADDR, (uint8_t)sizeof(CommandPacket));
    if (bytesRead == sizeof(CommandPacket)) {
      CommandPacket cmdPkt;
      uint8_t *cmdBuf = (uint8_t *)&cmdPkt;
      for (size_t i = 0; i < sizeof(CommandPacket); i++) {
        cmdBuf[i] = Wire.read();
      }

      uint8_t cmdCrc = computeControlCRC(cmdBuf, sizeof(CommandPacket) - 1);
      if (cmdPkt.crc == cmdCrc) {
        sensors.potentiometer_value = cmdPkt.potentiometer_value;
        if (state == States::ASCENT) {
          airbrake_servo_1.setExtension(cmdPkt.servo_angle_1);
          airbrake_servo_2.setExtension(cmdPkt.servo_angle_2);
          BrakeState.pct = cmdPkt.servo_angle_1;
        }
      }
    } else {
      // Drain any partial data
      while (Wire.available()) Wire.read();
    }
  }

  I2CControl.lastSend = millis();
}

static void logTelemetry(float altitude) {
  LogBuffer buf;

  buf.append("%lu", millis());
  buf.append(",%.2f,%.2f,%.2f", sensors.accel_x, sensors.accel_y, sensors.accel_z);
  buf.append(",%.2f,%.2f,%.2f", sensors.accel_x_high_g, sensors.accel_y_high_g, sensors.accel_z_high_g);
  buf.append(",%.2f,%.2f,%.2f", sensors.pressure, sensors.temperature, altitude);
  buf.append(",%.2f,%.2f,%.2f", sensors.bno_x, sensors.bno_y, sensors.bno_z);
  buf.append(",%.4f,%.4f,%.4f,%.4f", sensors.bno_i, sensors.bno_j, sensors.bno_k, sensors.bno_real);
  buf.append(",%s,%.1f,%d,%u", stateToString(state), BrakeState.pct, BrakeState.direction, sensors.potentiometer_value);

  logging.log(buf.str());

  if (millis() - last_sd_write > 1000) {
    logging.flush();
    last_sd_write = millis();
  }
}

void setup() {
  Serial1.begin(115200);

  pinMode(PinDefs.ARM, INPUT_PULLUP);
  pinMode(PinDefs.IGNITER_0, OUTPUT);
  pinMode(PinDefs.IGNITER_1, OUTPUT);

  airbrake_servo_1.begin(PinDefs.SERVO);
  airbrake_servo_2.begin(PinDefs.SERVO_2);

  while (!Serial1) {
    statusIndicator.solid(StatusIndicator::RED);
  }
  statusIndicator.solid(StatusIndicator::RED);

  Wire.setSDA(PinDefs.SDA);
  Wire.setSCL(PinDefs.SCL);
  Wire.begin();
  Wire.setClock(100000);

  delay(4000);  // allow sensors to power up

  SPI.setMOSI(PinDefs.SDI);
  SPI.setMISO(PinDefs.SDO);
  SPI.setSCLK(PinDefs.SCK);
  SPI.begin();

  while (!logging.begin()) {
    statusIndicator.solid(StatusIndicator::RED);
    Serial1.println(F("Waiting for SD card..."));
    delay(1000);
  }
  Serial1.println(F("SD card initialized"));

  initSensor(adxl345, "ADXL345");
  initSensor(adxl375, "ADXL375");
  initSensor(lps22, "LPS22");
#if USE_BNO080
  initSensor(bno080, "BNO080");
#endif

  calibrateSensors(lps22);

  logging.log(
      "Time,Xg,Yg,Zg,Xhg,Yhg,Zhg,Pressure,Temperature,Altitude,"
      "BNO_X,BNO_Y,BNO_Z,BNO_I,BNO_J,BNO_K,BNO_Real,State,"
      "Airbrake_pct,Airbrake_dir,Potentiometer");
  logging.flush();

  if (failed_sensors > 0) {
    statusIndicator.solid(StatusIndicator::WHITE);
    Serial1.println(F("Setup failed"));
    state = States::SENSOR_ERROR;
  } else {
    statusIndicator.solid(StatusIndicator::GREEN);
    Serial1.println(F("Setup complete"));
    state = States::IDLE;
  }
}

void loop() {
  unsigned long now = millis();
  if (now - last_loop_time < LOOP_INTERVAL_MS) return;
  last_loop_time = now;

  float altitude = readSensors();
  updateStateMachine(altitude);
  sendControlPacket(altitude);
  logTelemetry(altitude);
}