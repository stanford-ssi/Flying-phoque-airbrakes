#include "Logging.h"

#include <RTClib.h>
#include <SD.h>
#include <States.h>

RTC_DS3231 rtc;
bool rtc_ready = false;

void dateTime(uint16_t *date, uint16_t *time) {
  DateTime now;

  if (rtc_ready) {
    now = rtc.now();
  } else {
    // fallback to compile time
    now = DateTime(F(__DATE__), F(__TIME__));
  }

  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

Logging::Logging(bool debug, bool logToSD, int SD_CS) {
  this->debug = debug;
  this->logToSD = logToSD;
  this->SD_CS = SD_CS;
}

int Logging::getNextLogFileNumber() {
  int number = 1;
  char filename[20];

  // Increment until we find a number that does NOT exist
  while (true) {
    sprintf(filename, "LOG%03d.TXT", number);
    if (!SD.exists(filename)) {
      break;
    }
    number++;
  }
  return number;
}

bool Logging::begin() {
  if (debug) {
    delay(100);
    Serial1.println(F("Logger starting..."));
  }

  if (!rtc.begin()) {
    if (debug) Serial1.println(F("RTC not found, using compile time"));
    rtc_ready = false;
  } else {
    rtc_ready = true;
  }

  if (logToSD) {
    if (!SD.begin(SD_CS)) {
      Serial1.println(F("SD init failed"));
      return false;
    } else {
      Serial1.println("SD init succeeded");
    }

    // callback so SD sets file timestamps
    SdFile::dateTimeCallback(dateTime);

    int logNumber = getNextLogFileNumber();
    char logFileName[20];
    sprintf(logFileName, "LOG%03d.TXT", logNumber);
    Serial1.print("Opening log file: ");
    Serial1.println(logFileName);
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (!dataFile) {
      Serial1.print("Failed to open file: ");
      Serial1.println(logFileName);
      return false;
    }

    Serial1.print("Logging to ");
    Serial1.println(logFileName);
  }

  return true;
}

void Logging::log(const char *message, bool newline) {
  if (debug) {
    if (newline)
      Serial1.println(message);
    else
      Serial1.print(message);
  }
  if (logToSD) {
    if (newline)
      dataFile.println(message);
    else
      dataFile.print(message);
  }
}

void Logging::flush() {
  if (logToSD) {
    dataFile.flush();
  }
}

void Logging::logTelemetry(float altitude, const SensorData_t &sens, const BrakeState_t &brake, States st) {
  LogBuffer buf;
  buf.appendLong(millis());
  buf.field(sens.accel_x);
  buf.field(sens.accel_y);
  buf.field(sens.accel_z);
  buf.field(sens.accel_x_high_g);
  buf.field(sens.accel_y_high_g);
  buf.field(sens.accel_z_high_g);
  buf.field(sens.pressure);
  buf.field(sens.temperature);
  buf.field(altitude);
  buf.field(sens.bno_x);
  buf.field(sens.bno_y);
  buf.field(sens.bno_z);
  buf.field(sens.bno_i, 4);
  buf.field(sens.bno_j, 4);
  buf.field(sens.bno_k, 4);
  buf.field(sens.bno_real, 4);
  buf.field(stateToString(st));
  buf.field(brake.pct, 1);
  buf.field(brake.direction);
  buf.field((int)sens.potentiometer_value);
  log(buf.str());

  if (millis() - lastFlush > 1000) {
    flush();
    lastFlush = millis();
  }
}
