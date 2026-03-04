#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SD.h>

struct LogBuffer {
  char data[256];
  int pos = 0;

  template <typename... Args>
  void append(const char *fmt, Args... args) {
    if (pos < (int)sizeof(data)) {
      pos += snprintf(data + pos, sizeof(data) - pos, fmt, args...);
    }
  }

  const char *str() { return data; }
};

class Logging {
 public:
  Logging(bool debug, bool logToSD, int SD_CS);

  void log(const char* message, bool newline = true);
  bool begin();
  void flush();

 private:
  bool debug;
  bool logToSD;
  int SD_CS;
  int getNextLogFileNumber();
  int logNumber;
  String logFileName;
  File dataFile;
};

#endif
