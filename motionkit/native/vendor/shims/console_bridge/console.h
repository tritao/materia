#pragma once

#include <cstdarg>
#include <cstdio>
#include <string>

namespace console_bridge {
enum LogLevel {
  CONSOLE_BRIDGE_LOG_DEBUG = 0,
  CONSOLE_BRIDGE_LOG_INFO = 1,
  CONSOLE_BRIDGE_LOG_WARN = 2,
  CONSOLE_BRIDGE_LOG_ERROR = 3
};

inline thread_local std::string *diagnostic_sink = nullptr;
inline LogLevel getLogLevel() { return CONSOLE_BRIDGE_LOG_WARN; }

inline void log(LogLevel level, const char *format, ...) {
  if (!diagnostic_sink || level < getLogLevel()) return;
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (!diagnostic_sink->empty()) diagnostic_sink->push_back('\n');
  diagnostic_sink->append(buffer);
}

class ScopedDiagnosticSink {
 public:
  explicit ScopedDiagnosticSink(std::string &sink) : previous_(diagnostic_sink) {
    diagnostic_sink = &sink;
  }
  ~ScopedDiagnosticSink() { diagnostic_sink = previous_; }
 private:
  std::string *previous_;
};
}  // namespace console_bridge

#define CONSOLE_BRIDGE_logDebug(...) ::console_bridge::log(::console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, __VA_ARGS__)
#define CONSOLE_BRIDGE_logInform(...) ::console_bridge::log(::console_bridge::CONSOLE_BRIDGE_LOG_INFO, __VA_ARGS__)
#define CONSOLE_BRIDGE_logWarn(...) ::console_bridge::log(::console_bridge::CONSOLE_BRIDGE_LOG_WARN, __VA_ARGS__)
#define CONSOLE_BRIDGE_logError(...) ::console_bridge::log(::console_bridge::CONSOLE_BRIDGE_LOG_ERROR, __VA_ARGS__)
