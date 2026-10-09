package cnckit;

/** Ordering and interpretation of commands within one G-code block. */
enum CncDialect {
  LinuxCnc;
  /** The Marlin, Klipper and RepRap family: extruder words, heater and fan M-codes, comments as markers. */
  Printer;
}
