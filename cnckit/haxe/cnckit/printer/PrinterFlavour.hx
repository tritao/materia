package cnckit.printer;

/** The firmware dialect `PrinterWriter` writes. They differ only in object markers and non-standard heaters and fans. */
enum PrinterFlavour {
  /** Klipper: `EXCLUDE_OBJECT_*` markers, `SET_HEATER_TEMPERATURE` and `SET_FAN_SPEED` for generic parts. */
  Klipper;
  /** Marlin: `M486` object markers; only the hotend, bed, chamber and numbered fans exist. */
  Marlin;
}
