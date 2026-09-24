package robotkit.safety;

/** User-visible operating condition reported by safety policy. */
enum SafetyPhase {
  Normal;
  Restricted;
  ProtectiveStop;
  EmergencyStop;
}
