package machinekit.operator;

/** What an F1 to F3 key does. The pendant shows the label; the host owns the behaviour. */
enum KeyAction {
  Unused;
  DoorToggle;
  SpindleJog;
  WorkLight;
  Mist;
  Vacuum;
}
