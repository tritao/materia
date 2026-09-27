package processkit;

enum ProcessRunState {
  Preparation;
  Ready;
  Active;
  ControlledInterruption;
  Recovery;
  Completion;
}
