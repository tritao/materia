package processkit;

enum ProcessRunState {
  Preparation;
  Ready;
  Active;
  ControlledInterruption;
  Recovery;
  Completion;
  /** The process could not begin: the device did not become ready in time. */
  Failed;
}
