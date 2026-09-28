package motionkit.robot;

enum SessionState {
  Idle;
  Running;
  Holding;
  Held;
  Stopping(then:StopDisposition);
  Faulted;
}
