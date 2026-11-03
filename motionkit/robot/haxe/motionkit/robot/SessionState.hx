package motionkit.robot;

enum SessionState : Equatable {
  Idle;
  Running;
  Holding;
  Held;
  Stopping(then:StopDisposition);
  Faulted;
}
