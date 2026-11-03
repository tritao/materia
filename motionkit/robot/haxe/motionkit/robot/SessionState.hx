package motionkit.robot;

enum SessionState implements Equatable {
  Idle;
  Running;
  Holding;
  Held;
  Stopping(then:StopDisposition);
  Faulted;
}
