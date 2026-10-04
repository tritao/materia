package robotkit.streams;
/** Idempotent owner-thread cancellation of a stream subscription. */
class SensorStreamSubscription {
  var cancelListener:Null<Void->Void>;
  public function new(cancel:Void->Void) cancelListener = cancel;
  public function cancel():Void {
    var action = cancelListener; cancelListener = null;
    if (action != null) action();
  }
}
