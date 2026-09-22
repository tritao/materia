package robotkit.world;

/** Disposable observation subscription owned by one RobotWorld. */
class RobotWorldSubscription {
  final cancel:Void->Void;
  var active:Bool = true;

  public function new(cancel:Void->Void) this.cancel = cancel;

  public function dispose():Void {
    if (!active) return;
    active = false;
    cancel();
  }
}
