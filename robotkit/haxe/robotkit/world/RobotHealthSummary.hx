package robotkit.world;

/** Aggregate health counts; it contains no adapter references. */
class RobotHealthSummary {
  public final total:Int;
  public final ready:Int;
  public final connecting:Int;
  public final disconnected:Int;
  public final faulted:Int;

  public function new(total:Int, ready:Int, connecting:Int, disconnected:Int, faulted:Int) {
    this.total = total;
    this.ready = ready;
    this.connecting = connecting;
    this.disconnected = disconnected;
    this.faulted = faulted;
  }
}
