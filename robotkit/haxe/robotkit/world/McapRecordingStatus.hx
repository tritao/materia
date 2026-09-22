package robotkit.world;

import haxe.Int64;

class McapRecordingStatus {
  public final state:Int;
  public final accepted:Int64;
  public final written:Int64;
  public final dropped:Int64;
  public final queued:Int64;
  public final error:String;
  public function new(state:Int, accepted:Int64, written:Int64, dropped:Int64, queued:Int64, error:String) {
    this.state=state;this.accepted=accepted;this.written=written;this.dropped=dropped;this.queued=queued;this.error=error;
  }
}
