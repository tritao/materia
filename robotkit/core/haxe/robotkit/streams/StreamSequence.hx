package robotkit.streams;
import haxe.Int64;
/** Latest stream identity observed when a control snapshot was captured. */
class StreamSequence {
  public final streamId:String;
  public final kind:String;
  public final sequence:Int64;
  public final sourceClockId:String;
  public function new(streamId:String, kind:String, sequence:Int64, sourceClockId:String) {
    this.streamId = streamId; this.kind = kind; this.sequence = sequence; this.sourceClockId = sourceClockId;
  }
}
