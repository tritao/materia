package robotkit.streams;
import haxe.Int64;
import robotkit.core.SensorFrame;

/** Owner-thread timestamped streams with one retained sample per stream. */
class SensorStreams {
  final latest:Map<String, SensorStreamSample> = [];
  final listeners:Array<StreamListener> = [];
  public function new() {}
  public function subscribe(streamId:String, listener:SensorStreamSample->Void,
      ?replayLatest:Bool = true):SensorStreamSubscription {
    if (streamId == null || streamId.length == 0 || listener == null)
      throw "Stream subscription requires an ID (or *) and listener";
    var value = new StreamListener(streamId, listener); listeners.push(value);
    if (replayLatest) for (sample in latest) if (streamId == "*" || streamId == sample.streamId) listener(sample);
    return new SensorStreamSubscription(function() listeners.remove(value));
  }
  public function publish(sample:SensorStreamSample):Void {
    if (sample == null) throw "Stream sample is required";
    var previous = latest.get(sample.streamId);
    if (previous != null && previous.sourceClockId == sample.sourceClockId &&
        Int64.compare(sample.sequence, previous.sequence) <= 0 &&
        // Native simulated sources currently have no epoch identifier. A reset
        // may repeat their sequence, but a freshly accepted receipt distinguishes
        // that measurement from the retained sample.
        !(sample.sourceClockId == "unspecified" &&
          Int64.compare(sample.receivedTimestampNs, previous.receivedTimestampNs) > 0)) return;
    latest.set(sample.streamId, sample);
    for (value in listeners.copy())
      if (listeners.contains(value) && (value.id == "*" || value.id == sample.streamId)) value.receive(sample);
  }
  public function publishFrames(frames:Array<SensorFrame>):Void
    for (frame in frames) publish(SensorStreamSample.sensor(frame));
  public function latestFrames():Array<SensorFrame> {
    var result:Array<SensorFrame> = [];
    for (sample in latest) if (sample.frame != null) result.push(sample.frame.copy());
    result.sort(function(a,b) return Reflect.compare(a.sensorId,b.sensorId));
    return result;
  }
  public function sequences():Array<StreamSequence> {
    var result = [for (sample in latest) new StreamSequence(sample.streamId, sample.kind, sample.sequence, sample.sourceClockId)];
    result.sort(function(a,b) return Reflect.compare(a.streamId,b.streamId));
    return result;
  }
  public function clear():Void { latest.clear(); listeners.resize(0); }
}
private class StreamListener {
  public final id:String;
  public final receive:SensorStreamSample->Void;
  public function new(id:String, receive:SensorStreamSample->Void) { this.id = id; this.receive = receive; }
}
