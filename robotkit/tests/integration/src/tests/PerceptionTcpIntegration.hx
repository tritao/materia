package tests;

import haxe.Int64;
import NativeKitRuntime;
import robotkit.behavior.WorldBehavior;
import robotkit.behavior.WorldBehaviorContext;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.recording.RecordingRobot;
import robotkit.recording.ReplayRobot;
import robotkit.core.RobotEvent;
import robotkit.core.RobotEventRing;
import robotkit.recording.RobotRecording;
import robotkit.worldd.WorldHost;

private class DetectionBehavior implements WorldBehavior {
  public var count:Int = 0;
  public function new() {}
  public function update(context:WorldBehaviorContext):Void
    for (event in context.events) switch event {
      case Observation(_, value): if (value.detections.length > 0) count++;
      case Overflow(_, _): // A late subscriber sees the gap before retained observations.
    }
}

class PerceptionTcpIntegration {
  public static function run(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var worldHost = new WorldHost();
    var remote = worldHost.addRemoteRobot("tcp-perception");
    remote.enableCamera();
    remote.enableObservations();
    var failure:Dynamic = null;
    try {
      remote.connect(host, port, runtime.events);
      var recording = new RobotRecording();
      var recorded = new RecordingRobot(remote, recording);
      var behavior = new DetectionBehavior();
      var runner = new WorldBehaviorRunner(behavior);
      var event:Null<RobotEvent> = null;
      for (tick in 0...500) {
        while (runtime.events.poll()) {}
        worldHost.step(Int64.fromFloat((tick + 1) * 10000000.0));
        runner.update(recorded);
        var available = remote.events(Int64.ofInt(0), 8);
        for (candidate in available) switch candidate {
          case Observation(_, _) if (event == null): event = candidate;
          case Observation(_, _):
          case Overflow(_, _):
        }
        if (event != null && behavior.count > 0) break;
        runtime.events.wait(0.01);
      }
      if (event == null) throw "robotd detection did not reach RemoteRobot over TCP";
      var observation = switch event { case Observation(_, value): value; case _: throw "unexpected event"; };
      if (observation.detections.length != 1 || observation.sourceFrameId.length == 0)
        throw "TCP detection lost image boxes or provenance";
      if (behavior.count < 1) throw "World behavior did not receive TCP observation";
      var replay = new ReplayRobot(remote.id(), recording);
      while (replay.advance()) {}
      var liveEvents = remote.events(Int64.ofInt(0), 256);
      var replayed = replay.events(Int64.ofInt(0), 256);
      if (liveEvents.length != replayed.length) throw "Replay changed event count";
      for (index in 0...liveEvents.length) {
        if (Int64.compare(RobotEventRing.ordinalOf(liveEvents[index]),
            RobotEventRing.ordinalOf(replayed[index])) != 0)
          throw "Replay changed event order";
        switch [liveEvents[index], replayed[index]] {
          case [Observation(_, live), Observation(_, copy)]:
            if (live.producerId != copy.producerId || live.sequence != copy.sequence ||
                live.sourceFrameId != copy.sourceFrameId ||
                live.detections.length != copy.detections.length)
              throw "Replay changed observation payload";
          case [Overflow(_, liveCount), Overflow(_, copyCount)]:
            if (liveCount != copyCount) throw "Replay changed overflow gap";
          case _: throw "Replay changed event kind";
        }
      }
      replay.close();
      Sys.println("robotd perception reached RemoteRobot, behavior, recording and replay");
    } catch (error:Dynamic) failure = error;
    worldHost.close(); runtime.dispose();
    if (failure != null) throw failure;
  }
}
