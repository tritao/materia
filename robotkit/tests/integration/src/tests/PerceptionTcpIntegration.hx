package tests;

import haxe.Int64;
import NativeKitRuntime;
import robotkit.behavior.WorldBehavior;
import robotkit.behavior.WorldBehaviorContext;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.world.RecordingRobot;
import robotkit.world.ReplayRobot;
import robotkit.world.RobotEvent;
import robotkit.world.RobotEventRing;
import robotkit.world.RobotRecording;
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
    var failure:Dynamic = null;
    try {
      remote.connect(host, port, runtime.events);
      var event:Null<RobotEvent> = null;
      for (_ in 0...500) {
        while (runtime.events.poll()) {}
        var available = remote.events(Int64.ofInt(0), 8);
        for (candidate in available) switch candidate {
          case Observation(_, _) if (event == null): event = candidate;
          case Observation(_, _):
          case Overflow(_, _):
        }
        if (event != null) break;
        runtime.events.wait(0.01);
      }
      if (event == null) throw "robotd detection did not reach RemoteRobot over TCP";
      var observation = switch event { case Observation(_, value): value; case _: throw "unexpected event"; };
      if (observation.detections.length != 1 || observation.sourceFrameId.length == 0)
        throw "TCP detection lost image boxes or provenance";
      var recording = new RobotRecording();
      recording.recordSnapshot(remote.snapshot());
      var recorded = new RecordingRobot(remote, recording);
      var behavior = new DetectionBehavior();
      new WorldBehaviorRunner(behavior).update(recorded);
      if (behavior.count < 1) throw "World behavior did not receive TCP observation";
      var replay = new ReplayRobot(remote.id(), recording);
      if (!replay.advance()) throw "Replay did not advance to detection";
      var replayed = replay.events(Int64.ofInt(0), 8);
      var replayObservation:Null<RobotEvent> = null;
      for (candidate in replayed) switch candidate {
        case Observation(_, _): replayObservation = candidate;
        case Overflow(_, _):
      }
      if (replayObservation == null || Int64.compare(RobotEventRing.ordinalOf(replayObservation),
          RobotEventRing.ordinalOf(event)) != 0)
        throw "Replay changed event order";
      replay.close();
      Sys.println("robotd perception reached RemoteRobot, behavior, recording and replay");
    } catch (error:Dynamic) failure = error;
    worldHost.close(); runtime.dispose();
    if (failure != null) throw failure;
  }
}
