package tests;

import haxe.Int64;
import haxeon.wire.MessagePack;
import robotkit.behavior.WorldBehavior;
import robotkit.behavior.WorldBehaviorContext;
import robotkit.behavior.WorldBehaviorRunner;
import robotkit.perception.ImageDetection;
import robotkit.perception.ImageDetectionObservation;
import robotkit.protocol.ImageDetectionObservationMsg;
import robotkit.protocol.RobotMessageType;
import robotkit.protocol.RobotProtocol;
import robotkit.world.RecordingChannels;
import robotkit.world.RecordingRobot;
import robotkit.world.McapRobotRecording;
import robotkit.world.McapRecordingReader;
import robotkit.world.RemoteRobot;
import robotkit.world.ReplayRobot;
import robotkit.world.RobotEvent;
import robotkit.world.RobotEventRing;
import robotkit.world.RobotRecording;

private class EventBehavior implements WorldBehavior {
  public var seen:Int = 0;
  public function new() {}
  public function update(context:WorldBehaviorContext):Void seen += context.events.length;
}

private class LocalEventBehavior implements robotkit.behavior.RobotBehavior {
  public var observations:Int = 0;
  public function new() {}
  public function update(context:robotkit.behavior.RobotContext):Void
    for (event in context.events) switch event {
      case Observation(_, _): observations++;
      case Overflow(_, _):
    }
}

class RobotEventTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function observation(sequence:Int):ImageDetectionObservation
    return new ImageDetectionObservation("robotd/front", "front", "camera", "model",
      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
      "frame/camera", Int64.ofInt(sequence), Int64.ofInt(10), Int64.ofInt(20),
      "camera.clock", "host.clock", Int64.ofInt(30), "host.clock",
      [new ImageDetection("box", 0.9, 1, 2, 3, 4)], 0);

  public static function run():Int {
    var ring = new RobotEventRing(2);
    for (index in 1...4) ring.publish(observation(index));
    var first = ring.events(Int64.ofInt(0), 1);
    check(first.length == 1 && switch first[0] { case Overflow(ordinal, count):
      Int64.compare(ordinal, Int64.ofInt(1)) == 0 && count == 1;
      case _: false; }, "slow consumer receives one overflow marker");
    var retained = ring.events(RobotEventRing.ordinalOf(first[0]), 4);
    check(retained.length == 2 && Int64.compare(RobotEventRing.ordinalOf(retained[1]),
      Int64.ofInt(3)) == 0, "retained event order is monotonic");

    var message = ImageDetectionObservationMsg.fromObservation(Int64.ofInt(42),
      Int64.ofInt(9), observation(7));
    var frame = RobotProtocol.imageDetectionObservation(message);
    check(frame.messageType == RobotMessageType.ImageDetectionObservation,
      "detection uses message type 19");
    var decoded = RobotProtocol.decodeImageDetectionObservation(frame);
    check(decoded.ordinal == message.ordinal &&
      decoded.toObservation().detections[0].width == 3,
      "wire round trip keeps ordinal and pixel box");

    var remote = new RemoteRobot("robot-events");
    var wireRemote = new RemoteRobot("wire-events");
    wireRemote.onImageDetection(ImageDetectionObservationMsg.fromObservation(Int64.ofInt(42),
      Int64.ofInt(1), observation(1)));
    wireRemote.onImageDetection(ImageDetectionObservationMsg.fromObservation(Int64.ofInt(42),
      Int64.ofInt(3), observation(3)));
    var wireGap = wireRemote.events(Int64.ofInt(1), 10);
    check(wireGap.length == 2 && switch wireGap[0] {
      case Overflow(ordinal, count): Int64.compare(ordinal, Int64.ofInt(2)) == 0 && count == 1;
      case _: false;
    } && switch wireGap[1] {
      case Observation(ordinal, _): Int64.compare(ordinal, Int64.ofInt(3)) == 0;
      case _: false;
    }, "wire observation gaps become overflow events");
    wireRemote.close();
    var memory = new RobotRecording();
    memory.recordSnapshot(remote.snapshot());
    var recording = new RecordingRobot(remote, memory);
    remote.publishObservation(observation(7));
    var behavior = new EventBehavior();
    var runner = new WorldBehaviorRunner(behavior);
    runner.update(recording);
    runner.update(recording);
    check(behavior.seen == 1, "behavior receives event once even without a new snapshot");
    check(memory.entries.length >= 3, "recording captures observation alongside snapshot");
    var channel = new RecordingChannels().get("perception.image_detections");
    check(channel != null, "perception recording channel is registered");
    if (channel != null) {
      var encoded = channel.encode(memory.entries[1]);
      var decodedEntry = channel.decode(encoded);
      check(decodedEntry.robotId == "robot-events" && switch decodedEntry.event {
        case Channel(_, _, payload):
          var event:RobotEvent = cast payload;
          Int64.compare(RobotEventRing.ordinalOf(event), Int64.ofInt(1)) == 0;
        case _: false;
      }, "MCAP channel keeps logical robot and event ordinal");
    }
    var path = '/tmp/robotkit-perception-${Sys.getPid()}.mcap';
    var writer = new McapRobotRecording(path);
    writer.recordSnapshot(remote.snapshot());
    writer.recordRobotEvent(remote.id(), RobotEvent.Observation(Int64.ofInt(1), observation(7)));
    writer.close();
    var persisted = McapRecordingReader.load(path);
    check(persisted.entries.length == 2 && switch persisted.entries[1].event {
      case Channel(id, name, payload):
        id == "robot-events" && name == "perception.image_detections" &&
          Int64.compare(RobotEventRing.ordinalOf(cast payload), Int64.ofInt(1)) == 0;
      case _: false;
    }, "MCAP persists the typed detection channel and ordinal");
    sys.FileSystem.deleteFile(path);
    var replay = new ReplayRobot("robot-events", memory);
    check(replay.events(Int64.ofInt(0), 10).length == 0, "replay waits for recorded event");
    check(replay.advance(), "replay advances to event");
    var replayed = replay.events(Int64.ofInt(0), 10);
    check(replayed.length == 1 && Int64.compare(RobotEventRing.ordinalOf(replayed[0]),
      Int64.ofInt(1)) == 0, "replay restores original event ordinal");
    replay.close(); recording.close();
    var gapRecording = new RobotRecording();
    gapRecording.recordRobotEvent("robot-events", RobotEvent.Observation(Int64.ofInt(1), observation(1)));
    gapRecording.recordRobotEvent("robot-events", RobotEvent.Observation(Int64.ofInt(3), observation(3)));
    var gapReplay = new ReplayRobot("robot-events", gapRecording);
    check(gapReplay.advance(), "replay advances over the ordinal gap");
    var gapEvents = gapReplay.events(Int64.ofInt(1), 10);
    check(gapEvents.length == 2 && switch gapEvents[0] {
      case Overflow(ordinal, count): Int64.compare(ordinal, Int64.ofInt(2)) == 0 && count == 1;
      case _: false;
    } && switch gapEvents[1] {
      case Observation(ordinal, _): Int64.compare(ordinal, Int64.ofInt(3)) == 0;
      case _: false;
    }, "replay emits the same overflow gap as the live ring");
    gapReplay.close();
    var localBehavior = new LocalEventBehavior();
    var localRunner = new robotkit.behavior.RobotBehaviorRunner(localBehavior);
    var runtimeSnapshot = new robotkit.runtime.RobotSnapshot(Int64.ofInt(1),
      Int64.ofInt(1), Int64.ofInt(10), 1, 0, 0, 0, [], [], [], Int64.ofInt(20));
    localRunner.update(runtimeSnapshot, Int64.ofInt(20));
    localRunner.offerEvent(RobotEvent.Observation(Int64.ofInt(1), observation(1)));
    localRunner.update(runtimeSnapshot, Int64.ofInt(21));
    check(localBehavior.observations == 1,
      "local robotd behavior receives observation without a new snapshot");
    Sys.println('RobotKit robot event tests passed ($assertions assertions)');
    return assertions;
  }
}
