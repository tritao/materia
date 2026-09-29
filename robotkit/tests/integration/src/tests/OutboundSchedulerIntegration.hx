package tests;

import haxe.Int64;
import haxe.io.Bytes;
import nativekit.ffi.NativeKitTypes;
import NativeKitEventBytes;
import NativeKitEventValue;
import NativeKitRuntime;
import robotd.OutboundScheduler.OutboundScheduler;
import robotd.OutboundFamily;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotFrame.RobotFrameStream;
import robotkit.protocol.RobotMessageType;
import robotkit.protocol.Hello;
import robotkit.protocol.RobotProtocol;
import robotkit.protocol.StreamSubscription;
import haxeon.wire.MessagePackWriter;
import robotkit.transport.NativeTransport;

/** Exercises real NativeKit queue/transport behavior with a bounded dispatch step. */
class OutboundSchedulerIntegration {
  public static function run(port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var listener = NativeTransport.listen(port + 1);
    var accepted:Null<TransportHandle> = null;
    var subscription = runtime.events.listen(function(value:NativeKitEventValue) {
      switch value {
        case Raw(kind, source, _, _, _, _, data):
          if (kind == EventKind.TransportAccepted &&
              source.rawValue() == listener.borrow().rawValue())
            accepted = new TransportHandle(NativeKitEventBytes.readU32(data, 4));
        case _: return;
      }
    });
    var client = NativeTransport.connect("127.0.0.1", port + 1);
    var failure:Dynamic = null;
    try {
      var deadline = Sys.time() + 5.0;
      while (accepted == null && Sys.time() < deadline) {
        if (!runtime.events.poll()) runtime.events.wait(0.01);
      }
      var server = accepted;
      if (server == null) throw "scheduler test TCP accept timed out";
      var scheduler = new OutboundScheduler(server, 256);
      var legacyWriter = new MessagePackWriter();
      legacyWriter.writeMapHeader(4);
      legacyWriter.writeInt(1); legacyWriter.writeInt(1);
      legacyWriter.writeInt(2); legacyWriter.writeString("legacy");
      legacyWriter.writeInt(3); legacyWriter.writeString("robotkit-v1");
      legacyWriter.writeInt(4); legacyWriter.writeString("observer");
      var legacy = RobotProtocol.decodeHello(new RobotFrame(RobotMessageType.Hello,
        legacyWriter.getBytes()));
      if (legacy.subscriptions.length != 0) throw "missing Hello subscriptions did not decode empty";
      scheduler.configure(legacy.subscriptions);
      if (!scheduler.shouldOffer(OutboundFamily.Camera, "legacy", Int64.ofInt(1),
          Int64.ofInt(0))) throw "legacy client lost camera family";
      scheduler.configure([new StreamSubscription("sensor", 2.0)]);
      if (scheduler.shouldOffer(OutboundFamily.Camera, "filtered", Int64.ofInt(1),
          Int64.ofInt(0)) || !scheduler.subscribed(OutboundFamily.Essential))
        throw "subscription filtering failed";
      if (!scheduler.shouldOffer(OutboundFamily.Sensor, "rate", Int64.ofInt(1),
          Int64.ofInt(0)) || scheduler.shouldOffer(OutboundFamily.Sensor, "rate",
          Int64.ofInt(2), Int64.ofInt(250000000)) ||
          !scheduler.shouldOffer(OutboundFamily.Sensor, "rate", Int64.ofInt(3),
          Int64.ofInt(500000000))) throw "per-key subscription rate limiting failed";
      if (!scheduler.shouldOffer(OutboundFamily.Sensor, "other", Int64.ofInt(1),
          Int64.ofInt(250000000))) throw "rate limit was shared across sensor keys";
      scheduler.configure([new StreamSubscription("camera", 1.0)]);
      if (!scheduler.shouldOffer(OutboundFamily.Camera, "paced", Int64.ofInt(1),
          Int64.ofInt(0))) throw "initial paced frame was rejected";
      scheduler.offer(OutboundFamily.Camera, "paced", Int64.ofInt(1), frame("paced1"));
      if (!scheduler.flush(1) || receiveOne(client.borrow(), new RobotFrameStream(), runtime) !=
          "paced1") throw "initial paced frame was not delivered";
      if (!scheduler.shouldOffer(OutboundFamily.Camera, "paced", Int64.ofInt(2),
          Int64.ofInt(1000000000))) throw "next paced frame was rejected";
      scheduler.offer(OutboundFamily.Camera, "paced", Int64.ofInt(2), frame("paced2"));
      if (!scheduler.flush(1)) throw "rate-limited flush failed";
      var quietUntil = Sys.time() + 0.1;
      while (Sys.time() < quietUntil) {
        if (NativeTransport.receive(client.borrow()).length > 0)
          throw "per-key rate limit delivered frames too quickly";
        runtime.events.wait(0.01);
      }
      scheduler.configure([new StreamSubscription("sensor")]);
      scheduler.configure([]);
      var capabilities = robotd.OutboundScheduler.OutboundPolicy.capabilities();
      if (capabilities.indexOf("essential") < 0 || capabilities.indexOf("sensor") < 0 ||
          capabilities.indexOf("camera") < 0 || capabilities.indexOf("observation") < 0)
        throw "server capabilities omit an emitted family";
      var stream = new RobotFrameStream();
      scheduler.offer(OutboundFamily.Camera, "a", Int64.ofInt(1), frame("a1"));
      scheduler.offer(OutboundFamily.Camera, "b", Int64.ofInt(1), frame("b1"));
      scheduler.offer(OutboundFamily.Camera, "a", Int64.ofInt(2), frame("a2"));
      scheduler.offer(OutboundFamily.Camera, "a", Int64.ofInt(2), frame("duplicate"));
      if (scheduler.dropCount(OutboundFamily.Camera) != 1)
        throw "latest-wins replacement did not count one drop";
      if (!scheduler.flush(1)) throw "first scheduler send failed";
      var first = receiveOne(client.borrow(), stream, runtime);
      if (first != "a2") throw 'latest-wins sent $first instead of a2';
      scheduler.offer(OutboundFamily.Camera, "a", Int64.ofInt(3), frame("a3"));
      if (!scheduler.flush(1)) throw "second scheduler send failed";
      var second = receiveOne(client.borrow(), stream, runtime);
      if (second != "b1") throw 'round-robin sent $second instead of b1';
      if (!scheduler.flush(1)) throw "third scheduler send failed";
      var third = receiveOne(client.borrow(), stream, runtime);
      if (third != "a3") throw 'round-robin sent $third instead of a3';
      var numeric = new RobotFrame(RobotMessageType.SensorFrame, Bytes.ofString("imu1"));
      scheduler.offer(OutboundFamily.Sensor, "imu", Int64.ofInt(1), numeric);
      scheduler.offer(OutboundFamily.Sensor, "imu", Int64.ofInt(1), numeric);
      if (!scheduler.flush(1) || receiveOne(client.borrow(), stream, runtime) != "imu1")
        throw "numeric sensor sequence was not sent exactly once";
      if (scheduler.dropCount(OutboundFamily.Sensor) != 0)
        throw "camera drop affected numeric sensor counter";
      scheduler.configure([new StreamSubscription("observation")]);
      var observationFrame = new RobotFrame(RobotMessageType.ImageDetectionObservation,
        Bytes.ofString("detection"));
      scheduler.offer(OutboundFamily.Observation, "front", Int64.ofInt(1), observationFrame);
      scheduler.offer(OutboundFamily.Observation, "front", Int64.ofInt(2), observationFrame);
      if (scheduler.dropCount(OutboundFamily.Observation) != 1 || !scheduler.flush(1) ||
          receiveOne(client.borrow(), stream, runtime) != "detection")
        throw "per-producer observation latest-wins delivery failed";
      var oversized = new RobotFrame(RobotMessageType.CameraFrame, Bytes.alloc(300));
      scheduler.offer(OutboundFamily.Camera, "blocked", Int64.ofInt(1), oversized);
      if (!scheduler.flush(1)) throw "over-budget bulk frame failed the connection";
      scheduler.offer(OutboundFamily.Camera, "blocked", Int64.ofInt(2), oversized);
      if (scheduler.dropCount(OutboundFamily.Camera) != 2)
        throw "over-budget slot replacement did not count a drop";
      NativeTransport.send(server,
        new RobotFrame(RobotMessageType.RobotState, Bytes.ofString("essential")).encode());
      if (receiveOne(client.borrow(), stream, runtime) != "essential")
        throw "essential frame was blocked by over-budget bulk traffic";
      Sys.println("robotd outbound scheduler latest-wins and fairness passed");
    } catch (error:Dynamic) failure = error;
    subscription.dispose();
    client.close();
    var acceptedHandle = accepted;
    if (acceptedHandle != null) NativeTransport.close(acceptedHandle);
    listener.close();
    runtime.dispose();
    if (failure != null) throw failure;
  }

  static function frame(value:String):RobotFrame {
    return new RobotFrame(RobotMessageType.CameraFrame, Bytes.ofString(value));
  }

  static function receiveOne(client:TransportHandle, stream:RobotFrameStream,
      runtime:NativeKitRuntime):String {
    var deadline = Sys.time() + 5.0;
    while (Sys.time() < deadline) {
      var bytes = NativeTransport.receive(client);
      if (bytes.length > 0) {
        var frames = stream.push(bytes);
        if (frames.length > 0) return frames[0].payload.toString();
      }
      runtime.events.wait(0.01);
    }
    throw "scheduler frame was not received";
  }
}
