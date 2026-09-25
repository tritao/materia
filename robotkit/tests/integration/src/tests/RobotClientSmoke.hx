package tests;

import haxe.io.Bytes;
import robotkit.client.RobotClient;
import robotkit.protocol.RobotFrame;
import robotkit.protocol.RobotFrame.RobotFrameStream;
import robotkit.protocol.RobotMessageType;

/** Small end-to-end client proving the normal editor-to-robotd TCP path. */
class RobotClientSmoke {
  public static function run(host:String, port:Int):Void {
    verifyFrameCodec();
    var client = new RobotClient("materia-cli");
    var targetStateReceived = false;
    var targetSent = false;
    var stateCount = 0;
    client.stateListener = function(state) {
      stateCount++;
      if (state.q.length > 0 && state.q[0] == 0.5)
        targetStateReceived = true;
    };
    try {
      client.connect(host, port);
      var attempts = 0;
      while (!targetStateReceived && attempts < 500) {
        var hadEvent = client.poll();
        if (client.isReady() && !targetSent) {
          client.sendJointTarget(0, 1, 0.5);
          targetSent = true;
        }
        if (!hadEvent)
          client.wait(0.01);
        attempts++;
      }
      if (!targetStateReceived)
        throw "robotd TCP smoke exchange timed out";
      if (stateCount < 3)
        throw "robotd TCP smoke exchange did not receive periodic snapshots";
      Sys.println('robotd client: received state q0=0.5 over TCP ($stateCount snapshots)');
    } catch (error:Dynamic) {
      Sys.println('robotd client error: ${Std.string(error)}');
      client.close();
      throw error;
    }
    client.close();
  }

  static function verifyFrameCodec():Void {
    var original = new RobotFrame(RobotMessageType.Hello, Bytes.ofString("probe"), -1,
      [Bytes.ofString("attachment")]);
    var encoded = original.encode();
    var stream = new RobotFrameStream();
    var decoded:Array<RobotFrame> = [];
    var offset = 0;
    var width = 1;
    while (offset < encoded.length) {
      var amount = encoded.length - offset < width ? encoded.length - offset : width;
      for (frame in stream.push(Bytes.view(encoded, offset, amount)))
        decoded.push(frame);
      offset += amount;
      width = width == 1 ? 3 : width == 3 ? 7 : encoded.length;
    }
    if (decoded.length != 1 || stream.pendingBytes() != 0 || decoded[0].flags != -1
        || decoded[0].attachments.length != 1)
      throw "RobotFrame segmented buffering or u32 flag round-trip failed";
  }
}
