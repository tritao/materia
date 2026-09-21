package app;

import NativeKitEvents;
import robotkit.client.RobotClient;
import robotkit.protocol.Fault;
import robotkit.protocol.RobotCapabilities;
import robotkit.protocol.RobotDescription;
import robotkit.protocol.RobotStateMsg;

/** Editor-owned view of one authoritative robotd connection. */
class RobotConnection {
  public final client:RobotClient;
  final pump:NativeKitEvents;
  public var host:Null<String> = null;
  public var port:Int = 0;

  public function new(events:NativeKitEvents) {
    pump = events;
    client = new RobotClient("materia-editor");
  }

  public function connect(host:String, port:Int):Void {
    this.host = host;
    this.port = port;
    client.connectWithEvents(host, port, pump);
  }

  public function close():Void
    client.close();

  public function status():String {
    if (client.lastFault != null && client.lastFault.fatal)
      return "fault";
    if (client.isReady())
      return "ready";
    if (client.isConnected())
      return "connecting";
    return "disconnected";
  }

  public function state():Null<RobotStateMsg>
    return client.latestState;

  public function fault():Null<Fault>
    return client.lastFault;

  public function description():Null<RobotDescription>
    return client.description;

  public function capabilities():Null<RobotCapabilities>
    return client.capabilities;

}
