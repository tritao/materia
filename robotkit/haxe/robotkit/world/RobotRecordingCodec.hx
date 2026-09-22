package robotkit.world;

import haxe.Int64;
import haxe.Json;
import haxe.io.Bytes;

/** Stable JSON payload contract stored inside MCAP messages. Wide integers are strings. */
class RobotRecordingCodec {
  public static inline final VERSION:Int = 1;

  public static function encode(entry:RobotRecordingEntry):Bytes {
    var root:Dynamic = {
      version: VERSION,
      ordinal: Int64.toStr(entry.ordinal),
      recordingTimestampNs: Int64.toStr(entry.recordingTimestampNs),
      robotId: entry.robotId,
      sourceSequence: Int64.toStr(entry.sourceSequence),
      sourceTimestampNs: Int64.toStr(entry.sourceTimestampNs),
      sourceClockId: entry.sourceClockId
    };
    switch entry.event {
      case Command(value):
        Reflect.setField(root, "type", "command");
        switch value { case JointPosition(joint, target, expiryNs):
          Reflect.setField(root, "payload", {kind:"jointPosition", joint:joint, target:target,
            expiryNs:expiryNs == null ? null : Int64.toStr(expiryNs)});
        }
      case Snapshot(value): Reflect.setField(root, "type", "snapshot"); Reflect.setField(root, "payload", snapshot(value));
      case Sensor(robotId, value): Reflect.setField(root, "type", "sensor"); Reflect.setField(root, "robotId", robotId); Reflect.setField(root, "payload", sensor(value));
      case Fault(value): Reflect.setField(root, "type", "fault"); Reflect.setField(root, "payload", {id:value.id, code:value.code, message:value.message, fatal:value.fatal});
      case World(value):
        Reflect.setField(root, "type", "world");
        Reflect.setField(root, "payload", {sequence:Std.string(value.sequence), topologyRevision:Std.string(value.topologyRevision),
          sourceTimestampNs:Int64.toStr(value.sourceTimestampNs), receivedTimestampNs:Int64.toStr(value.receivedTimestampNs),
          robots:[for (robot in value.robots()) snapshot(robot)]});
      case WorldEvent(value):
        Reflect.setField(root, "type", "worldEvent");
        Reflect.setField(root, "payload", switch value {case RobotAttached(id):{kind:"attached",robotId:id};case RobotDetached(id):{kind:"detached",robotId:id};case RobotChanged(id):{kind:"changed",robotId:id};});
    }
    return Bytes.ofString(Json.stringify(root));
  }

  public static function decode(bytes:Bytes):RobotRecordingEntry {
    var root:Dynamic;
    try root = Json.parse(bytes.toString()) catch (_:Dynamic) throw "Malformed RobotKit recording payload";
    if (fieldInt(root, "version") != VERSION) throw "Unsupported RobotKit recording schema";
    var ordinal = wide(root, "ordinal"), robotId = string(root, "robotId");
    var sequence = wide(root, "sourceSequence"), timestamp = wide(root, "sourceTimestampNs");
    var clock = string(root, "sourceClockId"), payload:Dynamic = Reflect.field(root, "payload");
    var event:RobotRecordingEvent = switch string(root, "type") {
      case "command":
        if (string(payload,"kind") != "jointPosition") throw "Unsupported RobotKit command payload";
        Command(JointPosition(fieldInt(payload,"joint"), fieldFloat(payload,"target"), nullableWide(payload,"expiryNs")));
      case "snapshot": Snapshot(readSnapshot(payload));
      case "sensor": Sensor(robotId, readSensor(payload));
      case "fault": Fault(new RobotFault(string(payload,"id"),fieldInt(payload,"code"),string(payload,"message"),fieldBool(payload,"fatal")));
      case "world":
        var robots = new Map<RobotId,RobotSnapshot>();
        for (item in array(payload,"robots")) { var value=readSnapshot(item); robots.set(value.id,value); }
        World(new WorldSnapshot(fieldIntString(payload,"sequence"),fieldIntString(payload,"topologyRevision"),wide(payload,"sourceTimestampNs"),robots,wide(payload,"receivedTimestampNs")));
      case "worldEvent":
        var id=string(payload,"robotId");
        WorldEvent(switch string(payload,"kind") {case "attached":RobotAttached(id);case "detached":RobotDetached(id);case "changed":RobotChanged(id);case _:throw "Unsupported RobotKit world event";});
      case _: throw "Unsupported RobotKit recording event type";
    };
    return new RobotRecordingEntry(ordinal, robotId, event, sequence, timestamp, clock,
      wide(root, "recordingTimestampNs"));
  }

  static function snapshot(v:RobotSnapshot):Dynamic return {id:v.id, sourceSequence:Int64.toStr(v.sourceSequence),sourceTimestampNs:Int64.toStr(v.sourceTimestampNs),receivedTimestampNs:Int64.toStr(v.receivedTimestampNs),sourceClockId:v.sourceClockId,receivedClockId:v.receivedClockId,positions:v.positions.toArray(),velocities:v.velocities.toArray(),efforts:v.efforts.toArray(),mode:v.mode,faultCode:v.faultCode,sensors:[for(s in v.sensors.toArray()) sensor(s)]};
  static function sensor(v:SensorFrame):Dynamic return {sensorId:v.sensorId,kind:v.kind,frameId:v.frameId,sequence:Int64.toStr(v.sequence),sourceTimestampNs:Int64.toStr(v.sourceTimestampNs),receivedTimestampNs:Int64.toStr(v.receivedTimestampNs),sourceClockId:v.sourceClockId,receivedClockId:v.receivedClockId,values:v.values.toArray(),linkId:v.linkId,mountPosition:v.mountPosition.toArray(),mountRotation:v.mountRotation.toArray()};
  static function readSnapshot(v:Dynamic):RobotSnapshot return new RobotSnapshot(string(v,"id"),wide(v,"sourceSequence"),wide(v,"sourceTimestampNs"),floats(v,"positions"),floats(v,"velocities"),floats(v,"efforts"),fieldInt(v,"mode"),fieldInt(v,"faultCode"),wide(v,"receivedTimestampNs"),[for(s in array(v,"sensors")) readSensor(s)],string(v,"sourceClockId"),string(v,"receivedClockId"));
  static function readSensor(v:Dynamic):SensorFrame {
    var position = floats(v, "mountPosition");
    var rotation = floats(v, "mountRotation");
    if (position.length != 3) throw "Sensor mount position must contain three values";
    if (rotation.length != 4) throw "Sensor mount rotation must contain four values";
    var norm = 0.0;
    for (value in rotation) norm += value * value;
    if (!Math.isFinite(norm) || Math.abs(norm - 1.0) > 0.000001)
      throw "Sensor mount rotation must be a unit quaternion";
    return new SensorFrame(string(v,"sensorId"),string(v,"kind"),string(v,"frameId"),
      wide(v,"sequence"),wide(v,"sourceTimestampNs"),floats(v,"values"),
      wide(v,"receivedTimestampNs"),string(v,"linkId"),position,rotation,
      string(v,"sourceClockId"),string(v,"receivedClockId"));
  }
  static function string(v:Dynamic,n:String):String {var x=Reflect.field(v,n);if(!Std.isOfType(x,String))throw 'Invalid recording field $n';return x;}
  static function wide(v:Dynamic,n:String):Int64 {
    try {
      return Int64.parseString(string(v,n));
    } catch(_:Dynamic) {
      throw 'Invalid recording integer $n';
    }
  }
  static function nullableWide(v:Dynamic,n:String):Null<Int64> {var x=Reflect.field(v,n);return x==null?null:wide(v,n);}
  static function fieldInt(v:Dynamic,n:String):Int {var x=Reflect.field(v,n);if(!Std.isOfType(x,Int))throw 'Invalid recording field $n';return x;}
  static function fieldIntString(v:Dynamic,n:String):Int {var x=Std.parseInt(string(v,n));if(x==null)throw 'Invalid recording field $n';return x;}
  static function fieldFloat(v:Dynamic,n:String):Float {var x=Reflect.field(v,n);if(!Std.isOfType(x,Float)&&!Std.isOfType(x,Int))throw 'Invalid recording field $n';var result:Float=x;if(!Math.isFinite(result))throw 'Non-finite recording field $n';return result;}
  static function fieldBool(v:Dynamic,n:String):Bool {var x=Reflect.field(v,n);if(!Std.isOfType(x,Bool))throw 'Invalid recording field $n';return x;}
  static function array(v:Dynamic,n:String):Array<Dynamic> {var x=Reflect.field(v,n);if(!Std.isOfType(x,Array))throw 'Invalid recording field $n';return cast x;}
  static function floats(v:Dynamic,n:String):Array<Float> return [for(x in array(v,n)) {if(!Std.isOfType(x,Float)&&!Std.isOfType(x,Int))throw 'Invalid recording field $n';var result:Float=cast x;if(!Math.isFinite(result))throw 'Non-finite recording field $n';result;}];
}
