package robotkit.recording;

import robotkit.core.RobotId;

import haxe.io.Bytes;

/** A typed @:wire payload and its conversion to a recording event. */
interface RecordingChannel<T> {
  function name():String;
  function wireClass():String;
  function schemaData():String;
  function toWire(entry:RobotRecordingEntry):T;
  function fromWire(value:T):RobotRecordingEvent;
  function robotId(value:T):RobotId;
  function encode(value:T):Bytes;
  function decode(bytes:Bytes):T;
}
