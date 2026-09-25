package materia.sensor.wire;

import haxe.Int64;
import haxe.io.Bytes;

/** Generated from schema/sensor_wire.wire.idl. Do not edit by hand. */
@:wire
class PackedFrameMessage {
	@:id(1)
	public var schemaVersion:Int;
	@:id(2)
	public var messageType:Int;
	@:id(3)
	public var sensor:Int64;
	@:id(4)
	public var sequence:Int64;
	@:id(5)
	public var captureTime:Float;
	@:id(6)
	public var deliveryTime:Float;
	@:id(7)
	public var frame:Int64;
	@:id(8)
	public var width:Int64;
	@:id(9)
	public var height:Int64;
	@:id(10)
	public var stride:Int64;
	@:id(11)
	public var pixelFormat:Int;
	@:id(12)
	public var data:Bytes;
}
