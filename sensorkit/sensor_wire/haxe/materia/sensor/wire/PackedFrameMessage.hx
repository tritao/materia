package materia.sensor.wire;

import haxe.Int64;
import haxe.io.Bytes;

/**
 * Haxeon representation of SensorKit's v1 packed-frame wire value.
 *
 * Camera, depth, segmentation, and future image-like sensors share this one
 * record. `messageType` and `pixelFormat` select the interpretation of `data`.
 * The native encoder wraps MessagePack.encode(PackedFrameMessage) in
 * haxeon.wire.MessagePackFrame.
 */
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
	public var width:Int;
	@:id(9)
	public var height:Int;
	@:id(10)
	public var stride:Int;
	@:id(11)
	public var pixelFormat:Int;
	@:id(12)
	public var data:Bytes;
}
