package materia.sensor.wire;

import haxe.Int64;
import haxe.io.Bytes;

/**
 * Haxeon representation of SensorKit's v1 packed LiDAR wire value.
 *
 * The binary data contains one fixed 12-byte record per ray: little-endian
 * float32 range, little-endian float32 intensity, one hit flag, and three
 * reserved bytes. Records are ordered vertical-major, then horizontal.
 */
@:wire
class LidarScanMessage {
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
	public var horizontalCount:Int;
	@:id(9)
	public var verticalCount:Int;
	@:id(10)
	public var returnStride:Int;
	@:id(11)
	public var data:Bytes;
}
