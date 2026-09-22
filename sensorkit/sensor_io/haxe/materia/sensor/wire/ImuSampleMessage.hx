package materia.sensor.wire;

import haxe.Int64;
import haxe.io.Bytes;

/**
 * Haxeon representation of SensorKit's v1 packed IMU wire value.
 *
 * `data` contains 24 little-endian IEEE-754 binary64 values in this order:
 * angular velocity xyz, linear acceleration xyz, angular-velocity covariance
 * row-major, and linear-acceleration covariance row-major.
 */
@:wire
class ImuSampleMessage {
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
	public var data:Bytes;
}
