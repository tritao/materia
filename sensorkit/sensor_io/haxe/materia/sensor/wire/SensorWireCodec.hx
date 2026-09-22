package materia.sensor.wire;

import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import haxeon.wire.MessagePackReader;
import materia.sensor.wire.PackedFrameMessage;
import materia.sensor.wire.ImuSampleMessage;
import materia.sensor.wire.LidarScanMessage;

/** Generated MessagePack-payload entry points; wrap or unwrap HMPK separately. */
class SensorWireCodec {
	public static function encodePackedFrameMessage(value:PackedFrameMessage):Bytes {
		validatePackedFrameMessage(value);
		return MessagePack.encode(value);
	}

	public static function decodePackedFrameMessage(bytes:Bytes):PackedFrameMessage {
		validateKeys(bytes, "PackedFrameMessage", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
		var decoded:PackedFrameMessage = MessagePack.decode(bytes);
		validatePackedFrameMessage(decoded);
		return decoded;
	}

	static function validatePackedFrameMessage(value:PackedFrameMessage):Void {
		if (value.schemaVersion != 1) throw "invalid constant field PackedFrameMessage.schema_version";
		if (value.schemaVersion < 0 || value.schemaVersion > 255) throw "PackedFrameMessage.schema_version is out of range";
		if (value.messageType < 0 || value.messageType > 255) throw "PackedFrameMessage.message_type is out of range";
		if (Int64.compare(value.sensor, Int64.ofInt(0)) < 0) throw "PackedFrameMessage.sensor is out of range";
		if (Int64.compare(value.sequence, Int64.ofInt(0)) < 0) throw "PackedFrameMessage.sequence is out of range";
		if (Int64.compare(value.frame, Int64.ofInt(0)) < 0) throw "PackedFrameMessage.frame is out of range";
		if (Int64.compare(value.width, Int64.ofInt(0)) < 0 || Int64.compare(value.width, Int64.make(0, -1)) > 0) throw "PackedFrameMessage.width is out of range";
		if (Int64.compare(value.height, Int64.ofInt(0)) < 0 || Int64.compare(value.height, Int64.make(0, -1)) > 0) throw "PackedFrameMessage.height is out of range";
		if (Int64.compare(value.stride, Int64.ofInt(0)) < 0 || Int64.compare(value.stride, Int64.make(0, -1)) > 0) throw "PackedFrameMessage.stride is out of range";
		if (value.pixelFormat < 0 || value.pixelFormat > 255) throw "PackedFrameMessage.pixel_format is out of range";
	}

	public static function encodeImuSampleMessage(value:ImuSampleMessage):Bytes {
		validateImuSampleMessage(value);
		return MessagePack.encode(value);
	}

	public static function decodeImuSampleMessage(bytes:Bytes):ImuSampleMessage {
		validateKeys(bytes, "ImuSampleMessage", [1, 2, 3, 4, 5, 6, 7, 8]);
		var decoded:ImuSampleMessage = MessagePack.decode(bytes);
		validateImuSampleMessage(decoded);
		return decoded;
	}

	static function validateImuSampleMessage(value:ImuSampleMessage):Void {
		if (value.schemaVersion != 1) throw "invalid constant field ImuSampleMessage.schema_version";
		if (value.schemaVersion < 0 || value.schemaVersion > 255) throw "ImuSampleMessage.schema_version is out of range";
		if (value.messageType != 4) throw "invalid constant field ImuSampleMessage.message_type";
		if (value.messageType < 0 || value.messageType > 255) throw "ImuSampleMessage.message_type is out of range";
		if (Int64.compare(value.sensor, Int64.ofInt(0)) < 0) throw "ImuSampleMessage.sensor is out of range";
		if (Int64.compare(value.sequence, Int64.ofInt(0)) < 0) throw "ImuSampleMessage.sequence is out of range";
		if (Int64.compare(value.frame, Int64.ofInt(0)) < 0) throw "ImuSampleMessage.frame is out of range";
	}

	public static function encodeLidarScanMessage(value:LidarScanMessage):Bytes {
		validateLidarScanMessage(value);
		return MessagePack.encode(value);
	}

	public static function decodeLidarScanMessage(bytes:Bytes):LidarScanMessage {
		validateKeys(bytes, "LidarScanMessage", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
		var decoded:LidarScanMessage = MessagePack.decode(bytes);
		validateLidarScanMessage(decoded);
		return decoded;
	}

	static function validateLidarScanMessage(value:LidarScanMessage):Void {
		if (value.schemaVersion != 1) throw "invalid constant field LidarScanMessage.schema_version";
		if (value.schemaVersion < 0 || value.schemaVersion > 255) throw "LidarScanMessage.schema_version is out of range";
		if (value.messageType != 5) throw "invalid constant field LidarScanMessage.message_type";
		if (value.messageType < 0 || value.messageType > 255) throw "LidarScanMessage.message_type is out of range";
		if (Int64.compare(value.sensor, Int64.ofInt(0)) < 0) throw "LidarScanMessage.sensor is out of range";
		if (Int64.compare(value.sequence, Int64.ofInt(0)) < 0) throw "LidarScanMessage.sequence is out of range";
		if (Int64.compare(value.frame, Int64.ofInt(0)) < 0) throw "LidarScanMessage.frame is out of range";
		if (Int64.compare(value.horizontalCount, Int64.ofInt(0)) < 0 || Int64.compare(value.horizontalCount, Int64.make(0, -1)) > 0) throw "LidarScanMessage.horizontal_count is out of range";
		if (Int64.compare(value.verticalCount, Int64.ofInt(0)) < 0 || Int64.compare(value.verticalCount, Int64.make(0, -1)) > 0) throw "LidarScanMessage.vertical_count is out of range";
		if (Int64.compare(value.returnStride, Int64.parseString("12")) != 0) throw "invalid constant field LidarScanMessage.return_stride";
		if (Int64.compare(value.returnStride, Int64.ofInt(0)) < 0 || Int64.compare(value.returnStride, Int64.make(0, -1)) > 0) throw "LidarScanMessage.return_stride is out of range";
	}

	static function validateKeys(bytes:Bytes, message:String, required:Array<Int>):Void {
		var reader = new MessagePackReader(bytes);
		var count = reader.readMapHeader();
		var seen:Array<Int> = [];
		for (_ in 0...count) {
			var key = reader.readInt();
			if (key < 0) throw 'negative field ID $key in $message';
			if (seen.indexOf(key) >= 0) throw 'duplicate field ID $key in $message';
			seen.push(key);
			reader.skip();
		}
		for (id in required)
			if (seen.indexOf(id) < 0) throw 'missing required field ID $id in $message';
		if (!reader.atEnd()) throw 'trailing bytes after $message';
	}
}
