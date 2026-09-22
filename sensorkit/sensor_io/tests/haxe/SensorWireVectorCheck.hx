import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePackFrame;
import materia.sensor.wire.ImuSampleMessage;
import materia.sensor.wire.LidarScanMessage;
import materia.sensor.wire.PackedFrameMessage;
import materia.sensor.wire.SensorWireCodec;
import materia.sensor.wire.MessageType;
import materia.sensor.wire.PixelFormat;
import sys.io.File;

/** Generated interop check. Compile with Haxeon and pass the generated TSV path. */
class SensorWireVectorCheck {
	static function main():Int {
		var expected = readVectors(Sys.args()[0]);
		var packed = new PackedFrameMessage();
		packed.schemaVersion = 1;
		packed.messageType = MessageType.cameraFrame;
		packed.sensor = Int64.parseString("40");
		packed.sequence = Int64.parseString("7");
		packed.captureTime = 3.0;
		packed.deliveryTime = 3.125;
		packed.frame = Int64.parseString("9");
		packed.width = Int64.parseString("2");
		packed.height = Int64.parseString("1");
		packed.stride = Int64.parseString("8");
		packed.pixelFormat = PixelFormat.rgba8;
		packed.data = bytesFromHex("ff0000ff00ff00ff");
		var packedBytes = SensorWireCodec.encodePackedFrameMessage(packed);
		verify(expected, "PackedFrameMessage", MessagePackFrame.pack(packedBytes));
		SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(getVector(expected, "PackedFrameMessage")));
		expectRejected(expected, "PackedFrameMessage.invalid.schema_version", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.missing_schema_version", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.sensor", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.sequence", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.frame", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.duplicate_field", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.missing_field", "PackedFrameMessage");
		expectRejected(expected, "PackedFrameMessage.invalid.trailing_data", "PackedFrameMessage");
		SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(getVector(expected, "PackedFrameMessage.unknown_field")));
		SensorWireCodec.decodePackedFrameMessage(MessagePackFrame.unpack(getVector(expected, "PackedFrameMessage.uint32_max_width")));
		expectRejected(expected, "PackedFrameMessage.invalid.width_range", "PackedFrameMessage");
		var imu = new ImuSampleMessage();
		imu.schemaVersion = 1;
		imu.messageType = MessageType.imuSample;
		imu.sensor = Int64.parseString("61");
		imu.sequence = Int64.parseString("14");
		imu.captureTime = 8.0;
		imu.deliveryTime = 8.04;
		imu.frame = Int64.parseString("17");
		imu.data = bytesFromHex("000000000000f03f00000000000004c00000000000000a4000000000000010c000000000000016400000000000001bc09a9999999999b93f9a9999999999c93f333333333333d33f9a9999999999d93f000000000000e03f333333333333e33f666666666666e63f9a9999999999e93fcdccccccccccec3f9a9999999999f13f333333333333f33fcdccccccccccf43f666666666666f63f000000000000f83f9a9999999999f93f333333333333fb3fcdccccccccccfc3f666666666666fe3f");
		var imuBytes = SensorWireCodec.encodeImuSampleMessage(imu);
		verify(expected, "ImuSampleMessage", MessagePackFrame.pack(imuBytes));
		SensorWireCodec.decodeImuSampleMessage(MessagePackFrame.unpack(getVector(expected, "ImuSampleMessage")));
		expectRejected(expected, "ImuSampleMessage.invalid.schema_version", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.missing_schema_version", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.message_type", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.missing_message_type", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.sensor", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.sequence", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.frame", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.duplicate_field", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.missing_field", "ImuSampleMessage");
		expectRejected(expected, "ImuSampleMessage.invalid.trailing_data", "ImuSampleMessage");
		var lidar = new LidarScanMessage();
		lidar.schemaVersion = 1;
		lidar.messageType = MessageType.lidarScan;
		lidar.sensor = Int64.parseString("63");
		lidar.sequence = Int64.parseString("15");
		lidar.captureTime = 9.0;
		lidar.deliveryTime = 9.05;
		lidar.frame = Int64.parseString("19");
		lidar.horizontalCount = Int64.parseString("3");
		lidar.verticalCount = Int64.parseString("1");
		lidar.returnStride = Int64.parseString("12");
		lidar.data = bytesFromHex("000020400000403f010000000000c8420000000000000000000040410000904001000000");
		var lidarBytes = SensorWireCodec.encodeLidarScanMessage(lidar);
		verify(expected, "LidarScanMessage", MessagePackFrame.pack(lidarBytes));
		SensorWireCodec.decodeLidarScanMessage(MessagePackFrame.unpack(getVector(expected, "LidarScanMessage")));
		expectRejected(expected, "LidarScanMessage.invalid.schema_version", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.missing_schema_version", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.message_type", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.missing_message_type", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.sensor", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.sequence", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.frame", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.return_stride", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.missing_return_stride", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.duplicate_field", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.missing_field", "LidarScanMessage");
		expectRejected(expected, "LidarScanMessage.invalid.trailing_data", "LidarScanMessage");
		return 42;
	}

	static function readVectors(path:String):Map<String, Bytes> {
		var result:Map<String, Bytes> = [];
		for (line in File.getContent(path).split("\n")) {
			var text = StringTools.trim(line);
			if (text == "" || StringTools.startsWith(text, "#")) continue;
			var pair = text.split("\t");
			result.set(pair[0], bytesFromHex(pair[1]));
		}
		return result;
	}

	static function verify(expected:Map<String, Bytes>, name:String, actual:Bytes):Void {
		var vector = expected.get(name);
		if (vector == null || vector.compare(actual) != 0) throw 'Sensor wire vector differs: $name';
	}

	static function getVector(expected:Map<String, Bytes>, name:String):Bytes {
		var value = expected.get(name);
		if (value == null) throw 'Missing Sensor wire vector: $name';
		return value;
	}

	static function expectRejected(expected:Map<String, Bytes>, name:String, message:String):Void {
		var bytes = MessagePackFrame.unpack(getVector(expected, name));
		var rejected = false;
		try {
			switch (message) {
				case "PackedFrameMessage": SensorWireCodec.decodePackedFrameMessage(bytes);
				case "ImuSampleMessage": SensorWireCodec.decodeImuSampleMessage(bytes);
				case "LidarScanMessage": SensorWireCodec.decodeLidarScanMessage(bytes);
				default: throw 'Unknown Sensor wire message: $message';
			}
		} catch (error:Dynamic) {
			rejected = true;
		}
		if (!rejected) throw 'Sensor wire decoder accepted invalid vector: $name';
	}

	static function bytesFromHex(text:String):Bytes {
		var bytes = Bytes.alloc(text.length >> 1);
		for (index in 0...bytes.length)
			bytes.set(index, (hexDigit(text.charCodeAt(index * 2)) << 4) | hexDigit(text.charCodeAt(index * 2 + 1)));
		return bytes;
	}

	static function hexDigit(code:Int):Int {
		if (code >= 48 && code <= 57) return code - 48;
		if (code >= 65 && code <= 70) return code - 55;
		return code - 87;
	}
}
