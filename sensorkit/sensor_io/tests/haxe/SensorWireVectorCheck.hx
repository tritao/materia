import haxe.Int64;
import haxe.io.Bytes;
import haxeon.wire.MessagePack;
import haxeon.wire.MessagePackFrame;
import materia.sensor.wire.ImuSampleMessage;
import materia.sensor.wire.LidarScanMessage;
import materia.sensor.wire.PackedFrameMessage;
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
		packed.width = 2;
		packed.height = 1;
		packed.stride = 8;
		packed.pixelFormat = PixelFormat.rgba8;
		packed.data = bytesFromHex("ff0000ff00ff00ff");
		verify(expected, "PackedFrameMessage", MessagePackFrame.pack(MessagePack.encode(packed)));
		var imu = new ImuSampleMessage();
		imu.schemaVersion = 1;
		imu.messageType = MessageType.imuSample;
		imu.sensor = Int64.parseString("61");
		imu.sequence = Int64.parseString("14");
		imu.captureTime = 8.0;
		imu.deliveryTime = 8.04;
		imu.frame = Int64.parseString("17");
		imu.data = bytesFromHex("000000000000f03f00000000000004c00000000000000a4000000000000010c000000000000016400000000000001bc09a9999999999b93f9a9999999999c93f333333333333d33f9a9999999999d93f000000000000e03f333333333333e33f666666666666e63f9a9999999999e93fcdccccccccccec3f9a9999999999f13f333333333333f33fcdccccccccccf43f666666666666f63f000000000000f83f9a9999999999f93f333333333333fb3fcdccccccccccfc3f666666666666fe3f");
		verify(expected, "ImuSampleMessage", MessagePackFrame.pack(MessagePack.encode(imu)));
		var lidar = new LidarScanMessage();
		lidar.schemaVersion = 1;
		lidar.messageType = MessageType.lidarScan;
		lidar.sensor = Int64.parseString("63");
		lidar.sequence = Int64.parseString("15");
		lidar.captureTime = 9.0;
		lidar.deliveryTime = 9.05;
		lidar.frame = Int64.parseString("19");
		lidar.horizontalCount = 3;
		lidar.verticalCount = 1;
		lidar.returnStride = 12;
		lidar.data = bytesFromHex("000020400000403f010000000000c8420000000000000000000040410000904001000000");
		verify(expected, "LidarScanMessage", MessagePackFrame.pack(MessagePack.encode(lidar)));
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
