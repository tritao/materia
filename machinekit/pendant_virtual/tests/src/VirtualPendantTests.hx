import haxe.io.Bytes;
import machinekit.operator.VirtualPendant;

class VirtualPendantTests {
	static function check(condition:Bool, what:String):Void {
		if (!condition)
			throw what;
	}

	static function lit(pixels:Bytes):Int {
		var count = 0;
		for (i in 0...pixels.length)
			if (pixels.get(i) != 0)
				count++;
		return count;
	}

	static function main():Void {
		var pendant = new VirtualPendant(0);
		var pixels = pendant.pixels(0);
		check(pixels.length == VirtualPendant.WIDTH * VirtualPendant.HEIGHT, "the OLED is 128x64");
		check(lit(pixels) > 100, "NO LINK is drawn");
		check(pendant.checksum(0) != 0, "the frame has a checksum");
		check(pendant.displayPower(0) == VirtualPendant.POWER_ON, "awake at power-up");

		check(pendant.tick(10, 0, 3, 1, 0, true, true), "a state frame is due at 10 ms");
		check(pendant.frame().length == 8, "PENDANT_STATE is 8 bytes");
		check(!pendant.tick(11, 0, 3, 1, 0, true, true), "none is due a millisecond later");
		check(!pendant.onFrame(12, 0x202, Bytes.alloc(0)), "an empty controller frame is refused");
		check(pendant.leds(12) == 0, "no LEDs without a controller");

		check(lit(pendant.pixels(70000)) == 0, "blanked after a minute idle");
		check(pendant.displayPower(70000) == VirtualPendant.POWER_OFF, "reports the display off");
		pendant.close();
		trace("virtual pendant tests passed");
	}
}
