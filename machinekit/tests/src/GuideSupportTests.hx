import machinekit.motion.GuideSupport;
import machinekit.motion.GuideSupport.GuideSupportWrench;

/** Balance the resulting reactions independently, rather than repeat their construction. */
class GuideSupportTests {
	static function near(actual:Float, expected:Float, message:String):Void {
		if (!Math.isFinite(actual) || Math.abs(actual - expected) > 1e-8)
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		var support = new GuideSupport(0.14, 0.0654);
		var cases:Array<GuideSupportWrench> = [
			{fx: 0, fy: 1000, fz: 0, mx: 0, my: 0, mz: 0},
			{fx: 170, fy: -2300, fz: 480, mx: 63, my: -37, mz: 121},
			{fx: -170, fy: 2300, fz: -480, mx: -63, my: 37, mz: -121}
		];
		for (wrench in cases) {
			var fx = 0.0, fy = 0.0, fz = 0.0;
			var mx = 0.0, my = 0.0, mz = 0.0;
			for (load in support.loads(wrench)) {
				fx += load.fx; fy += load.fy; fz += load.fz;
				mx -= load.z * load.fy;
				my += load.z * load.fx - load.x * load.fz;
				mz += load.x * load.fy;
			}
			near(fx, wrench.fx, "guide X force balance");
			near(fy, wrench.fy, "guide Y force balance");
			near(fz, wrench.fz, "guide Z force balance");
			near(mx, wrench.mx, "guide X moment balance");
			near(my, wrench.my, "guide Y moment balance");
			near(mz, wrench.mz, "guide Z moment balance");
		}
		near(support.utilization(cases[0], 1000, 2), 0.5, "equal gravity sharing");
		if (!(support.utilization(cases[1], 100, 2) > 1))
			throw "Guide support must report an overloaded bearing";
		var wider = new GuideSupport(0.28, 0.1308);
		if (!(wider.utilization(cases[1], 1000, 2) < support.utilization(cases[1], 1000, 2)))
			throw "Wider support must reduce the moment-induced bearing load";
	}
}
