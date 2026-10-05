package machinekit.pressbrake;

/** Reference tooling for the 1600 mm demonstration press.
 * Contact angles and R0.6 are the plan's reference values. Body dimensions are explicit
 * assumptions, rather than a manufacturer's drawing or a tooling strength rating.
 */
class ToolingCatalogue {
	public static function punches():Array<PunchProfile> return [straight(), gooseneck(), acute()];

	public static function straight():PunchProfile return tapered("reference-straight-88-r06", Straight, 88 * Math.PI / 180, 100);

	public static function acute():PunchProfile return tapered("reference-acute-30-r06", Acute, Math.PI / 6, 100);

	static function tapered(id:String, kind:PunchKind, angle:Float, height:Float):PunchProfile {
		var radius = 0.6;
		var z = radius * (1 - Math.sin(angle / 2)), y = radius * Math.cos(angle / 2);
		var shoulderY = y + (15 - z) * Math.tan(angle / 2);
		return new PunchProfile(id, kind, radius, angle, height, new ToolProfilePolygon([
			new ToolProfilePoint(-y, z), new ToolProfilePoint(y, z),
			new ToolProfilePoint(shoulderY, 15), new ToolProfilePoint(8, 30),
			new ToolProfilePoint(8, height), new ToolProfilePoint(-8, height),
			new ToolProfilePoint(-8, 30), new ToolProfilePoint(-shoulderY, 15)
		]));
	}

	public static function gooseneck():PunchProfile {
		var radius = 0.6, angle = 88 * Math.PI / 180;
		var z = radius * (1 - Math.sin(angle / 2)), y = radius * Math.cos(angle / 2);
		var shoulderY = y + (15 - z) * Math.tan(angle / 2);
		return new PunchProfile("reference-gooseneck-88-r06", Gooseneck, radius, angle, 120,
			new ToolProfilePolygon([
				new ToolProfilePoint(-y, z), new ToolProfilePoint(y, z),
				new ToolProfilePoint(shoulderY, 15), new ToolProfilePoint(20, 25),
				new ToolProfilePoint(-5, 35), new ToolProfilePoint(-5, 65),
				new ToolProfilePoint(8, 80), new ToolProfilePoint(8, 120),
				new ToolProfilePoint(-8, 120), new ToolProfilePoint(-20, 80),
				new ToolProfilePoint(-20, 25), new ToolProfilePoint(-shoulderY, 15)
			]));
	}
}
