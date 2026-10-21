package materia.project;

/** Portable surface finish. Physical/BOM material is a separate property. */
typedef Appearance = {
	var finish:String;
	var metallic:Float;
	var roughness:Float;
}

class Appearances {
	public static function same(a:Null<Appearance>, b:Null<Appearance>):Bool {
		var left = a == null ? neutral() : a;
		var right = b == null ? neutral() : b;
		return left.finish == right.finish && Math.abs(left.metallic - right.metallic) < 1e-6
			&& Math.abs(left.roughness - right.roughness) < 1e-6;
	}
	public static function presets():Array<Appearance> return [neutral(), painted(), machinedSteel(),
		blackOxide(), bearingSteel(), aluminium(), rubber()];
	public static function preset(id:String):Null<Appearance> return switch (id) {
		case "neutral": neutral();
		case "painted": painted();
		case "machined-steel": machinedSteel();
		case "black-oxide": blackOxide();
		case "bearing-steel": bearingSteel();
		case "aluminium": aluminium();
		case "rubber": rubber();
		default: null;
	}
	// Finishes are the material library's visuals, so a finish and its material never disagree.
	public static function neutral():Appearance return MaterialLibrary.appearance("neutral");
	public static function painted():Appearance return MaterialLibrary.appearance("painted");
	public static function machinedSteel():Appearance return MaterialLibrary.appearance("machined-steel");
	public static function blackOxide():Appearance return MaterialLibrary.appearance("black-oxide");
	public static function bearingSteel():Appearance return MaterialLibrary.appearance("bearing-steel");
	public static function aluminium():Appearance return MaterialLibrary.appearance("aluminium");
	public static function rubber():Appearance return MaterialLibrary.appearance("rubber");
}
