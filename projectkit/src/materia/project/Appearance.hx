package materia.project;

/** Portable surface finish. Physical/BOM material is a separate property. */
typedef Appearance = {
	var finish:String;
	var metallic:Float;
	var roughness:Float;
}

class Appearances {
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
	public static function neutral():Appearance return {finish: "neutral", metallic: 0.0, roughness: 0.65};
	public static function painted():Appearance return {finish: "painted", metallic: 0.0, roughness: 0.48};
	public static function machinedSteel():Appearance return {finish: "machined-steel", metallic: 0.8, roughness: 0.34};
	public static function blackOxide():Appearance return {finish: "black-oxide", metallic: 0.55, roughness: 0.52};
	public static function bearingSteel():Appearance return {finish: "bearing-steel", metallic: 0.78, roughness: 0.3};
	public static function aluminium():Appearance return {finish: "aluminium", metallic: 0.72, roughness: 0.42};
	public static function rubber():Appearance return {finish: "rubber", metallic: 0.0, roughness: 0.9};
}
