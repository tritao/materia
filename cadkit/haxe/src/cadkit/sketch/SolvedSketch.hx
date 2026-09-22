package cadkit.sketch;

class SolvedSketch {
	private final coordinates:Map<String, Array<Float>>;
	private final radii:Map<String, Float>;
	public final diagnostic:SolveDiagnostic;

	public function new(coordinates:Map<String, Array<Float>>, radii:Map<String, Float>, diagnostic:SolveDiagnostic) {
		this.coordinates = coordinates; this.radii = radii; this.diagnostic = diagnostic;
	}

	public function x(id:String):Float return point(id)[0];
	public function y(id:String):Float return point(id)[1];
	public function point(id:String):Array<Float> {
		var value = coordinates.get(id);
		if (value == null) throw "unknown solved point: " + id;
		return value.copy();
	}
	public function radius(id:String):Float {
		var value = radii.get(id);
		if (value == null) throw "entity has no solved radius: " + id;
		return value;
	}
}
