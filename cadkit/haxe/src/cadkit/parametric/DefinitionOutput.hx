package cadkit.parametric;

class DefinitionOutput {
	public static inline var Geometry:String = "geometry";
	public static inline var Tool:String = "tool";
	public static inline var Connector:String = "connector";

	public final name:String;
	public final purpose:String;

	public function new(name:String, purpose:String) {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("definition output name must not be empty");
		switch purpose {
			case "geometry", "tool", "connector":
			default:
				throw new ParametricError("unsupported definition output purpose: " + purpose);
		}
		this.name = name;
		this.purpose = purpose;
	}
}
