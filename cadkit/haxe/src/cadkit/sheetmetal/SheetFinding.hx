package cadkit.sheetmetal;

/** One named finding keeps the measured rule and its exact geometry together. */
class SheetFinding {
	public final kind:SheetCheckKind;
	public final code:String;
	public final message:String;
	public final assumed:Bool;
	public final measure:SheetCheckMeasure;
	final geometryData:Array<SheetCheckGeometry>;
	public function new(kind:SheetCheckKind, message:String, assumed:Bool, measure:SheetCheckMeasure, geometry:Array<SheetCheckGeometry>) {
		this.kind = kind; this.message = message; this.assumed = assumed; this.measure = measure; geometryData = geometry.copy();
		code = switch (kind) {
			case FlatOverlap: "sheet.flat-overlap";
			case ShortFlange: "sheet.short-flange";
			case CutNearBend: "sheet.cut-near-bend";
			case MissingRelief: "sheet.missing-relief";
			case SmallRadius: "sheet.radius-below-minimum";
			case AlongGrain: "sheet.bend-along-grain";
		};
	}
	public function geometry():Array<SheetCheckGeometry> { return geometryData.copy(); }
}
