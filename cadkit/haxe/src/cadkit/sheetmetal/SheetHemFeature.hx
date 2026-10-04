package cadkit.sheetmetal;

import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.TopologyFingerprint;

/** Hem features derive the circular bend and straight return from typed authored dimensions. */
class SheetHemFeature extends SheetEdgeFeature {
	public final type:HemType;
	public final side:SheetSide;
	public final length:Parameter;
	public final gap:Null<Parameter>;
	public final radius:Null<Parameter>;
	public function new(source:SheetFeature, selectedFaces:Null<Array<Shape>>, spec:HemSpec, lengthValue:Float, side:SheetSide = A,
		?savedFingerprints:Array<TopologyFingerprint>, ?savedWallIds:Array<Int>, ?savedBendIds:Array<Int>) {
		super(source, selectedFaces, savedFingerprints, savedWallIds, savedBendIds);
		this.side = side;
		length = new Parameter(this, "sheet.hem.length", lengthValue, 0, false, 1e300, ParameterKind.Length);
		switch (spec) {
			case Closed: type = HemType.Closed; gap = null; radius = null;
			case Open(value):
				type = HemType.Open; radius = null;
				gap = new Parameter(this, "sheet.hem.gap", value, -1e300, false, 1e300, ParameterKind.Length);
			case Teardrop(value, bulb):
				type = HemType.Teardrop;
				gap = new Parameter(this, "sheet.hem.gap", value, -1e300, false, 1e300, ParameterKind.Length);
				radius = bulb == null ? null : new Parameter(this, "sheet.hem.radius", bulb, -1e300, false, 1e300, ParameterKind.Length);
		}
	}
	public function gapValue():Float { var value = gap; return value == null ? 0 : value.value; }
	public function radiusValue():Null<Float> { var value = radius; return value == null ? null : value.value; }
	public function authoredSpec():HemSpec {
		return switch (type) {
			case Closed: HemSpec.Closed;
			case Open: HemSpec.Open(gapValue());
			case Teardrop: HemSpec.Teardrop(gapValue(), radiusValue());
		};
	}
	override public function serializationType():String { return "sheet-hem"; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = source.sheetForEvaluation(context); var selected = selectedEdges(context, graph);
		var walls = wallIds(); var bends = bendIds();
		for (i in 0...selected.length) {
			var edge = selected[i];
			graph = SheetHem.add(graph, edge.wall, SheetEdgeFeature.attachment(graph, edge), type, length.value,
				gapValue(), side, radiusValue(), walls[i], bends[i]);
		}
		return graph;
	}
}
