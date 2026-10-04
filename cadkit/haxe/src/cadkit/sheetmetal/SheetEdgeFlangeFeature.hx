package cadkit.sheetmetal;

import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.TopologyFingerprint;

/** Edge flanges consume persistent wall-side faces and derive their walls and bends on every rebuild. */
class SheetEdgeFlangeFeature extends SheetEdgeFeature {
	public final angle:Parameter;
	public final length:Parameter;
	public final radius:Parameter;
	public final offset:Parameter;
	public final gap1:Parameter;
	public final gap2:Parameter;
	public final extend1:Parameter;
	public final extend2:Parameter;
	public final lengthSpec:LengthSpec;
	public final position:FlangePosition;
	public final useMaterialRadius:Bool;
	public final reliefType:ReliefType;
	public final reliefSizing:ReliefSizing;
	public final reliefWidth:Parameter;
	public final reliefDepth:Parameter;

	public function new(source:SheetFeature, selectedFaces:Null<Array<Shape>>, lengthValue:Float, angleValue:Float,
		?radiusValue:Float, lengthSpec:LengthSpec = Leg, position:FlangePosition = BendOutside, offsetValue:Float = 0,
		gap1Value:Float = 0, gap2Value:Float = 0, extend1Value:Float = 0, extend2Value:Float = 0,
		?savedFingerprints:Array<TopologyFingerprint>, ?savedWallIds:Array<Int>, ?savedBendIds:Array<Int>,
		reliefType:ReliefType = None, reliefSizing:ReliefSizing = Absolute, reliefWidthValue:Float = 0, reliefDepthValue:Float = 0) {
		super(source, selectedFaces, savedFingerprints, savedWallIds, savedBendIds);
		this.lengthSpec = lengthSpec;
		this.position = position;
		this.reliefType = reliefType;
		this.reliefSizing = reliefSizing;
		var reliefKind = reliefSizing == Absolute ? ParameterKind.Length : ParameterKind.Scalar;
		reliefWidth = new Parameter(this, "sheet.relief.width", reliefWidthValue, -1e300, false, 1e300, reliefKind);
		reliefDepth = new Parameter(this, "sheet.relief.depth", reliefDepthValue, -1e300, false, 1e300, reliefKind);
		useMaterialRadius = radiusValue == null;
		angle = new Parameter(this, "sheet.angle", angleValue, -1e300, false, 1e300, ParameterKind.Angle);
		length = new Parameter(this, "sheet.length", lengthValue, 0, false, 1e300, ParameterKind.Length);
		radius = new Parameter(this, "sheet.radius", radiusValue == null ? 0 : radiusValue, -1e300, false, 1e300, ParameterKind.Length);
		offset = new Parameter(this, "sheet.offset", offsetValue, -1e300, false, 1e300, ParameterKind.Length);
		gap1 = new Parameter(this, "sheet.gap1", gap1Value, -1e300, false, 1e300, ParameterKind.Length);
		gap2 = new Parameter(this, "sheet.gap2", gap2Value, -1e300, false, 1e300, ParameterKind.Length);
		extend1 = new Parameter(this, "sheet.extend1", extend1Value, -1e300, false, 1e300, ParameterKind.Length);
		extend2 = new Parameter(this, "sheet.extend2", extend2Value, -1e300, false, 1e300, ParameterKind.Length);
	}

	override public function serializationType():String { return "sheet-edge-flange"; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = source.sheetForEvaluation(context);
		var selected = selectedEdges(context, graph);
		var walls = wallIds(); var bends = bendIds();
		var bendRadius = useMaterialRadius ? graph.material.defaultRadius() : radius.value;
		var scale = reliefSizing == ThicknessRatio ? graph.thickness() : 1.0;
		for (slot in 0...selected.length) {
			var selection = selected[slot];
			var attachment = SheetEdgeFeature.attachment(graph, selection);
			graph = SheetFlange.add(graph, selection.wall, attachment, length.value, angle.value, bendRadius,
				lengthSpec, position, offset.value, gap1.value, gap2.value, extend1.value, extend2.value,
				walls[slot], bends[slot], reliefType, reliefWidth.value * scale, reliefDepth.value * scale);
		}
		return graph;
	}
}
