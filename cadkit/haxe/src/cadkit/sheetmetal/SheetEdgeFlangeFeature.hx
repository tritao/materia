package cadkit.sheetmetal;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParametricError;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyReference;

/** Edge flanges consume persistent wall-side faces and derive their walls and bends on every rebuild. */
class SheetEdgeFlangeFeature extends SheetFeature {
	public final source:SheetFeature;
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
	final references:Array<TopologyReference>;
	var pendingFaces:Null<Array<Shape>>;
	var pendingFingerprints:Null<Array<TopologyFingerprint>>;
	var outputWallIds:Array<Int>;
	var outputBendIds:Array<Int>;
	final restoreIdentities:Bool;

	public function new(source:SheetFeature, selectedFaces:Null<Array<Shape>>, lengthValue:Float, angleValue:Float,
		?radiusValue:Float, lengthSpec:LengthSpec = Leg, position:FlangePosition = BendOutside, offsetValue:Float = 0,
		gap1Value:Float = 0, gap2Value:Float = 0, extend1Value:Float = 0, extend2Value:Float = 0,
		?savedFingerprints:Array<TopologyFingerprint>, ?savedWallIds:Array<Int>, ?savedBendIds:Array<Int>) {
		super();
		if ((selectedFaces == null) == (savedFingerprints == null))
			throw new ParametricError("flange requires either live side faces or saved face fingerprints");
		if ((selectedFaces != null && selectedFaces.length == 0) || (savedFingerprints != null && savedFingerprints.length == 0))
			throw new ParametricError("flange side-face selection must not be empty");
		var count:Int;
		if (selectedFaces != null) count = selectedFaces.length;
		else if (savedFingerprints != null) count = savedFingerprints.length;
		else throw new ParametricError("flange has no side-face selection");
		if ((savedWallIds == null) != (savedBendIds == null))
			throw new ParametricError("saved flange identities must cover every selected edge");
		if (savedWallIds != null && savedWallIds.length != count)
			throw new ParametricError("saved flange wall identities must cover every selected edge");
		if (savedBendIds != null && savedBendIds.length != count)
			throw new ParametricError("saved flange bend identities must cover every selected edge");
		restoreIdentities = savedWallIds != null;
		outputWallIds = savedWallIds == null ? [] : savedWallIds.copy();
		outputBendIds = savedBendIds == null ? [] : savedBendIds.copy();
		this.source = source;
		this.lengthSpec = lengthSpec;
		this.position = position;
		useMaterialRadius = radiusValue == null;
		angle = new Parameter(this, "sheet.angle", angleValue, -1e300, false, 1e300, ParameterKind.Angle);
		length = new Parameter(this, "sheet.length", lengthValue, 0, false, 1e300, ParameterKind.Length);
		radius = new Parameter(this, "sheet.radius", radiusValue == null ? 0 : radiusValue, -1e300, false, 1e300, ParameterKind.Length);
		offset = new Parameter(this, "sheet.offset", offsetValue, -1e300, false, 1e300, ParameterKind.Length);
		gap1 = new Parameter(this, "sheet.gap1", gap1Value, -1e300, false, 1e300, ParameterKind.Length);
		gap2 = new Parameter(this, "sheet.gap2", gap2Value, -1e300, false, 1e300, ParameterKind.Length);
		extend1 = new Parameter(this, "sheet.extend1", extend1Value, -1e300, false, 1e300, ParameterKind.Length);
		extend2 = new Parameter(this, "sheet.extend2", extend2Value, -1e300, false, 1e300, ParameterKind.Length);
		references = [];
		pendingFaces = null;
		pendingFingerprints = savedFingerprints == null ? null : savedFingerprints.copy();
		if (selectedFaces != null) {
			var owned:Array<Shape> = [];
			try {
				for (face in selectedFaces) {
					if (face == null || face.kind() != CadKit.ShapeKind.Face)
						throw new ParametricError("flange selection requires wall-side faces");
					owned.push(face.cloneShape());
				}
			} catch (error:Dynamic) { for (face in owned) face.close(); throw error; }
			pendingFaces = owned;
		}
	}
	override public function onAttached():Void {
		if (pendingFaces != null) {
			for (face in pendingFaces) references.push(TopologyReference.fromShape(this, face, source));
			pendingFaces = null;
		} else if (pendingFingerprints != null) {
			for (fingerprint in pendingFingerprints)
				references.push(TopologyReference.fromFingerprint(this, CadKit.ShapeKind.Face, fingerprint, source));
			pendingFingerprints = null;
		}
		var identities = identityLedger();
		var owner = "feature:" + id.toInt();
		if (restoreIdentities) {
			identities.claimWalls(owner, outputWallIds); identities.claimBends(owner, outputBendIds);
		} else {
			outputWallIds = identities.reserveWalls(owner, references.length);
			outputBendIds = identities.reserveBends(owner, references.length);
		}
	}
	override public function identityLedger():SheetIdentityLedger { return source.identityLedger(); }
	public function wallIds():Array<Int> { return outputWallIds.copy(); }
	public function bendIds():Array<Int> { return outputBendIds.copy(); }
	public function faceReferences():Array<TopologyReference> { return references.copy(); }
	override public function serializationType():String { return "sheet-edge-flange"; }
	override public function dependencies():Array<FeatureId> { return [source.id]; }
	override public function dependencyFeatures():Array<Feature> { return [source]; }
	override public function buildSheet(context:EvaluationContext):Sheet {
		var graph = source.sheetForEvaluation(context);
		var shape = context.shape(source);
		var selected:Array<{wall:Int, identity:String}> = [];
		var seen:Map<String, Bool> = new Map();
		for (reference in references) {
			var face = reference.resolveFor(shape);
			try {
				var edge = SheetEdgeSelection.identify(graph, face);
				var identity = graph.wall(edge.wall).outline.identity(edge.edge.loop, edge.edge.segment);
				var key = edge.wall + ":" + identity;
				if (seen.exists(key)) throw new SheetDiagnosis("sheet.flange-duplicate-edge", "Flange selects one edge more than once", [key]);
				seen.set(key, true); selected.push({wall: edge.wall, identity: identity});
				face.close();
			} catch (error:Dynamic) { face.close(); throw error; }
		}
		var bendRadius = useMaterialRadius ? graph.material.defaultRadius() : radius.value;
		for (slot in 0...selected.length) {
			var selection = selected[slot];
			var identities = graph.wall(selection.wall).outline.identities();
			var attachment:Null<WallEdge> = null;
			for (i in 0...identities.length)
				for (j in 0...identities[i].length)
					if (identities[i][j] == selection.identity) attachment = new WallEdge(j, i);
			if (attachment == null)
				throw new SheetDiagnosis("sheet.flange-edge-selection", "An earlier flange removed the selected edge", [selection.identity]);
			graph = SheetFlange.add(graph, selection.wall, attachment, length.value, angle.value, bendRadius,
				lengthSpec, position, offset.value, gap1.value, gap2.value, extend1.value, extend2.value,
				outputWallIds[slot], outputBendIds[slot]);
		}
		return graph;
	}
	override public function onClose():Void {
		if (pendingFaces != null) for (face in pendingFaces) face.close();
		pendingFaces = null; pendingFingerprints = null;
		super.onClose();
	}
}
