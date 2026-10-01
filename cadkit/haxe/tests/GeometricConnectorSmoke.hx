import cadkit.Geometry;
import cadkit.Shape;
import cadkit.modeling.AssemblyMateSolver;
import cadkit.modeling.AssemblyState;
import cadkit.parametric.AssemblyDocuments;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionEvaluator;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.GeometricConnectors;
import cadkit.parametric.GeometricConnectors.GeometricConnectorError;
import cadkit.parametric.GeometricConnectors.GeometricFeatureKind;
import cadkit.parametric.ParameterKind;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A 40 mm deep, 10 mm thick plate of the given width with a radius-5 bore through it, 20 mm from its near edge. */
private class PlateEvaluator implements DefinitionEvaluator {
	public function new() {}

	public function evaluate(definition:Definition, instance:cadkit.parametric.InstanceElement, output:String):Shape
		return GeometricConnectorSmoke.plate(instance.resolved("width"));
}

/** A radius-5 pin, 30 mm long, standing on the origin along z. */
private class PinEvaluator implements DefinitionEvaluator {
	public function new() {}

	public function evaluate(definition:Definition, instance:cadkit.parametric.InstanceElement, output:String):Shape
		return Shape.cylinder(5, 30);
}

/**
	Mates on faces instead of connectors (plan C4.4): a pin seated on a
	plate's top face and in its bore, through geometric connectors that a
	document keeps as topology fingerprints and frames again from the current
	geometry, so the pin follows the bore when the plate is edited.
*/
class GeometricConnectorSmoke {
	public static function run():Void {
		checkEdgeAnchors();
		checkFrames();
		checkCompatibility();
		DefinitionEvaluatorRegistry.register("cadkit.test.mate-plate", new PlateEvaluator());
		DefinitionEvaluatorRegistry.register("cadkit.test.mate-pin", new PinEvaluator());
		var document = new Document();
		var plateDefinition = document.createDefinition("Plate", "cadkit.test.mate-plate",
			[new DefinitionInput("width", ParameterKind.Length, "mm", 60)], [new DefinitionOutput("body", DefinitionOutput.Geometry)]);
		var pinDefinition = document.createDefinition("Pin", "cadkit.test.mate-pin", [],
			[new DefinitionOutput("body", DefinitionOutput.Geometry)]);

		// In memory: connectors captured on the shapes are ordinary connectors with a reference; the mates place the pin.
		var plateShape = plate(60), pinShape = Shape.cylinder(5, 30);
		var authored = pinOnPlate(plateShape, pinShape);
		var base = GeometricConnectors.reference(authored.definitions[1].connectors[0]);
		check(base != null && base.flip && base.feature == GeometricFeatureKind.Plane, "a connector's reference keeps its feature and flip");
		expectSeated(authored, 30, "in memory");

		// In a document: the records keep the references and last frames; `reframe` frames them from the evaluated geometry.
		var root = AssemblyDocuments.fromDefinition(document, authored, (path, occurrence, component) ->
			occurrence.definition == "plate" ? plateDefinition : pinDefinition);
		expectSeated(AssemblyDocuments.toDefinition(root), 30, "from the document's last frames");
		expectSeated(AssemblyDocuments.reframe(root), 30, "framed from the document");
		var reloaded = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		expectSeated(AssemblyDocuments.reframe(reloaded.element(root.id)), 30, "after a save and reload");
		reloaded.close();

		// Widening the plate moves its bore: the records' frames are the old ones until reframed; then the pin follows.
		plateDefinition.setDefault("width", 90);
		expectSeated(AssemblyDocuments.toDefinition(root), 30, "before reframing");
		expectSeated(AssemblyDocuments.reframe(root), 45, "after the plate is widened");

		// Two plates of different widths cannot share one bore connector.
		var twoPlates = AssemblyDocuments.reframe(root);
		twoPlates.occurrences.push({id: "spare", definition: "plate", initialPose: AssemblyFrames.translation(0, 200, 0)});
		twoPlates.id = "two-plates";
		var spareRoot = AssemblyDocuments.fromDefinition(document, twoPlates, (path, occurrence, component) ->
			occurrence.definition == "plate" ? plateDefinition : pinDefinition);
		for (element in document.allElements()) {
			var id = element.property("cadkit.assembly.id");
			if (id != null && Std.string(id.value) == "spare") (cast element : cadkit.parametric.InstanceElement).setOverride("width", 70);
		}
		check(diagnosticCode(() -> AssemblyDocuments.reframe(spareRoot)) == "assembly.instance-dependent",
			"occurrences that place a connector differently are reported");
		expectSeated(AssemblyDocuments.reframe(root), 45, "the first assembly is unaffected");

		// A connector on geometry the part does not have is reported, not guessed.
		var ball = Shape.sphere(8);
		var withBall = pinOnPlate(plateShape, pinShape);
		withBall.definitions[1].connectors.push(GeometricConnectors.connector(
			GeometricConnectors.capture("ball", ball, CadKit.ShapeKind.Face, 0), ball));
		var code = "";
		try GeometricConnectors.reframe(withBall, (scope, component) -> component == "plate" ? [plateShape] : [pinShape])
		catch (error:GeometricConnectorError) code = error.code;
		check(code == "unresolved", 'a connector whose face is gone is reported: "$code"');

		// A mate that asks a face for what it does not define is refused when authored.
		var pointOnPlane = pinOnPlate(plateShape, pinShape);
		pointOnPlane.mates = [{id: "touch", kind: AssemblyMateKind.Coincident, first: "plate", firstConnector: "top", second: "pin",
			secondConnector: "base", axis: {x: 0, y: 0, z: 1}}];
		check(diagnosticCode(() -> AssemblyDocuments.fromDefinition(new Document(), pointOnPlane)) == "assembly.incompatible-mate",
			"a coincident mate on a planar face is refused");

		ball.close();
		plateShape.close();
		pinShape.close();
		document.close();
	}

	static function diagnosticCode(action:()->Dynamic):String {
		try action() catch (error:cadkit.parametric.AssemblyDocuments.AssemblyDocumentDiagnostic) return error.code;
		return "";
	}

	/** What each feature defines decides the mates it takes. */
	static function checkCompatibility():Void {
		check(GeometricConnectors.compatible(GeometricFeatureKind.Plane, AssemblyMateKind.Planar) &&
			!GeometricConnectors.compatible(GeometricFeatureKind.Plane, AssemblyMateKind.Coaxial) &&
			!GeometricConnectors.compatible(GeometricFeatureKind.Plane, AssemblyMateKind.Distance), "a plane takes planar mates, not point or axis ones");
		check(GeometricConnectors.compatible(GeometricFeatureKind.Axis, AssemblyMateKind.Coaxial) &&
			!GeometricConnectors.compatible(GeometricFeatureKind.Axis, AssemblyMateKind.Planar), "an axis takes coaxial mates");
		check(GeometricConnectors.compatible(GeometricFeatureKind.Sphere, AssemblyMateKind.Coincident) &&
			!GeometricConnectors.compatible(GeometricFeatureKind.Sphere, AssemblyMateKind.Parallel), "a sphere has a center, no direction");
		check(GeometricConnectors.compatible(GeometricFeatureKind.Circle, AssemblyMateKind.Lock) &&
			!GeometricConnectors.compatible(GeometricFeatureKind.Line, AssemblyMateKind.Lock), "only a circle has a whole frame");
	}

	/** The pin's base is on the plate's top (z = 10) and its axis in the bore at (x, 20); it may still turn. */
	static function expectSeated(definition:AssemblyDefinition, x:Float, label:String):Void {
		var result = AssemblyMateSolver.solve(definition);
		check(result.converged && result.report.degreesOfFreedom == 1, '$label: the pin is seated with one turn free (${result.status}, ${result.report.degreesOfFreedom})');
		var state = new AssemblyState(definition, result.state(definition));
		var base = state.worldConnector("pin", "base"), side = state.worldConnector("pin", "side");
		near(base.z, 10, '$label: the pin stands on the plate');
		near(side.x, x, '$label: the pin is in the bore (x)');
		near(side.y, 20, '$label: the pin is in the bore (y)');
		var up = AssemblyFrames.transformVector(side, 0, 0, 1);
		near(Math.abs(up.z), 1, '$label: the pin stands upright');
	}

	/**
		Edge fingerprints are anchored at the edge's midpoint (document version 10); records from older
		documents, anchored at the first vertex, still resolve to the same edge.
	*/
	static function checkEdgeAnchors():Void {
		var box = Shape.box(30, 20, 10);
		for (index in [0, 5, 11]) {
			var edge = box.subshape(CadKit.ShapeKind.Edge, index);
			var fingerprint = cadkit.parametric.TopologyFingerprint.capture(edge);
			var middle = edge.positionAt(0.5), start = edge.subshape(CadKit.ShapeKind.Vertex, 0), first = start.position();
			start.close();
			edge.close();
			check(fingerprint.midpoint && Math.abs(fingerprint.x - middle.get_x()) < 1e-9, 'edge $index is anchored at its midpoint');
			var saved = DocumentCodec.decodeFingerprint(haxe.Json.parse(haxe.Json.stringify(DocumentCodec.encodeFingerprint(fingerprint))),
				CadKit.ShapeKind.Edge);
			check(saved.midpoint, 'edge $index keeps its anchor through a save');
			var legacyRecord:Dynamic = haxe.Json.parse(haxe.Json.stringify(DocumentCodec.encodeFingerprint(fingerprint)));
			Reflect.deleteField(legacyRecord, "anchor");
			Reflect.setField(legacyRecord, "x", first.get_x());
			Reflect.setField(legacyRecord, "y", first.get_y());
			Reflect.setField(legacyRecord, "z", first.get_z());
			var legacy = DocumentCodec.decodeFingerprint(legacyRecord, CadKit.ShapeKind.Edge);
			check(!legacy.midpoint, 'a record without an anchor is a first-vertex one');
			for (candidate in [saved, legacy]) {
				var resolution = cadkit.parametric.TopologyResolver.resolve(box, candidate, CadKit.ShapeKind.Edge);
				check(resolution.index == index, 'edge $index resolves (${candidate.midpoint ? "midpoint" : "first vertex"}): ${resolution.index}');
			}
		}
		box.close();
	}

	/** Each kind of face and edge offers the frame `GeometricConnectors.frameOf` promises. */
	static function checkFrames():Void {
		var pin = Shape.cylinder(5, 30);
		var side = pin.subshape(CadKit.ShapeKind.Face, face(pin, CadKit.SurfaceKind.Cylinder, 0));
		var sideFrame = GeometricConnectors.frameOf(side);
		expectFrame(sideFrame, 0, 0, 15, "a cylindrical face is framed on its axis, level with its middle");
		var x = AssemblyFrames.transformVector(sideFrame, 1, 0, 0), reference = side.faceAxis().get_reference();
		near(x.x * reference.get_x() + x.y * reference.get_y() + x.z * reference.get_z(), 1, "its x is the surface's own reference direction");
		side.close();
		var top = pin.subshape(CadKit.ShapeKind.Face, face(pin, CadKit.SurfaceKind.Plane, 1));
		var topFrame = GeometricConnectors.frameOf(top);
		expectFrame(topFrame, 0, 0, 30, "a planar face is framed at its centroid");
		near(AssemblyFrames.transformVector(topFrame, 0, 0, 1).z, 1, "along its outward normal");
		near(AssemblyFrames.transformVector(GeometricConnectors.frameOf(top, true), 0, 0, 1).z, -1, "or against it when flipped");
		top.close();
		var circles = 0;
		for (index in 0...pin.subshapeCount(CadKit.ShapeKind.Edge)) {
			var edge = pin.subshape(CadKit.ShapeKind.Edge, index);
			if (edge.curveKind() == CadKit.CurveKind.Circle) {
				var frame = GeometricConnectors.frameOf(edge);
				check(Math.abs(frame.z) < 1e-9 || Math.abs(frame.z - 30) < 1e-9, 'a circular edge is framed at its center: ${frame.z}');
				expectFrame(frame, 0, 0, frame.z, "a circular edge is framed on the axis");
				circles++;
			} else {
				var frame = GeometricConnectors.frameOf(edge);
				near(frame.z, 15, "a straight edge is framed at its middle");
				near(Math.abs(AssemblyFrames.transformVector(frame, 0, 0, 1).z), 1, "along its direction");
			}
			edge.close();
		}
		check(circles == 2, 'the pin has two circular edges: $circles');
		pin.close();

		// After an edit moves the bore, it is found again as the one face of its kind, axis and radius, or refused if not one.
		var narrow = plate(60), wide = plate(90);
		var bore = GeometricConnectors.capture("bore", narrow, CadKit.ShapeKind.Face, face(narrow, CadKit.SurfaceKind.Cylinder, 0));
		near(GeometricConnectors.frame(wide, bore).x, 45, "a moved bore is found by its axis and radius");
		var tool = Shape.cylinder(5, 12), placed = tool.translate(Geometry.vec3(10, 20, -1)), twoBores = wide.cut(placed);
		var state = "";
		try GeometricConnectors.frame(twoBores, bore) catch (error:cadkit.parametric.GeometricConnectors.GeometricConnectorError) state = error.code;
		check(state == "ambiguous", 'two bores like it make it ambiguous: "$state"');
		for (shape in [narrow, wide, tool, placed, twoBores]) shape.close();
	}

	public static function plate(width:Float):Shape {
		var block = Shape.box(width, 40, 10);
		var tool = Shape.cylinder(5, 12), placed = tool.translate(Geometry.vec3(width / 2, 20, -1));
		var result = block.cut(placed);
		block.close();
		tool.close();
		placed.close();
		return result;
	}

	/** The index of the face of `surface` kind, for planes the one whose normal's z has `normalZ`'s sign. */
	static function face(shape:Shape, surface:CadKit.SurfaceKind, normalZ:Int):Int {
		for (index in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var candidate = shape.subshape(CadKit.ShapeKind.Face, index);
			var matches = candidate.surfaceKind() == surface &&
				(normalZ == 0 || candidate.faceNormal().get_z() * normalZ > 0.5);
			candidate.close();
			if (matches) return index;
		}
		throw 'No $surface face';
	}

	static function expectFrame(frame:AssemblyFrame, x:Float, y:Float, z:Float, label:String):Void {
		near(frame.x, x, label + " (x)");
		near(frame.y, y, label + " (y)");
		near(frame.z, z, label + " (z)");
	}

	/** The plate (grounded) and a pin placed far off, with the plate's top and bore and the pin's base and side as connectors. */
	static function pinOnPlate(plateShape:Shape, pinShape:Shape):AssemblyDefinition {
		var top = GeometricConnectors.capture("top", plateShape, CadKit.ShapeKind.Face, face(plateShape, CadKit.SurfaceKind.Plane, 1));
		var bore = GeometricConnectors.capture("bore", plateShape, CadKit.ShapeKind.Face, face(plateShape, CadKit.SurfaceKind.Cylinder, 0));
		var base = GeometricConnectors.capture("base", pinShape, CadKit.ShapeKind.Face, face(pinShape, CadKit.SurfaceKind.Plane, -1), true);
		var side = GeometricConnectors.capture("side", pinShape, CadKit.ShapeKind.Face, face(pinShape, CadKit.SurfaceKind.Cylinder, 0));
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "pin-on-plate", lengthUnit: "mm",
			definitions: [{id: "plate", connectors: [GeometricConnectors.connector(top, plateShape), GeometricConnectors.connector(bore, plateShape)]},
				{id: "pin", connectors: [GeometricConnectors.connector(base, pinShape), GeometricConnectors.connector(side, pinShape)]}],
			occurrences: [{id: "plate", definition: "plate", initialPose: AssemblyFrames.identity(), grounded: true},
				{id: "pin", definition: "pin", initialPose: {x: 120, y: -40, z: 75, qx: 0.2, qy: 0, qz: 0, qw: Math.sqrt(0.96)}}],
			joints: [],
			mates: [{id: "seat", kind: AssemblyMateKind.Planar, first: "plate", firstConnector: "top", second: "pin", secondConnector: "base",
				axis: {x: 0, y: 0, z: 1}},
				{id: "shaft", kind: AssemblyMateKind.Coaxial, first: "plate", firstConnector: "bore", second: "pin", secondConnector: "side",
					axis: {x: 0, y: 0, z: 1}}]};
	}

	/** Within the mate solver's default position tolerance, 1 µm. */
	static function near(actual:Float, expected:Float, label:String):Void {
		if (!(Math.abs(actual - expected) <= 1e-3)) throw '$label: expected $expected, got $actual';
	}

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
