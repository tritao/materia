import CadKit;
import cadkit.Edge;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.Feature;
import cadkit.parametric.GeometricConnectors;
import cadkit.parametric.GeometricConnectors.GeometricConnectorError;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/** One reference after one edit: what it resolved to, judged by the case's oracle. */
typedef NamingOutcome = {
	var row:String;
	var outcome:String;
	var detail:String;
	var seconds:Float;
}

/**
	Topological naming TN0 (plans/TOPOLOGICAL_NAMING.md): models × edits × references. Each row builds its model
	afresh, captures a reference, applies one edit, and classes the result against a geometric oracle that says
	which element the reference should be now:
	- `correct`: it resolved to that element;
	- `reported`: it did not resolve and said so (Ambiguous, Unresolved, Deleted);
	- `wrong`: it resolved to a different element (or resolved where only a report is right).
	`EXPECTED` pins each row's outcome; later stages may only raise rows to `correct`, and no row may be `wrong`
	unless it is listed there as a known baseline failure.
*/
class NamingRobustnessSmoke {
	static inline var CORRECT = "correct";
	static inline var REPORTED = "reported";
	static inline var WRONG = "wrong";
	static inline var TOLERANCE = 1e-6;

	/** The outcome of every row: the TN0 baseline, raised as stages land (TN3: references by name). */
	static var EXPECTED:Map<String, String> = [
		"box fillet top/+x | control" => CORRECT,
		"box chamfer top/+y | control" => CORRECT,
		"box fillet top/+x | width" => CORRECT,
		"box chamfer top/+y | width" => CORRECT,
		"box fillet top/+x | depth" => CORRECT,
		"box chamfer top/+y | depth" => CORRECT,
		"box fillet top/+x | height" => CORRECT,
		"box chamfer top/+y | height" => CORRECT,
		"plate sketch on top face | control" => CORRECT,
		"plate sketch on top face | thickness" => CORRECT,
		"plate sketch on top face | width" => CORRECT,
		"plate sketch on top face | hole inside" => CORRECT,
		"plate sketch on top face | slot across" => REPORTED,
		"revolve fillet outer top rim | control" => CORRECT,
		"revolve fillet outer top rim | width" => CORRECT,
		"revolve fillet outer top rim | height" => CORRECT,
		"pattern fillet boss 1 rim | control" => CORRECT,
		"pattern fillet boss 1 rim | spacing" => CORRECT,
		"pattern fillet boss 1 rim | count" => CORRECT,
		"part connector on bore B | control" => CORRECT,
		"part connector on bore B | widen" => CORRECT,
		"part connector on bore B | move bore" => CORRECT,
		"sketch-made side face | control" => CORRECT,
		"sketch-made side face | move line" => CORRECT,
		"sketch-made side face | redraw line" => CORRECT,
		"sketch-made side face | split line" => REPORTED,
		"legacy v10 box fillet | load" => CORRECT
	];

	static var outcomes:Array<NamingOutcome> = [];

	public static function run():Void {
		outcomes = [];
		for (edit in ["control", "width", "depth", "height"]) {
			boxCase(edit, false);
			boxCase(edit, true);
		}
		for (edit in ["control", "thickness", "width", "hole inside", "slot across"])
			plateCase(edit);
		for (edit in ["control", "width", "height"])
			revolveCase(edit);
		for (edit in ["control", "spacing", "count"])
			patternCase(edit);
		for (edit in ["control", "widen", "move bore"])
			connectorCase(edit);
		for (edit in ["control", "move line", "redraw line", "split line"])
			sketchCase(edit);
		legacyCase();
		report();
	}

	// 1. Box with a fillet on the top/+x edge and a chamfer on the top/+y edge.
	static function boxCase(edit:String, chamfer:Bool):Void {
		var row = 'box ${chamfer ? "chamfer top/+y" : "fillet top/+x"} | $edit';
		var document = new Document();
		var box = document.add(new BoxFeature(10, 20, 30));
		document.recompute();
		var shape = box.currentShape();
		var edgeIndex = chamfer ? find(shape, CadKit.ShapeKind.Edge, s -> lineAt(s, 5, 20, 30, 1, 0, 0))
			: find(shape, CadKit.ShapeKind.Edge, s -> lineAt(s, 10, 10, 30, 0, 1, 0));
		var edge = new Edge(shape.subshape(CadKit.ShapeKind.Edge, edgeIndex));
		var finish:Feature = chamfer ? document.add(new ChamferFeature(box, 1, [edge])) : document.add(new FilletFeature(box, 1, [edge]));
		edge.close();
		document.recompute();
		switch edit {
			case "width": box.width.set(14);
			case "depth": box.depth.set(26);
			case "height": box.height.set(24);
			case _:
		}
		var reference = chamfer ? (cast finish : ChamferFeature).edgeReferences[0] : (cast finish : FilletFeature).edgeReferences[0];
		var w = box.width.value, d = box.depth.value, h = box.height.value;
		judgeReference(row, document, reference,
			chamfer ? s -> lineAt(s, w / 2, d, h, 1, 0, 0) : s -> lineAt(s, w, d / 2, h, 0, 1, 0));
		document.close();
	}

	// 2–3. A 60 × 40 × 10 plate cut by a hole tool and a slot tool that start outside it; a sketch on its top face.
	static function plateCase(edit:String):Void {
		var row = 'plate sketch on top face | $edit';
		var document = new Document();
		var plate = document.add(new BoxFeature(60, 40, 10));
		var hole = document.add(new TransformFeature(document.add(new CylinderFeature(3, 30)), -50, 20, -5));
		var slot = document.add(new TransformFeature(document.add(new BoxFeature(4, 60, 30)), -50, -10, -5));
		var drilled = document.add(new BooleanFeature(plate, hole, BooleanOperation.Cut));
		var body = document.add(new BooleanFeature(drilled, slot, BooleanOperation.Cut));
		document.recompute();
		var top = body.currentShape().subshape(CadKit.ShapeKind.Face,
			find(body.currentShape(), CadKit.ShapeKind.Face, s -> planeAt(s, 10, 1)));
		var sketch = document.add(new ConstrainedSketchFeature(rectangle(8, 6), body, null, Vector.X(), 0, false, top));
		top.close();
		document.recompute();
		var splits = false;
		switch edit {
			case "thickness": plate.height.set(14);
			case "width": plate.width.set(80);
			case "hole inside": hole.x.set(15);
			case "slot across":
				slot.x.set(28);
				splits = true;
			case _:
		}
		var height = plate.height.value;
		// A face split in two has no single answer: only a report is right.
		judgeReference(row, document, sketch.supportFaceReference, s -> !splits && planeAt(s, height, 1));
		document.close();
	}

	// 4. A rectangle revolved into a ring (inner radius 10, outer 20, z ±10); fillet on the outer top rim.
	static function revolveCase(edit:String):Void {
		var row = 'revolve fillet outer top rim | $edit';
		var document = new Document();
		var plane = new Plane(new Vector(15, 0, 0), Vector.X(), new Vector(0, -1, 0));
		var profile = document.add(new SketchFeature("rectangle", 10, 20, plane));
		var ring = document.add(new RevolveFeature(profile, 0, 0, 0, 0, 0, 1, Math.PI * 2));
		document.recompute();
		var shape = ring.currentShape();
		var edge = new Edge(shape.subshape(CadKit.ShapeKind.Edge, find(shape, CadKit.ShapeKind.Edge, s -> circleAt(s, 10, 20))));
		var fillet = document.add(new FilletFeature(ring, 1, [edge]));
		edge.close();
		document.recompute();
		switch edit {
			case "width": profile.width.set(14);
			case "height": profile.height.set(26);
			case _:
		}
		var outer = 15 + profile.width.value / 2, top = profile.height.value / 2;
		judgeReference(row, document, fillet.edgeReferences[0], s -> circleAt(s, top, outer));
		document.close();
	}

	// 5. Three radius-3 bosses patterned 15 mm apart along x on an 80 × 30 × 10 plate; fillet on instance 1's rim.
	static function patternCase(edit:String):Void {
		var row = 'pattern fillet boss 1 rim | $edit';
		var document = new Document();
		var plate = document.add(new BoxFeature(80, 30, 10));
		var boss = document.add(new TransformFeature(document.add(new CylinderFeature(3, 5)), 40, 15, 10));
		var pattern = document.add(new LinearPatternFeature(boss, 3, 15, Vector.X()));
		var body = document.add(new BooleanFeature(plate, pattern, BooleanOperation.Fuse));
		document.recompute();
		var shape = body.currentShape();
		var edge = new Edge(shape.subshape(CadKit.ShapeKind.Edge,
			find(shape, CadKit.ShapeKind.Edge, s -> circleCenteredAt(s, 40, 15, 15, 3))));
		var fillet = document.add(new FilletFeature(body, 0.5, [edge]));
		edge.close();
		document.recompute();
		switch edit {
			case "spacing": pattern.spacing.set(18);
			case "count": pattern.count.set(4);
			case _:
		}
		var count = pattern.count.value, spacing = pattern.spacing.value;
		var x = 40 + (1 - (count - 1) / 2) * spacing;
		judgeReference(row, document, fillet.edgeReferences[0], s -> circleCenteredAt(s, x, 15, 15, 3));
		document.close();
	}

	// 6. A part built with the modeling layer: a plate with two radius-4 bores; a connector on the second bore.
	static function connectorCase(edit:String):Void {
		var row = 'part connector on bore B | $edit';
		var started = Sys.time();
		var original = boredPlate(60, 0.75);
		var bore = find(original, CadKit.ShapeKind.Face, s -> s.surfaceKind() == CadKit.SurfaceKind.Cylinder && Math.abs(s.center().get_x() - 45) < 0.5);
		var connector = GeometricConnectors.capture("bore", original, CadKit.ShapeKind.Face, bore);
		original.close();
		var width = edit == "widen" ? 90.0 : 60.0;
		var fraction = edit == "move bore" ? 0.6 : 0.75;
		var edited = boredPlate(width, fraction);
		var outcome = WRONG, detail = "";
		try {
			var frame = GeometricConnectors.frame(edited, connector);
			var ok = Math.abs(frame.x - width * fraction) < TOLERANCE && Math.abs(frame.y - 20) < TOLERANCE;
			outcome = ok ? CORRECT : WRONG;
			detail = 'framed at (${frame.x}, ${frame.y})';
		} catch (error:GeometricConnectorError) {
			outcome = REPORTED;
			detail = error.code;
		}
		edited.close();
		record(row, outcome, detail, Sys.time() - started);
	}

	static function boredPlate(width:Float, fraction:Float):Shape {
		// Built as a recipe would be, naming its bodies (TN4).
		var plate = namedPart(Part.box(width, 40, 10, Min, Min, Min), "plate");
		var first = namedPart(Part.cylinderSpan(4, -1, 11, width * 0.25, 20), "bore.a");
		var second = namedPart(Part.cylinderSpan(4, -1, 11, width * fraction, 20), "bore.b");
		var bored = plate.subtractAll([first, second]);
		var result = bored.shape.cloneShape();
		bored.close();
		plate.close();
		first.close();
		second.close();
		return result;
	}

	static function namedPart(part:Part, tag:String):Part {
		var result = part.named(tag);
		part.close();
		return result;
	}

	// 7. A constrained-sketch rectangle extruded 5 mm; a sketch on the side face made by the line at x = +10.
	static function sketchCase(edit:String):Void {
		var row = 'sketch-made side face | $edit';
		var document = new Document();
		var profile = document.add(new ConstrainedSketchFeature(rectangle(20, 10)));
		var block = document.add(ExtrudeFeature.along(profile, 5, Vector.Z()));
		document.recompute();
		var side = block.currentShape().subshape(CadKit.ShapeKind.Face,
			find(block.currentShape(), CadKit.ShapeKind.Face, s -> planeNormalX(s, 10)));
		var sketch = document.add(new ConstrainedSketchFeature(rectangle(2, 2), block, null, Vector.Y(), 0, false, side));
		side.close();
		document.recompute();
		var x = 10.0, splits = false;
		switch edit {
			case "move line":
				x = 12;
				for (index in 1...3) {
					var y = index == 1 ? -5 : 5;
					profile.replacePoint(new SketchPoint("rectangle.point" + index, x, y));
				}
			case "redraw line":
				profile.removeEntity("rectangle.edge1");
				profile.addEntity(SketchEntity.line("rectangle.redrawn", "rectangle.point1", "rectangle.point2"));
			case "split line":
				profile.removeEntity("rectangle.edge1");
				profile.addPoint(new SketchPoint("rectangle.middle", 10, 0));
				profile.addConstraint(SketchConstraint.fixed("rectangle.fixedMiddle", "rectangle.middle"));
				profile.addEntity(SketchEntity.line("rectangle.lower", "rectangle.point1", "rectangle.middle"));
				profile.addEntity(SketchEntity.line("rectangle.upper", "rectangle.middle", "rectangle.point2"));
				splits = true;
			case _:
		}
		judgeReference(row, document, sketch.supportFaceReference, s -> !splits && planeNormalX(s, x));
		document.close();
	}

	// 8. A document saved before names (version 10, fingerprint-only references) loads and resolves.
	static function legacyCase():Void {
		var row = 'legacy v10 box fillet | load';
		var document = DocumentCodec.decode(NamingLegacyFixtures.BOX_FILLET_V10);
		var fillet:FilletFeature = cast document.featureAt(1);
		judgeReference(row, document, fillet.edgeReferences[0], s -> lineAt(s, 10, 10, 30, 0, 1, 0));
		// Resolved by geometry, the reference takes its element's name, and a save writes it (version 11).
		var name = fillet.edgeReferences[0].fingerprintData().name;
		var saved = DocumentCodec.encode(document);
		if (name != "E(f1:box.+x|f1:box.+z)" || saved.indexOf('"name":"E(f1:box.+x|f1:box.+z)"') < 0 || saved.indexOf('"version":11') < 0)
			throw 'NamingRobustnessSmoke: a legacy reference was not upgraded to its name (got $name)';
		document.close();
	}

	/** Recompute after the edit and class what `reference` resolved to. */
	static function judgeReference(row:String, document:Document, reference:TopologyReference, oracle:Shape->Bool):Void {
		var started = Sys.time();
		var failure = "";
		try document.recompute() catch (error:Dynamic) failure = Std.string(error);
		var seconds = Sys.time() - started;
		if (reference.isResolved()) {
			var current = reference.currentShape();
			if (failure != "")
				record(row, WRONG, "resolved but recompute failed: " + failure, seconds);
			else if (!oracle(current))
				record(row, WRONG, Std.string(reference.state), seconds);
			else {
				// The oracle must single out one element, or "correct" would mean nothing.
				var producer = reference.remapTargetFeature().currentShape();
				var matches = 0;
				for (index in 0...producer.subshapeCount(reference.kind)) {
					var candidate = producer.subshape(reference.kind, index);
					if (oracle(candidate))
						matches++;
					candidate.close();
				}
				if (matches != 1)
					throw 'NamingRobustnessSmoke: the oracle of "$row" matches $matches elements';
				record(row, CORRECT, Std.string(reference.state), seconds);
			}
			return;
		}
		record(row, REPORTED, Std.string(reference.state), seconds);
	}

	static function record(row:String, outcome:String, detail:String, seconds:Float):Void
		outcomes.push({row: row, outcome: outcome, detail: detail, seconds: seconds});

	static function report():Void {
		var counts:Map<String, Int> = [CORRECT => 0, REPORTED => 0, WRONG => 0];
		var failures:Array<String> = [];
		var total = 0.0;
		for (outcome in outcomes) {
			Sys.println('naming  ${pad(outcome.outcome, 9)} ${pad(Std.string(Math.round(outcome.seconds * 1000)), 5)} ms  ${outcome.row}  (${outcome.detail})');
			var seen:Null<Int> = counts.get(outcome.outcome);
			var previous:Int = seen == null ? 0 : seen;
			counts.set(outcome.outcome, previous + 1);
			total += outcome.seconds;
			var expected = EXPECTED.get(outcome.row);
			if (expected == null)
				failures.push('${outcome.row}: no expected outcome (got ${outcome.outcome})');
			else if (expected != outcome.outcome)
				failures.push('${outcome.row}: expected $expected, got ${outcome.outcome}');
		}
		Sys.println('naming  ${counts.get(CORRECT)} correct, ${counts.get(REPORTED)} reported, ${counts.get(WRONG)} wrong; '
			+ '${Math.round(total * 1000)} ms in edited recomputes');
		if (failures.length > 0)
			throw "NamingRobustnessSmoke:\n  " + failures.join("\n  ");
	}

	static function pad(text:String, width:Int):String {
		var result = text;
		while (result.length < width)
			result += " ";
		return result;
	}

	/** The one subshape of `kind` that `matches`; throws when none or several do (the cases are set up to be unique). */
	static function find(shape:Shape, kind:CadKit.ShapeKind, matches:Shape->Bool):Int {
		var found = -1;
		for (index in 0...shape.subshapeCount(kind)) {
			var candidate = shape.subshape(kind, index);
			var hit = matches(candidate);
			candidate.close();
			if (!hit)
				continue;
			if (found >= 0)
				throw "NamingRobustnessSmoke: several subshapes match a setup predicate";
			found = index;
		}
		if (found < 0)
			throw "NamingRobustnessSmoke: no subshape matches a setup predicate";
		return found;
	}

	static function near(a:Float, b:Float):Bool
		return Math.abs(a - b) < TOLERANCE;

	/** A straight edge with its midpoint at (x, y, z), along ±(dx, dy, dz). */
	static function lineAt(shape:Shape, x:Float, y:Float, z:Float, dx:Float, dy:Float, dz:Float):Bool {
		if (shape.kind() != CadKit.ShapeKind.Edge || shape.curveKind() != CadKit.CurveKind.Line)
			return false;
		var middle = shape.positionAt(0.5), tangent = shape.tangentAt();
		return near(middle.get_x(), x) && near(middle.get_y(), y) && near(middle.get_z(), z)
			&& near(Math.abs(tangent.get_x() * dx + tangent.get_y() * dy + tangent.get_z() * dz), 1);
	}

	/** A circular edge about the z axis at height z with the given radius. */
	static function circleAt(shape:Shape, z:Float, radius:Float):Bool
		return circleCenteredAt(shape, 0, 0, z, radius);

	static function circleCenteredAt(shape:Shape, x:Float, y:Float, z:Float, radius:Float):Bool {
		if (shape.kind() != CadKit.ShapeKind.Edge || shape.curveKind() != CadKit.CurveKind.Circle)
			return false;
		var axis = shape.edgeAxis();
		var origin = axis.get_origin();
		return near(origin.get_x(), x) && near(origin.get_y(), y) && near(origin.get_z(), z) && near(axis.get_radius(), radius);
	}

	/** A planar face at height z whose normal is ±z (sign: 1 or -1). */
	static function planeAt(shape:Shape, z:Float, sign:Int):Bool {
		if (shape.kind() != CadKit.ShapeKind.Face || shape.surfaceKind() != CadKit.SurfaceKind.Plane)
			return false;
		return near(shape.center().get_z(), z) && near(shape.faceNormal().get_z(), sign);
	}

	/** A planar face at x with its normal +x. */
	static function planeNormalX(shape:Shape, x:Float):Bool {
		if (shape.kind() != CadKit.ShapeKind.Face || shape.surfaceKind() != CadKit.SurfaceKind.Plane)
			return false;
		return near(shape.center().get_x(), x) && near(shape.faceNormal().get_x(), 1);
	}

	/** A centred width × height rectangle of fixed points; its lines are `rectangle.edge0..3`, edge1 at x = +width/2. */
	static function rectangle(width:Float, height:Float):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var coordinates = [[-width / 2, -height / 2], [width / 2, -height / 2], [width / 2, height / 2], [-width / 2, height / 2]];
		for (index in 0...4) {
			sketch.addPoint(new SketchPoint("rectangle.point" + index, coordinates[index][0], coordinates[index][1]));
			sketch.addConstraint(SketchConstraint.fixed("rectangle.fixed" + index, "rectangle.point" + index));
		}
		for (index in 0...4)
			sketch.addEntity(SketchEntity.line("rectangle.edge" + index, "rectangle.point" + index, "rectangle.point" + ((index + 1) % 4)));
		return sketch;
	}
}
