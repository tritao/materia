package cadkit.sheetmetal;

import CadKit;
import cadkit.Shape;
import cadkit.Operation;
import cadkit.modeling.Align;
import cadkit.modeling.Axis;
import cadkit.modeling.Curve;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Scope;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;

/** Owning solids derived from the graph. Native history maps authored outline segments to extrusion faces. */
class SheetSolid {
	public static function build(sheet:Sheet, state:SheetState):Shape {
		var evaluation = sheet.evaluate(state);
		var pieces:Array<Shape> = [];
		var result:Null<Shape> = null;
		try {
			for (wall in sheet.walls()) {
				var local = wallSolid(wall, sheet.thickness());
				try {
					pieces.push(evaluation.wall(wall.id).apply(local));
					local.close();
				} catch (error:Dynamic) { local.close(); throw error; }
			}
			for (bend in sheet.bends()) {
				var zone = evaluation.bend(bend.id);
				var local = bendSolid(bend, zone);
				try {
					pieces.push(zone.frame.apply(local));
					local.close();
				} catch (error:Dynamic) { local.close(); throw error; }
			}
			result = pieces[0].cloneShape();
			for (i in 1...pieces.length) {
				var combined = result.fuse(pieces[i]);
				result.close();
				result = combined;
			}
			if (CadKit.shapeValidChecked(result.borrowHandle()) == 0)
				throw new SheetDiagnosis("sheet.invalid-solid", "Fusing the evaluated wall and bend solids produced an invalid shape");
			for (piece in pieces) piece.close();
			return result;
		} catch (error:Dynamic) {
			if (result != null) result.close();
			for (piece in pieces) piece.close();
			throw error;
		}
	}

	/** One exact face; named input edges carry authored loop/segment identity through the native operation. */
	static function profile(wall:Wall, sources:Map<String, String>):Sketch {
		return Scope.run(function(scope:Scope) {
			var wires:Array<Curve> = [];
			var loops = wall.outline.loops();
			for (i in 0...loops.length) {
				var curves:Array<Curve> = [];
				for (j in 0...loops[i].length) {
					var segment = loops[i][j];
					var curve:Curve;
					if (segment.isLine())
						curve = Curve.line(vector(segment.start), vector(segment.end));
					else if (Math.abs(segment.sweep) >= 2 * Math.PI - 1e-12) {
						var center:Point2 = cast segment.center;
						curve = Curve.circle(segment.radius, new Plane(vector(center), Vector.X(), Vector.Z()));
					} else
						curve = Curve.arc(vector(segment.start), vector(segment.point(0.5)), vector(segment.end));
					var id = "wall:" + wall.id + ":edge:" + wall.outline.identity(i, j);
					var named = scope.own(Curve.named(curve, id));
					sources.set(named.shape.elementName(CadKit.ShapeKind.Edge, 0), id);
					curves.push(named);
				}
				wires.push(scope.own(Curve.wire(curves)));
			}
			return scope.own(Sketch.face(wires[0], wires.slice(1), Plane.XY()));
		});
	}
	static function vector(point:Point2):Vector { return new Vector(point.x, point.y, 0); }

	static function wallSolid(wall:Wall, thickness:Float):Shape {
		var sources:Map<String, String> = new Map();
		var sketch = profile(wall, sources);
		var part:Null<Part> = null;
		try {
			part = sketch.extrude(-thickness);
			var raw = part.shape;
			var ids = [for (i in 0...raw.subshapeCount(CadKit.ShapeKind.Face)) ""];
			for (i in 0...ids.length) {
				var face = raw.subshape(CadKit.ShapeKind.Face, i);
				try {
					if (face.surfaceKind() == CadKit.SurfaceKind.Plane) {
						var normal = Vector.fromNative(face.faceNormal());
						var center = Vector.fromNative(face.center());
						if (Math.abs(normal.z) > 1 - 1e-9) {
							if (Math.abs(center.z) < 1e-7) ids[i] = "wall:" + wall.id + ":a";
							else if (Math.abs(center.z + thickness) < 1e-7) ids[i] = "wall:" + wall.id + ":b";
						}
					}
					face.close();
				} catch (error:Dynamic) { face.close(); throw error; }
			}
			var op:Operation = cast part.operation;
			for (i in 0...op.historyCount(CadKit.HistoryRelation.Generated)) {
				var source = op.historySourceAt(CadKit.HistoryRelation.Generated, i);
				var target = op.historyTargetAt(CadKit.HistoryRelation.Generated, i);
				try {
					if (source.kind() == CadKit.ShapeKind.Edge && target.kind() == CadKit.ShapeKind.Face) {
						var id = sourceId(source, sketch.shape, sources);
						if (id != null)
							assignTarget(raw, target, ids, id);
					}
					source.close(); target.close();
				} catch (error:Dynamic) { source.close(); target.close(); throw error; }
			}
			var result = seeded(raw, ids, "wall:" + wall.id);
			sketch.close();
			part.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			if (part != null) part.close();
			throw error;
		}
	}
	/** History accessors return bare topology; match it to the still-owned named input by IsSame. */
	static function sourceId(source:Shape, profile:Shape, sources:Map<String, String>):Null<String> {
		for (i in 0...profile.subshapeCount(CadKit.ShapeKind.Edge)) {
			var candidate = profile.subshape(CadKit.ShapeKind.Edge, i);
			try {
				var matches = candidate.sameAs(source);
				var id = matches ? sources.get(candidate.elementName(CadKit.ShapeKind.Edge, 0)) : null;
				candidate.close();
				if (matches) return id;
			} catch (error:Dynamic) { candidate.close(); throw error; }
		}
		return null;
	}
	static function assignTarget(raw:Shape, target:Shape, ids:Array<String>, id:String):Void {
		for (i in 0...ids.length) {
			var candidate = raw.subshape(CadKit.ShapeKind.Face, i);
			var matches:Bool;
			try {
				matches = candidate.sameAs(target);
				candidate.close();
			} catch (error:Dynamic) { candidate.close(); throw error; }
			if (matches) {
				if (ids[i] != "" && ids[i] != id)
					throw new SheetDiagnosis("sheet.solid-naming", "Two authored segments generated the same face", [ids[i], id]);
				ids[i] = id;
				return;
			}
		}
		throw new SheetDiagnosis("sheet.solid-naming", "A generated face is missing from the extrusion", [id]);
	}

	static function bendSolid(bend:Bend, zone:BendZone):Shape {
		if (zone.allowance <= 0)
			throw new SheetDiagnosis("sheet.zero-bend-allowance", "The rule gives zero neutral length for a nonzero bend", ["bend:" + bend.id]);
		var sketch:Sketch;
		if (zone.angle == 0)
			sketch = Sketch.rectangle(zone.allowance, zone.width, Plane.XY(), Align.Min, Align.Min);
		else
			sketch = Sketch.polygon([new Vector(0, -zone.thickness), new Vector(zone.width, -zone.thickness),
				new Vector(zone.width, 0), new Vector(0, 0)], Plane.YZ());
		var part:Null<Part> = null;
		try {
			if (zone.angle == 0) part = sketch.extrude(-zone.thickness);
			else {
				var axis = zone.frame.plane.toLocal(zone.axis());
				part = sketch.revolve(new Axis(axis, Vector.Y()), -zone.angle);
			}
			sketch.close();
			var raw = part.shape;
			var ids:Array<String> = [];
			for (i in 0...raw.subshapeCount(CadKit.ShapeKind.Face)) {
				var face = raw.subshape(CadKit.ShapeKind.Face, i);
				try {
					var role = bendRole(face, bend, zone);
					ids.push("bend:" + bend.id + ":" + role);
					face.close();
				} catch (error:Dynamic) { face.close(); throw error; }
			}
			var result = seeded(raw, ids, "bend:" + bend.id);
			part.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			if (part != null) part.close();
			throw error;
		}
	}
	static function bendRole(face:Shape, bend:Bend, zone:BendZone):String {
		if (face.surfaceKind() == CadKit.SurfaceKind.Cylinder && zone.angle != 0) {
			var radius = face.faceAxis().get_radius();
			var inner:Float = cast zone.innerRadius;
			if (Math.abs(radius - inner) < 1e-7) return "inner";
			if (Math.abs(radius - inner - zone.thickness) < 1e-7) return "outer";
		} else if (face.surfaceKind() == CadKit.SurfaceKind.Plane) {
			var center = Vector.fromNative(face.center());
			if (Math.abs(center.y) < 1e-7) return "end:0";
			if (Math.abs(center.y - zone.width) < 1e-7) return "end:1";
			if (zone.angle == 0) {
				if (Math.abs(center.z) < 1e-7) return bend.angle > 0 ? "inner" : "outer";
				if (Math.abs(center.z + zone.thickness) < 1e-7) return bend.angle > 0 ? "outer" : "inner";
			}
			var start = new Vector(0, zone.width / 2, -zone.thickness / 2);
			var end = zone.frame.plane.toLocal(zone.point(1, zone.width / 2, zone.thickness / 2));
			if (center.subtract(start).length() < 1e-7) return "join:a";
			if (center.subtract(end).length() < 1e-7) return "join:b";
		}
		throw new SheetDiagnosis("sheet.solid-naming", "Bend face does not match an analytic strip surface", ["bend:" + bend.id]);
	}
	static function seeded(raw:Shape, ids:Array<String>, prefix:String):Shape {
		for (id in ids)
			if (id == "") throw new SheetDiagnosis("sheet.solid-naming", "Native history left a face unnamed", [prefix]);
		var faces = raw.withElementNames(CadKit.ShapeKind.Face, ids);
		try {
			var result = faces.withElementNames(CadKit.ShapeKind.Solid, [prefix + ":solid"]);
			faces.close();
			return result;
		} catch (error:Dynamic) { faces.close(); throw error; }
	}
}
