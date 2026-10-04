package cadkit.sheetmetal;

import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;

/** One sheet graph owns identities and dimensions; solids and flat patterns are derived from it. */
class Sheet {
	public final material:SheetMaterial;
	public final placement:Location;
	final wallData:Array<Wall>;
	final bendData:Array<Bend>;
	final ripData:Array<Rip>;
	var nextWall:Int = 1;
	var nextBend:Int = 1;

	public function new(material:SheetMaterial, ?placement:Location) {
		this.material = material;
		this.placement = placement == null ? Location.identity() : placement;
		wallData = [];
		bendData = [];
		ripData = [];
	}
	public function thickness():Float {
		return material.thickness;
	}
	/** Isolated editable graph, sharing only immutable material, outlines and connection records. */
	public function copy():Sheet {
		var result = new Sheet(material, placement);
		for (wall in wallData) result.wallData.push(wall);
		for (bend in bendData) result.bendData.push(bend);
		for (rip in ripData) result.ripData.push(rip);
		result.nextWall = nextWall;
		result.nextBend = nextBend;
		return result;
	}
	public function walls():Array<Wall> { return wallData.copy(); }
	public function bends():Array<Bend> { return bendData.copy(); }
	public function rips():Array<Rip> { return ripData.copy(); }
	public function addWall(outline:WallOutline):Wall {
		var result = new Wall(nextWall++, outline);
		wallData.push(result);
		return result;
	}
	public function wall(id:Int):Wall {
		for (w in wallData)
			if (w.id == id)
				return w;
		throw new SheetDiagnosis("sheet.missing-wall", "Wall does not belong to this sheet", ["wall:" + id]);
	}
	public function bend(id:Int):Bend {
		for (b in bendData)
			if (b.id == id)
				return b;
		throw new SheetDiagnosis("sheet.missing-bend", "Bend does not belong to this sheet", ["bend:" + id]);
	}
	public function addBend(wallA:Int, edgeA:WallEdge, wallB:Int, edgeB:WallEdge, angle:Float, radius:Float):Bend {
		var result = new Bend(nextBend, wallA, edgeA, wallB, edgeB, angle, radius);
		validateBend(result);
		for (b in bendData)
			if (uses(b, wallA, edgeA) || uses(b, wallB, edgeB))
				throw new SheetDiagnosis("sheet.bend-edge-in-use", "A boundary segment can belong to only one bend", ["bend:" + b.id]);
		nextBend++;
		bendData.push(result);
		return result;
	}
	static function uses(b:Bend, wall:Int, edge:WallEdge):Bool {
		return (b.wallA == wall && b.edgeA.key() == edge.key()) || (b.wallB == wall && b.edgeB.key() == edge.key());
	}
	function validateBend(b:Bend):Void {
		BendDimensions.validate(thickness(), b.radius, b.angle);
		if (b.wallA == b.wallB || b.angle == 0)
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "A bend needs different walls and a nonzero target angle", ["bend:" + b.id]);
		var a = wall(b.wallA).outline.segment(b.edgeA.loop, b.edgeA.segment);
		var c = wall(b.wallB).outline.segment(b.edgeB.loop, b.edgeB.segment);
		if (!a.isLine() || !c.isLine() || Math.abs(a.length() - c.length()) > 1e-8)
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "Bend edges must be straight and of equal length", ["bend:" + b.id]);
	}
	public function updateWall(id:Int, outline:WallOutline):Wall {
		var previous = wall(id);
		var result = new Wall(id, outline);
		var index = wallData.indexOf(previous);
		wallData[index] = result;
		try {
			for (b in bendData)
				if (b.wallA == id || b.wallB == id)
					validateBend(b);
		} catch (error:Dynamic) {
			wallData[index] = previous;
			throw error;
		}
		return result;
	}
	public function updateBend(id:Int, angle:Float, radius:Float):Bend {
		var previous = bend(id);
		var result = new Bend(id, previous.wallA, previous.edgeA, previous.wallB, previous.edgeB, angle, radius);
		validateBend(result);
		bendData[bendData.indexOf(previous)] = result;
		return result;
	}
	public function rip(id:Int):Rip {
		var b = bend(id);
		if (isRipped(b.id))
			throw new SheetDiagnosis("sheet.already-ripped", "Bend is already cut", ["bend:" + id]);
		var result = new Rip(id);
		ripData.push(result);
		return result;
	}
	public function isRipped(id:Int):Bool {
		for (r in ripData)
			if (r.bend == id)
				return true;
		return false;
	}
	public function cutHole(id:Int, loop:Array<Segment>):Wall {
		return updateWall(id, wall(id).outline.withHole(loop));
	}
	public function bendAllowance(b:Bend):Float {
		var current = bend(b.id);
		return BendDimensions.allowance(thickness(), current.radius, current.angle, material.bendRule);
	}
	public function flat():SheetState {
		var result = new SheetState();
		for (b in bendData)
			result.set(b.id, 0);
		return result;
	}
	public function folded():SheetState {
		var result = new SheetState();
		for (b in bendData)
			result.set(b.id, b.angle);
		return result;
	}

	/** Outward X, edge tangent Y, side-A normal Z. Child attachment uses the opposite tangent. */
	public static function edgeFrame(outline:WallOutline, edge:WallEdge, child:Bool = false):Location {
		var s = outline.segment(edge.loop, edge.segment);
		if (!s.isLine())
			throw new SheetDiagnosis("sheet.bend-edge-mismatch", "An arc cannot be a bend edge");
		var tangent = new Vector(s.end.x - s.start.x, s.end.y - s.start.y, 0).normalized();
		var p = child ? s.end : s.start;
		var outward = tangent.cross(Vector.Z());
		return new Location(new Plane(new Vector(p.x, p.y, 0), child ? outward.scale(-1) : outward, Vector.Z()));
	}
	function localZone(b:Bend, angle:Float):BendZone {
		var allowance = bendAllowance(b);
		var neutralOffset = allowance / Math.abs(b.angle) - b.radius;
		var edge = wall(b.wallA).outline.segment(b.edgeA.loop, b.edgeA.segment);
		return new BendZone(b.id, edgeFrame(wall(b.wallA).outline, b.edgeA), angle, allowance, edge.length(), thickness(), neutralOffset);
	}
	function relative(b:Bend, angle:Float):Location {
		return localZone(b, angle).endFrame().compose(edgeFrame(wall(b.wallB).outline, b.edgeB, true).inverse());
	}

	public function evaluate(state:SheetState, root:Int = 0, tolerance:Float = 1e-8):SheetEvaluation {
		if (!Math.isFinite(tolerance) || tolerance <= 0)
			throw new SheetDiagnosis("sheet.invalid-tolerance", "Closure tolerance must be finite and positive");
		if (material.bendRule == null)
			throw new SheetDiagnosis("sheet.no-bend-rule", "Choose a bend rule before evaluating a sheet");
		if (wallData.length == 0)
			throw new SheetDiagnosis("sheet.empty", "A sheet needs a root wall");
		if (root == 0)
			root = wallData[0].id;
		var rootWall = wall(root);
		for (id in state.ids()) { var known = bend(id); }
		for (b in bendData) {
			validateBend(b);
			var a = state.angle(b.id);
			if (a * b.angle < 0 || Math.abs(a) > Math.abs(b.angle))
				throw new SheetDiagnosis("sheet.invalid-state", "State angle must lie between zero and its target", ["bend:" + b.id]);
		}
		var frames:Map<Int, Location> = new Map();
		var tree:Map<Int, Bool> = new Map();
		frames.set(root, placement);
		var queue = [root];
		var cursor = 0;
		while (cursor < queue.length) {
			var id = queue[cursor++];
			var parent:Location = cast frames.get(id);
			for (b in bendData) {
				if (isRipped(b.id) || (b.wallA != id && b.wallB != id))
					continue;
				var other = b.wallA == id ? b.wallB : b.wallA;
				if (frames.exists(other))
					continue;
				var step = relative(b, state.angle(b.id));
				frames.set(other, parent.compose(b.wallA == id ? step : step.inverse()));
				tree.set(b.id, true);
				queue.push(other);
			}
		}
		for (w in wallData)
			if (!frames.exists(w.id))
				throw new SheetDiagnosis("sheet.disconnected", "Wall is not connected to the root after rips", ["wall:" + w.id]);
		var zones:Map<Int, BendZone> = new Map();
		for (b in bendData) {
			var frameA:Location = cast frames.get(b.wallA);
			var actual:Location = cast frames.get(b.wallB);
			var expected = frameA.compose(relative(b, state.angle(b.id)));
			if (!isRipped(b.id) && !tree.exists(b.id)) {
				var gap = expected.plane.origin.subtract(actual.plane.origin).length();
				var directionGap = Math.max(expected.plane.xDirection.subtract(actual.plane.xDirection).length(),
					expected.plane.normal.subtract(actual.plane.normal).length());
				if (gap > tolerance || directionGap > tolerance)
					throw new SheetDiagnosis("sheet.closure-gap", "Closure gap " + gap + " mm; frame direction gap " + directionGap,
						["bend:" + b.id, "wall:" + b.wallA, "wall:" + b.wallB]);
			}
			var local = localZone(b, state.angle(b.id));
			zones.set(b.id, new BendZone(b.id, frameA.compose(local.frame), local.angle, local.allowance, local.width,
				thickness(), local.allowance / Math.abs(b.angle) - b.radius));
		}
		return new SheetEvaluation(frames, zones, wallData, thickness());
	}
}
