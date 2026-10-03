package machinekit.welding;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.welding.WeldSeam;

/** A member's geometry for seam finding: its occurrence id and its part, in the frame the seams are to be found in. Borrowed. */
typedef SeamMember = {
	var id:String;
	var part:Part;
}

/** Seams that join end to end, in welding order and each turned to run on from the one before. */
typedef SeamChain = {
	var seams:Array<WeldSeam>;
	/** The last seam ends where the first begins. */
	var closed:Bool;
}

/** One edge of a member's B-rep, for seam finding. */
class SeamEdge {
	public final p0:Vector;
	public final p1:Vector;
	public final line:Bool;
	/** The two faces that meet along the edge, as indices into the member's faces. */
	public final faces:Array<Int> = [];

	public function new(p0:Vector, p1:Vector, line:Bool) {
		this.p0 = p0;
		this.p1 = p1;
		this.line = line;
	}
}

/** One face of a member's B-rep, for seam finding. */
class SeamSurface {
	public final name:String;
	public final planar:Bool;
	/** The outward normal and a point on the face; only planar faces take part in seams. */
	public final normal:Vector;
	public final centre:Vector;
	/** The edge indices of each boundary loop, and which loop is the outer one. */
	public final loops:Array<Array<Int>> = [];
	public var outer:Int = 0;
	/** Every boundary edge as a chord, all loops together: curved edges are sampled. */
	public final chords:Array<Array<Vector>> = [];
	/** Chords of the outer loop only. */
	public final outline:Array<Vector> = [];

	public function new(name:String, planar:Bool, normal:Vector, centre:Vector) {
		this.name = name;
		this.planar = planar;
		this.normal = normal;
		this.centre = centre;
	}
}

/** A member's faces and edges with their adjacency, read once from its part. */
class SeamSolid {
	public final member:String;
	public final faces:Array<SeamSurface> = [];
	public final edges:Array<SeamEdge> = [];

	/** Samples per curved edge when it is read as chords. */
	static inline var CURVE_SAMPLES:Int = 16;

	function new(member:String) {
		this.member = member;
	}

	public static function of(member:SeamMember):SeamSolid {
		var result = new SeamSolid(member.id);
		var shape = member.part.shape;
		var edgeShapes:Array<Shape> = [];
		var chordsOf:Array<Array<Vector>> = [];
		var faceShapes:Array<Shape> = [];
		try {
			for (i in 0...shape.subshapeCount(CadKit.ShapeKind.Edge)) {
				var e = shape.subshape(CadKit.ShapeKind.Edge, i);
				edgeShapes.push(e);
				var p0 = Vector.fromNative(e.positionAt(0)), p1 = Vector.fromNative(e.positionAt(1));
				var line = e.curveKind() == CadKit.CurveKind.Line;
				result.edges.push(new SeamEdge(p0, p1, line));
				chordsOf.push(line ? [p0, p1] : [for (k in 0...CURVE_SAMPLES + 1) Vector.fromNative(e.positionAt(k / CURVE_SAMPLES))]);
			}
			var names = shape.elementNames(CadKit.ShapeKind.Face);
			for (j in 0...shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var f = shape.subshape(CadKit.ShapeKind.Face, j);
				faceShapes.push(f);
				var planar = f.surfaceKind() == CadKit.SurfaceKind.Plane;
				var face = new SeamSurface(names[j], planar, planar ? Vector.fromNative(f.faceNormal()) : new Vector(),
					planar ? Vector.fromNative(f.center()) : new Vector());
				for (w in 0...f.subshapeCount(CadKit.ShapeKind.Wire)) {
					var wire = f.subshape(CadKit.ShapeKind.Wire, w);
					var loop:Array<Int> = [];
					for (k in 0...wire.subshapeCount(CadKit.ShapeKind.Edge)) {
						var member = wire.subshape(CadKit.ShapeKind.Edge, k);
						for (index in 0...edgeShapes.length) if (edgeShapes[index].sameAs(member)) {
							loop.push(index);
							break;
						}
						member.close();
					}
					wire.close();
					face.loops.push(loop);
				}
				// The outer loop is the one that spans the most.
				var widest = -1.0;
				for (index in 0...face.loops.length) {
					var span = loopSpan(result.edges, face.loops[index]);
					if (span > widest) {
						widest = span;
						face.outer = index;
					}
				}
				// Chords, outline and edge-to-face adjacency.
				for (index in 0...face.loops.length) for (edge in face.loops[index]) {
					var chain = chordsOf[edge];
					for (k in 0...chain.length - 1) {
						face.chords.push([chain[k], chain[k + 1]]);
						if (index == face.outer) face.outline.push(chain[k]);
						if (index == face.outer && k == chain.length - 2) face.outline.push(chain[k + 1]);
					}
					if (result.edges[edge].faces.indexOf(j) < 0) result.edges[edge].faces.push(j);
				}
				result.faces.push(face);
			}
		} catch (error:Dynamic) {
			closeAll(edgeShapes);
			closeAll(faceShapes);
			throw error;
		}
		closeAll(edgeShapes);
		closeAll(faceShapes);
		return result;
	}

	static function closeAll(shapes:Array<Shape>):Void {
		for (shape in shapes) shape.close();
	}

	static function loopSpan(edges:Array<SeamEdge>, loop:Array<Int>):Float {
		if (loop.length == 0) return 0;
		var lowX = 1e300, lowY = 1e300, lowZ = 1e300, highX = -1e300, highY = -1e300, highZ = -1e300;
		for (index in loop) for (p in [edges[index].p0, edges[index].p1]) {
			lowX = Math.min(lowX, p.x);
			lowY = Math.min(lowY, p.y);
			lowZ = Math.min(lowZ, p.z);
			highX = Math.max(highX, p.x);
			highY = Math.max(highY, p.y);
			highZ = Math.max(highZ, p.z);
		}
		return Math.sqrt((highX - lowX) * (highX - lowX) + (highY - lowY) * (highY - lowY) + (highZ - lowZ) * (highZ - lowZ));
	}
}

/** A seam before it is named and given its process settings. */
typedef SeamCandidate = {
	var a:SeamFace;
	var b:SeamFace;
	var normalA:Vector;
	var normalB:Vector;
	var p0:Vector;
	var p1:Vector;
	var joint:JointType;
}

/**
 * Finds weld seams where two members meet, from their geometry alone.
 *
 * Two kinds of contact are looked for, both between planar faces:
 * - **Edge on face (fillet, lap).** An edge of one member lies on a face of the other, and the member's face
 *   that holds that edge lies flat against it (opposite normals). That face is the contact; the member's
 *   other face on the edge is the side face. The seam is the stretch of that edge where the other member's
 *   face continues past it, on the side the side face looks toward: the open side of the corner. Where the
 *   face stops (a flush end, or the edge of a plate), there is no corner and so no seam. Only edges on the
 *   contact face's outer boundary count, so the inside of a hollow section is left alone. The joint is a
 *   fillet when the contact is narrower than the side face is tall (a T-joint) and a lap when it is wider.
 * - **Edge to edge (butt, corner).** An edge of each member coincides, and the faces on either side that
 *   meet flat against each other are the contact. The seam is the groove the two other faces make.
 *
 * Each seam runs between one face of each member, named by the two faces' topological names. A tube
 * section's perimeter is one seam per side, since each side lies between a different pair of faces;
 * `chains` joins the sides that run on from one another.
 */
class WeldSeams {
	/** Two surfaces this close are touching, in millimetres: a bigger gap is a gap, not a seam. */
	public static inline var TOUCH:Float = 0.01;
	/** How far beside an edge to look for the other face, in millimetres. */
	static inline var PROBE:Float = 0.02;
	/** Tolerance on |cos| for parallel and opposite directions. */
	static inline var ANGLE:Float = 1e-6;

	/**
	 * The seams between members `a` and `b`, in the frame their parts are in. Each carries the leg size, passes and
	 * angles given. A pair that doesn't touch has none, so the result is empty.
	 */
	public static function find(a:SeamMember, b:SeamMember, legSize:Float = 0, passes:Int = 1, travelAngle:Float = WeldSeam.PUSH_ANGLE,
			workAngle:Float = 0):Array<WeldSeam> {
		var solidA = SeamSolid.of(a), solidB = SeamSolid.of(b);
		var found:Array<SeamCandidate> = [];
		onFaces(solidA, solidB, true, found);
		onFaces(solidB, solidA, false, found);
		coincident(solidA, solidB, found);
		return named(unique(found), legSize, passes, travelAngle, workAngle);
	}

	/** Edges of `x` that lie on faces of `y`: fillet and lap seams. `xFirst` says whether `x` is the pair's first member. */
	static function onFaces(x:SeamSolid, y:SeamSolid, xFirst:Bool, out:Array<SeamCandidate>):Void {
		for (index in 0...x.edges.length) {
			var edge = x.edges[index];
			var length = edge.p1.subtract(edge.p0).length();
			if (!edge.line || edge.faces.length != 2 || !(length > 1e-6)) continue;
			var direction = edge.p1.subtract(edge.p0).scale(1 / length);
			for (plane in y.faces) {
				if (!plane.planar || Math.abs(plane.normal.dot(edge.p0.subtract(plane.centre))) > TOUCH
					|| Math.abs(plane.normal.dot(edge.p1.subtract(plane.centre))) > TOUCH) continue;
				for (k in 0...2) {
					var contact = x.faces[edge.faces[k]], side = x.faces[edge.faces[1 - k]];
					if (!contact.planar || !side.planar || contact.normal.dot(plane.normal) > -1 + ANGLE) continue;
					if (contact.loops.length == 0 || contact.loops[contact.outer].indexOf(index) < 0) continue;
					// Where the corner is open: along the side face's normal, within the other member's face.
					var toward = inPlane(side.normal, plane.normal);
					if (toward == null) continue;
					for (span in spans(plane, edge.p0, direction, length, toward)) {
						var p0 = edge.p0.add(direction.scale(span.lo)), p1 = edge.p0.add(direction.scale(span.hi));
						var width = extent(contact.outline, edge.p0, toward.scale(-1));
						var height = extent(side.outline, edge.p0, plane.normal);
						out.push(candidate(x, y, xFirst, side, plane, p0, p1, width > height ? JointType.Lap : JointType.Fillet));
					}
				}
			}
		}
	}

	/** Edges of `a` that coincide with edges of `b`, the faces flat against each other: butt and corner seams. */
	static function coincident(a:SeamSolid, b:SeamSolid, out:Array<SeamCandidate>):Void {
		for (ia in 0...a.edges.length) {
			var ea = a.edges[ia];
			if (!ea.line || ea.faces.length != 2) continue;
			for (ib in 0...b.edges.length) {
				var eb = b.edges[ib];
				if (!eb.line || eb.faces.length != 2) continue;
				var same = ea.p0.subtract(eb.p0).length() <= TOUCH && ea.p1.subtract(eb.p1).length() <= TOUCH
					|| ea.p0.subtract(eb.p1).length() <= TOUCH && ea.p1.subtract(eb.p0).length() <= TOUCH;
				if (!same) continue;
				for (ka in 0...2) for (kb in 0...2) {
					var contactA = a.faces[ea.faces[ka]], sideA = a.faces[ea.faces[1 - ka]];
					var contactB = b.faces[eb.faces[kb]], sideB = b.faces[eb.faces[1 - kb]];
					if (!contactA.planar || !contactB.planar || !sideA.planar || !sideB.planar
						|| contactA.normal.dot(contactB.normal) > -1 + ANGLE) continue;
					if (contactA.loops[contactA.outer].indexOf(ia) < 0 || contactB.loops[contactB.outer].indexOf(ib) < 0) continue;
					// The contact faces must overlap beside the shared edge, not just meet along it.
					var into = inPlane(sideA.normal, contactA.normal);
					if (into == null) continue;
					var middle = ea.p0.add(ea.p1).scale(0.5);
					if (!inside(contactB, middle.subtract(into.scale(PROBE)))) continue;
					if (!(sideA.normal.add(sideB.normal).length() > 1e-6)) continue;
					var joint = sideA.normal.dot(sideB.normal) > 1 - ANGLE ? JointType.Butt : JointType.Corner;
					out.push({a: {member: a.member, face: sideA.name}, b: {member: b.member, face: sideB.name}, normalA: sideA.normal,
						normalB: sideB.normal, p0: ea.p0, p1: ea.p1, joint: joint});
				}
			}
		}
	}

	static function candidate(x:SeamSolid, y:SeamSolid, xFirst:Bool, side:SeamSurface, plane:SeamSurface, p0:Vector, p1:Vector,
			joint:JointType):SeamCandidate {
		var own:SeamFace = {member: x.member, face: side.name}, other:SeamFace = {member: y.member, face: plane.name};
		return xFirst ? {a: own, b: other, normalA: side.normal, normalB: plane.normal, p0: p0, p1: p1, joint: joint}
			: {a: other, b: own, normalA: plane.normal, normalB: side.normal, p0: p0, p1: p1, joint: joint};
	}

	/** `v` projected into the plane with unit normal `normal`, as a unit vector, or null when it is square to the plane. */
	static function inPlane(v:Vector, normal:Vector):Null<Vector> {
		var projected = v.subtract(normal.scale(v.dot(normal)));
		return projected.length() < 1e-3 ? null : projected.normalized();
	}

	/** How far `points` reach from `origin` along `direction`. */
	static function extent(points:Array<Vector>, origin:Vector, direction:Vector):Float {
		var reach = 0.0;
		for (point in points) reach = Math.max(reach, point.subtract(origin).dot(direction));
		return reach;
	}

	/**
	 * The stretches of the segment `origin + t * direction`, `t` in 0...`length`, where the face `plane` continues past it
	 * toward `toward`. Cut where the face's boundary crosses or runs along the segment, and each piece tested at its middle.
	 */
	static function spans(plane:SeamSurface, origin:Vector, direction:Vector, length:Float, toward:Vector):Array<{lo:Float, hi:Float}> {
		var breaks:Array<Float> = [0, length];
		var u = perpendicular(plane.normal), v = plane.normal.cross(u);
		function flat(p:Vector):{x:Float, y:Float} return {x: p.dot(u), y: p.dot(v)};
		var o = flat(origin), r = {x: direction.dot(u) * length, y: direction.dot(v) * length};
		for (chord in plane.chords) {
			var q0 = flat(chord[0]), q1 = flat(chord[1]);
			var s = {x: q1.x - q0.x, y: q1.y - q0.y}, w = {x: q0.x - o.x, y: q0.y - o.y};
			var denominator = r.x * s.y - r.y * s.x;
			var scale = length * Math.sqrt(s.x * s.x + s.y * s.y);
			if (Math.abs(denominator) > 1e-9 * Math.max(1, scale)) {
				var t = (w.x * s.y - w.y * s.x) / denominator, k = (w.x * r.y - w.y * r.x) / denominator;
				if (k >= -1e-9 && k <= 1 + 1e-9 && t >= 0 && t <= 1) breaks.push(t * length);
			} else if (Math.abs(w.x * r.y - w.y * r.x) <= TOUCH * length) {
				// Along the same line: the chord's ends cut the segment.
				for (p in [chord[0], chord[1]]) {
					var t = p.subtract(origin).dot(direction);
					if (t > 0 && t < length) breaks.push(t);
				}
			}
		}
		breaks.sort((a, b) -> a < b ? -1 : (a > b ? 1 : 0));
		var result:Array<{lo:Float, hi:Float}> = [];
		for (i in 0...breaks.length - 1) {
			var lo = breaks[i], hi = breaks[i + 1];
			if (hi - lo < 1e-4) continue;
			var middle = origin.add(direction.scale((lo + hi) / 2)).add(toward.scale(PROBE));
			if (!inside(plane, middle)) continue;
			if (result.length > 0 && lo - result[result.length - 1].hi < 1e-4) result[result.length - 1] = {lo: result[result.length - 1].lo, hi: hi};
			else result.push({lo: lo, hi: hi});
		}
		return result;
	}

	static function perpendicular(normal:Vector):Vector {
		var seed = Math.abs(normal.x) < 0.9 ? Vector.X() : Vector.Y();
		return seed.subtract(normal.scale(seed.dot(normal))).normalized();
	}

	/** Whether point `p`, in the plane of `face`, lies within it (even-odd over all its loops, so holes are outside). */
	static function inside(face:SeamSurface, p:Vector):Bool {
		var u = perpendicular(face.normal), v = face.normal.cross(u);
		var px = p.dot(u), py = p.dot(v);
		var within = false;
		for (chord in face.chords) {
			var x0 = chord[0].dot(u), y0 = chord[0].dot(v), x1 = chord[1].dot(u), y1 = chord[1].dot(v);
			if ((y0 > py) != (y1 > py) && x0 + (py - y0) * (x1 - x0) / (y1 - y0) > px) within = !within;
		}
		return within;
	}

	/** Drops a seam found twice, from either member's side. */
	static function unique(found:Array<SeamCandidate>):Array<SeamCandidate> {
		var result:Array<SeamCandidate> = [];
		for (c in found) {
			var duplicate = false;
			for (other in result)
				if (key(c) == key(other) && c.p0.add(c.p1).subtract(other.p0.add(other.p1)).length() <= 2 * TOUCH
					&& Math.abs(c.p1.subtract(c.p0).length() - other.p1.subtract(other.p0).length()) <= TOUCH) duplicate = true;
			if (!duplicate) result.push(c);
		}
		return result;
	}

	static function key(c:SeamCandidate):String {
		var first = '${c.a.member}:${c.a.face}', second = '${c.b.member}:${c.b.face}';
		return Reflect.compare(first, second) < 0 ? '$first|$second' : '$second|$first';
	}

	/** Builds the seams: faces ordered by name, the direction of travel from the faces, pieces numbered along their pair. */
	static function named(found:Array<SeamCandidate>, legSize:Float, passes:Int, travelAngle:Float, workAngle:Float):Array<WeldSeam> {
		var oriented:Array<SeamCandidate> = [];
		for (c in found) {
			var first = '${c.a.member}:${c.a.face}', second = '${c.b.member}:${c.b.face}';
			oriented.push(Reflect.compare(first, second) <= 0 ? c : {a: c.b, b: c.a, normalA: c.normalB, normalB: c.normalA, p0: c.p0, p1: c.p1, joint: c.joint});
		}
		var result:Array<WeldSeam> = [];
		var counts:Map<String, Int> = new Map();
		for (c in oriented) {
			var known = counts.get(key(c));
			counts.set(key(c), known == null ? 1 : known + 1);
		}
		var seen:Map<String, Int> = new Map();
		// Several seams between the same faces are numbered by where they lie along the seam's direction.
		oriented.sort(compareAlong);
		for (c in oriented) {
			var travel = c.normalA.cross(c.normalB);
			var along = c.p1.subtract(c.p0).normalized();
			// Faces that meet at an angle fix the direction; flat ones (a butt) go the positive way along the edge.
			var forward = travel.length() > 1e-6 ? travel.dot(along) >= 0 : firstPositive(along) >= 0;
			var piece = 0;
			var total = counts.get(key(c));
			if (total != null && total > 1) {
				var known = seen.get(key(c));
				piece = known == null ? 1 : known + 1;
				seen.set(key(c), piece);
			}
			result.push(new WeldSeam(c.a, c.b, c.joint, forward ? c.p0 : c.p1, forward ? c.p1 : c.p0, c.normalA, c.normalB, legSize,
				passes, travelAngle, workAngle, piece));
		}
		return result;
	}

	static function compareAlong(x:SeamCandidate, y:SeamCandidate):Int {
		var m = x.p0.add(x.p1), n = y.p0.add(y.p1);
		if (Math.abs(m.x - n.x) > 1e-6) return m.x < n.x ? -1 : 1;
		if (Math.abs(m.y - n.y) > 1e-6) return m.y < n.y ? -1 : 1;
		return m.z < n.z ? -1 : (m.z > n.z ? 1 : 0);
	}

	/** The sign of an edge's first significant component, to give flat seams a direction that does not depend on the edge. */
	static function firstPositive(direction:Vector):Float {
		for (c in [direction.x, direction.y, direction.z]) if (Math.abs(c) > 1e-6) return c;
		return 1;
	}

	/**
	 * Groups seams into chains of seams that meet end to end, as around a tube's perimeter. Each chain is in welding order
	 * and each seam is turned to run on from the one before; the first keeps its direction. `closed` is set for a loop.
	 */
	public static function chains(seams:Array<WeldSeam>):Array<SeamChain> {
		var left = seams.copy();
		var result:Array<SeamChain> = [];
		while (left.length > 0) {
			var chain = [left.shift()];
			var extended = true;
			while (extended) {
				extended = false;
				var end = chain[chain.length - 1].stop;
				for (i in 0...left.length) {
					var next = left[i];
					if (next.start.subtract(end).length() <= TOUCH) chain.push(next);
					else if (next.stop.subtract(end).length() <= TOUCH) chain.push(next.reversed());
					else continue;
					left.splice(i, 1);
					extended = true;
					break;
				}
			}
			var closed = chain.length > 1 && chain[0].start.subtract(chain[chain.length - 1].stop).length() <= TOUCH;
			result.push({seams: chain, closed: closed});
		}
		return result;
	}
}
