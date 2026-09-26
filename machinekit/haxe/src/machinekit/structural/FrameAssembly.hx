package machinekit.structural;

import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

typedef SectionBounds = {
	var minX:Float;
	var maxX:Float;
	var minY:Float;
	var maxY:Float;
}

/** A cross-section factory for `FrameAssembly` members: `geometry(length)` extrudes the section
 * along local +Z from z=0 to z=length, matching every other machinekit generator's axis
 * convention. `RectTube`, `RoundTube`, `Angle`, `Channel`, and `FlatBar` implement this.
 */
interface StructuralProfile {
	function profileDesignation():String;
	function profileDescription():String;
	function sectionBounds():SectionBounds;
	function geometry(length:Float):Part;
}

/** End treatment for one structural member. `setback` is the stock removed from a member's
 * centreline endpoint for the frame's cut-length envelope. Mitres use a sloped end plane and
 * copes use a curved notch sized from the profile envelope.
 */
enum FrameEndCut {
	Square;
	Mitre(setback:Float);
	Cope(setback:Float);
}

/** Lengths of one profile aggregated across every member that uses it. `totalLength` sums the
 * members' cut lengths after their explicit end setbacks.
 */
typedef CutListLine = {
	var designation:String;
	var description:String;
	var quantity:Int;
	var totalLength:Float;
}

/** Builds a structural frame from named 3D points and members that extrude a `StructuralProfile`
 * between two points. A frame is one rigid weldment, not a kinematic mechanism, so members carry
 * no assembly joints: `geometry(name)` returns the member's `Part` already placed in world space.
 *
 * Members are placed by centreline, node to node. Square ends preserve that envelope; mitre and
 * cope treatments remove the requested stock setback from the corresponding endpoint. `length`
 * remains the node-to-node distance, while `cutLength` and `cutList` report stock lengths.
 *
 * Section orientation: the profile's local +Z runs from `start` to `end`, its local +Y (a
 * channel's `height`, an angle's `legB`, a tube's `height`) follows `reference` projected
 * perpendicular to the member's axis, and its local +X is +Y x +Z. `reference` defaults to +Z,
 * or +Y when the member is nearly vertical, matching the axis-picking convention CadKit's own
 * examples use for arbitrary-direction placement; a `reference` parallel to the member is an
 * error.
 */
typedef FrameMember = {
	var name:String;
	var start:String;
	var end:String;
	var profile:StructuralProfile;
	var reference:Null<Vector>;
	var startCut:FrameEndCut;
	var endCut:FrameEndCut;
}

class FrameAssembly {
	final points:Map<String, Vector> = new Map();
	final members:Array<FrameMember> = [];

	public function new() {}

	public function point(name:String, x:Float, y:Float, z:Float):Void {
		if (name == null || name.length == 0 || points.exists(name)) throw 'Duplicate frame point "$name"';
		points.set(name, new Vector(x, y, z));
	}

	public function member(name:String, start:String, end:String, profile:StructuralProfile, ?reference:Vector,
			?startCut:FrameEndCut, ?endCut:FrameEndCut):Void {
		if (name == null || name.length == 0) throw "Frame member needs a name";
		for (existing in members) if (existing.name == name) throw 'Duplicate frame member "$name"';
		if (!points.exists(start)) throw 'Unknown frame point "$start"';
		if (!points.exists(end)) throw 'Unknown frame point "$end"';
		if (start == end) throw 'Member "$name" needs distinct endpoints';
		if (reference != null && !(reference.length() > 1e-9)) throw 'Member "$name" reference has zero length';
		var resolvedStartCut = startCut == null ? Square : startCut;
		var resolvedEndCut = endCut == null ? Square : endCut;
		var centerline = points.get(end).subtract(points.get(start)).length();
		var stockLength = centerline - cutBack(resolvedStartCut) - cutBack(resolvedEndCut);
		if (centerline > 1e-9 && !(stockLength > 1e-9)) throw 'Member "$name" end cuts consume its length';
		members.push({name: name, start: start, end: end, profile: profile, reference: reference,
			startCut: resolvedStartCut, endCut: resolvedEndCut});
	}

	/** Straight-line (centreline, untrimmed) distance between a member's endpoints. */
	public function length(name:String):Float {
		var m = find(name);
		return points.get(m.end).subtract(points.get(m.start)).length();
	}

	/** Stock length after the member's start and end cut setbacks. */
	public function cutLength(name:String):Float {
		var m = find(name);
		return length(name) - cutBack(m.startCut) - cutBack(m.endCut);
	}

	/** The member's `Part`, extruded to its length and placed in world space along its endpoints. */
	public function geometry(name:String):Part {
		var m = find(name);
		var start = points.get(m.start), end = points.get(m.end);
		var axis = end.subtract(start);
		var len = axis.length();
		if (!(len > 1e-9)) throw 'Member "$name" has coincident endpoints';
		var direction = axis.scale(1 / len);
		var reference = m.reference != null ? m.reference
			: (Math.abs(direction.dot(Vector.Z())) < 0.9 ? Vector.Z() : Vector.Y());
		// Local X = reference x direction makes local Y = direction x X the reference's
		// component perpendicular to the member.
		var sideways = reference.cross(direction);
		if (!(sideways.length() > 1e-6 * reference.length()))
			throw 'Member "$name" reference is parallel to its axis; pick a reference across the member';
		sideways = sideways.normalized();
		var startSetback = cutBack(m.startCut);
		var stockLength = len - startSetback - cutBack(m.endCut);
		var body = m.profile.geometry(stockLength);
		try {
			var section = m.profile.sectionBounds();
			body = applyEndCut(body, m.startCut, true, stockLength, section);
			body = applyEndCut(body, m.endCut, false, stockLength, section);
			var cutStart = start.add(direction.scale(startSetback));
			var placed = body.placed(new Location(new Plane(cutStart, sideways, direction)));
			body.close();
			return placed;
		} catch (error:Dynamic) {
			body.close();
			throw error;
		}
	}

	/** Cut lengths aggregated by profile designation, in first-used order. */
	public function cutList():Array<CutListLine> {
		var byDesignation:Map<String, CutListLine> = new Map();
		var order:Array<String> = [];
		for (m in members) {
			var designation = m.profile.profileDesignation();
			var len = cutLength(m.name);
			var line = byDesignation.get(designation);
			if (line == null) {
				byDesignation.set(designation, {designation: designation, description: m.profile.profileDescription(),
					quantity: 1, totalLength: len});
				order.push(designation);
			} else {
				line.quantity++;
				line.totalLength += len;
				byDesignation.set(designation, line);
			}
		}
		return [for (designation in order) byDesignation.get(designation)];
	}

	function find(name:String):FrameMember {
		for (m in members) if (m.name == name) return m;
		throw 'Unknown frame member "$name"';
	}

	static function cutBack(cut:FrameEndCut):Float {
		return switch (cut) {
			case Square: 0;
			case Mitre(setback) | Cope(setback):
				if (!(setback > 0) || !Math.isFinite(setback)) throw "Frame end-cut setback must be positive";
				setback;
		};
	}

	static function applyEndCut(body:Part, cut:FrameEndCut, atStart:Bool, length:Float, bounds:SectionBounds):Part {
		return switch (cut) {
			case Square: body;
			case Mitre(setback):
				var tool = mitreTool(length, bounds, atStart, setback);
				intersectClosing(body, tool);
			case Cope(setback):
				var radius = Math.min(setback, Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY));
				var yMargin = Math.max(1, (bounds.maxY - bounds.minY) * 0.1);
				var tool = Part.cylinder(radius, bounds.maxY - bounds.minY + 2 * yMargin);
				var origin = new Location(new Plane(new Vector((bounds.minX + bounds.maxX) / 2,
					bounds.minY - yMargin, atStart ? 0 : length), Vector.X(), Vector.Y()));
				var placed = tool.placed(origin);
				tool.close();
				var result = Solids.cut(body, [placed]);
				result;
		};
	}

	static function mitreTool(length:Float, bounds:SectionBounds, atStart:Bool, setback:Float):Part {
		var xSpan = bounds.maxX - bounds.minX;
		var xMargin = Math.max(1, xSpan * 0.1);
		var x0 = bounds.minX - xMargin, x1 = bounds.maxX + xMargin;
		var z0 = atStart ? setback * (x0 - bounds.minX) / xSpan : 0;
		var z1 = atStart ? setback * (x1 - bounds.minX) / xSpan : length - setback * (x1 - bounds.minX) / xSpan;
		var far = length + setback + 1;
		var points = atStart
			? [new Vector(x0, z0), new Vector(x1, z1), new Vector(x1, far), new Vector(x0, far)]
			: [new Vector(x0, -1), new Vector(x1, -1), new Vector(x1, z1), new Vector(x0, length - setback * (x0 - bounds.minX) / xSpan)];
		var yMargin = Math.max(1, (bounds.maxY - bounds.minY) * 0.1);
		var plane = new Plane(new Vector(0, bounds.maxY + yMargin, 0), Vector.X(), Vector.Y().scale(-1));
		var sketch = Sketch.polygon(points, plane);
		try {
			var result = sketch.extrude(bounds.maxY - bounds.minY + 2 * yMargin);
			sketch.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}

	static function intersectClosing(base:Part, tool:Part):Part {
		try {
			var result = base.intersect(tool);
			base.close();
			tool.close();
			return result;
		} catch (error:Dynamic) {
			base.close();
			tool.close();
			throw error;
		}
	}
}
