package machinekit.screwdriving;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.threads.Threads;
import cadkit.threads.Threads.ResolvedThread;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.standard.ScrewFastener;
import machinekit.standard.FastenerTightening.TighteningTableRow;
import machinekit.screwdriving.FastenedAssembly.FasteningPolicy;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Every seated screw yields either a connection or a diagnostic naming the failure. */
class FasteningsResult {
	public final fastenings:Array<Fastening>;
	public final diagnostics:Diagnostics;
	public function new(fastenings:Array<Fastening>, diagnostics:Diagnostics) {
		this.fastenings = fastenings; this.diagnostics = diagnostics;
	}
	public function require():Array<Fastening> { diagnostics.throwIfErrors(); return fastenings; }
	public function named(name:String):Null<Fastening> {
		for (fastening in fastenings) if (fastening.name() == name) return fastening;
		return null;
	}
}

private typedef Bore = {
	var index:Int; var name:String; var radius:Float; var minimum:Float; var maximum:Float;
}
private typedef ThreadCandidate = {
	var member:String; var thread:ResolvedThread; var minimum:Float; var maximum:Float; var bore:Bore;
}

/** Seat mates select the clamped occurrence; semantic named threads select the threaded occurrence. */
class Fastenings {
	public static inline var TOLERANCE:Float = 1e-4;

	public static function find(assembly:MachineAssembly, policies:Map<String, FasteningPolicy> = null):FasteningsResult {
		var diagnostics = new Diagnostics(), fastenings:Array<Fastening> = [];
		var parts:Map<String, Part> = [], components:Map<String, MachineComponent> = [];
		var threads:Map<String, Array<ResolvedThread>> = [];
		try {
			var poses = assembly.solvedPoses();
			for (entry in assembly.components()) {
				components.set(entry.id, entry.component);
				if (!entry.component.hasGeometry()) continue;
				try {
					var part = posed(entry.component.geometry(ComponentDetail.Preview), poses.get(entry.id));
					parts.set(entry.id, part);
					threads.set(entry.id, Threads.of(part));
				} catch (error:Dynamic) {
					diagnostics.error("fastening.thread-reference", entry.id, '${entry.id}: $error');
				}
			}
			var memberIds = [for (member in components.keys()) member];
			memberIds.sort(Reflect.compare);
			var visited:Map<String, Bool> = [], ambiguousSeats:Map<String, Bool> = [];
			for (joint in assembly.definition().joints) {
				if (joint.type != "fixed") continue;
				var parent = components.get(joint.parent), child = components.get(joint.child);
				var screwId:Null<String> = null, clampedId:Null<String> = null;
				if (Std.isOfType(child, ScrewFastener) && joint.childConnector == "head") {
					screwId = joint.child; clampedId = joint.parent;
				} else if (Std.isOfType(parent, ScrewFastener) && joint.parentConnector == "head") {
					screwId = joint.parent; clampedId = joint.child;
				} else continue;
				var id:String = cast screwId, clamp:String = cast clampedId;
				if (visited.exists(id)) { ambiguousSeats.set(id, true); diagnostics.error("fastening.ambiguous-seat", id, '$id has more than one fixed head seat'); continue; }
				visited.set(id, true);
				var screw:ScrewFastener = cast components.get(id);
				var screwThreads = threads.get(id), clampPart = parts.get(clamp);
				if (screwThreads == null || screwThreads.length != 1 || clampPart == null) {
					diagnostics.error("fastening.missing-geometry", id, '$id or its clamped member $clamp has no unique thread geometry'); continue;
				}
				var external = screwThreads[0];
				var component:MachineComponent = cast components.get(id);
				var head = AssemblyFrames.compose(cast poses.get(id), component.connector("head").frame);
				var origin = new Vector(head.x, head.y, head.z), axis = external.frame.normal;
				var clearances:Array<Bore> = [];
				for (index in 0...clampPart.shape.subshapeCount(CadKit.ShapeKind.Face)) {
					var bore = boreOnAxis(clampPart.shape, index, origin, axis);
					if (bore != null && bore.minimum <= TOLERANCE && bore.maximum > TOLERANCE && inward(clampPart.shape, index))
						clearances.push(bore);
				}
				if (clearances.length != 1) {
					diagnostics.error("fastening.clearance", id, '$id seat on $clamp has ${clearances.length} clearance faces on its axis'); continue;
				}
				var clearance = clearances[0];
				if (2 * clearance.radius + TOLERANCE < screw.threadSpec().majorDiameter) {
					diagnostics.error("fastening.clearance-small", id, '$id clearance $clamp/${clearance.name} is ${2 * clearance.radius} mm, below ${screw.threadSpec().majorDiameter} mm'); continue;
				}
				var candidates:Array<ThreadCandidate> = [];
				for (member in memberIds) {
					if (member == id || member == clamp || Std.isOfType(components.get(member), ScrewFastener)) continue;
					var memberThreads = threads.get(member);
					if (memberThreads == null) continue;
					for (thread in memberThreads) {
						if (!thread.attribute.spec.internal || !coaxial(thread.frame.origin, thread.frame.normal, origin, axis)) continue;
						var first = projection(thread.frame.origin, origin, axis);
						var last = projection(thread.frame.toWorld(new Vector(0, 0, thread.length)), origin, axis);
						var low = Math.min(first, last), high = Math.max(first, last);
						if (low < clearance.maximum - TOLERANCE) continue;
						var memberPart:Part = cast parts.get(member);
						var bore = boreOnAxis(memberPart.shape, thread.faceIndex, origin, axis, true);
						if (bore != null) candidates.push({member: member, thread: thread, minimum: low, maximum: high, bore: bore});
					}
				}
				candidates.sort((a, b) -> a.bore.minimum < b.bore.minimum ? -1 : a.bore.minimum > b.bore.minimum ? 1 :
					(a.member != b.member ? Reflect.compare(a.member, b.member) : Reflect.compare(a.thread.attribute.faceName, b.thread.attribute.faceName)));
				if (candidates.length == 0) {
					diagnostics.error("fastening.no-thread", id, '$id through $clamp/${clearance.name} has no named internal thread on its axis'); continue;
				}
				var target = candidates[0];
				if (candidates.length > 1 && Math.abs(candidates[1].bore.minimum - target.bore.minimum) < TOLERANCE) {
					diagnostics.error("fastening.ambiguous-thread", id, '$id has ambiguous threads ${target.member}/${target.thread.attribute.faceName} and ${candidates[1].member}/${candidates[1].thread.attribute.faceName}'); continue;
				}
				var subject = '$id -> ${target.member}/${target.thread.attribute.faceName}';
				if (!external.attribute.spec.matches(target.thread.attribute.spec)) {
					diagnostics.error("fastening.thread-mismatch", id, '$subject: ${external.attribute.spec.designation} does not match ${target.thread.attribute.spec.designation} (diameter, pitch, starts, hand or profile)'); continue;
				}
				var start = projection(external.frame.origin, origin, axis), end = start + external.length;
				var actualScrew:Part = cast parts.get(id);
				var tip = axialEnd(actualScrew, origin, external.frame.xDirection, axis);
				var intervening = false;
				for (member in memberIds) {
					if (member == id || member == clamp || member == target.member || Std.isOfType(components.get(member), ScrewFastener)) continue;
					var other = parts.get(member);
					if (other == null) continue;
					for (index in 0...other.shape.subshapeCount(CadKit.ShapeKind.Face)) {
						var bore = boreOnAxis(other.shape, index, origin, axis, false, true);
						if (bore != null && bore.minimum >= clearance.maximum - TOLERANCE && bore.minimum < target.bore.minimum - TOLERANCE
							&& inward(other.shape, index)) {
							diagnostics.error("fastening.no-thread", id, '$id meets ${member}/${bore.name} before $subject; the next part has no matching named coaxial thread');
							intervening = true; break;
						}
					}
					if (occupied(other, origin, axis, clearance.maximum, Math.min(tip, target.bore.minimum), Math.min(clearance.radius, target.bore.radius))) {
						diagnostics.error("fastening.blocked-path", id, '$id path from $clamp/${clearance.name} to $subject crosses solid member $member');
						intervening = true;
					}
				}
				if (intervening) continue;
				var engagement = Math.max(0, Math.min(end, target.maximum) - Math.max(start, target.minimum));
				var policy = policies == null ? null : policies.get(id);
				var diameters = policy != null && policy.minimumDiameters != null ? policy.minimumDiameters : defaultMinimum(components.get(target.member), diagnostics, id);
				if (diameters == null) continue;
				var minimum:Float = screw.threadSpec().majorDiameter * diameters;
				var valid = true;
				if (engagement + TOLERANCE < minimum) {
					diagnostics.error("fastening.under-engagement", id, '$subject: engagement $engagement mm is below $minimum mm'); valid = false;
				}
				if (start > target.bore.minimum + TOLERANCE) {
					diagnostics.error("fastening.plain-shank", id, '$subject: plain shank enters the threaded bore'); valid = false;
				}
				if (bottoms(parts.get(target.member), target.bore, origin, axis, tip)) {
					diagnostics.error("fastening.blind-bottom", id, '$subject: screw tip at $tip mm exceeds blind bore end ${target.bore.maximum} mm'); valid = false;
				}
				var tightening:Null<TighteningTableRow> = null;
				try tightening = screw.tightening() catch (error:Dynamic) {
					diagnostics.error("fastening.tightening-data", id, '$subject: $error'); valid = false;
				}
				if (valid) fastenings.push(new Fastening({member: id, faceName: external.attribute.faceName},
					{member: clamp, faceName: clearance.name}, {member: target.member, faceName: target.thread.attribute.faceName},
					new Plane(origin.add(axis.scale(target.bore.minimum)), external.frame.xDirection, axis), engagement, minimum,
					external.attribute.spec, cast tightening, policy == null ? null : policy.angleWindow));
			}
			if (policies != null) for (id in policies.keys()) if (!visited.exists(id))
				diagnostics.error("fastening.missing-seat", id, 'Declared fastening $id has no fixed head seat');
			for (part in parts) part.close();
			fastenings = [for (fastening in fastenings) if (!ambiguousSeats.exists(fastening.screw.member)) fastening];
			fastenings.sort((a, b) -> Reflect.compare(a.name(), b.name()));
			return new FasteningsResult(fastenings, diagnostics);
		} catch (error:Dynamic) { for (part in parts) part.close(); throw error; }
	}

	static function defaultMinimum(component:MachineComponent, diagnostics:Diagnostics, id:String):Null<Float> {
		var material = component.materialSpec();
		if (material.indexOf("aluminium") >= 0) return 1.5;
		if (material.indexOf("steel") >= 0) return 1;
		diagnostics.error("fastening.engagement-material", id, '$id needs an explicit engagement minimum for threaded material $material');
		return null;
	}
	static function projection(point:Vector, origin:Vector, axis:Vector):Float return point.subtract(origin).dot(axis);
	static function coaxial(point:Vector, direction:Vector, origin:Vector, axis:Vector):Bool {
		var delta = point.subtract(origin);
		return Math.abs(direction.dot(axis)) > 1 - 1e-6 && delta.subtract(axis.scale(delta.dot(axis))).length() <= TOLERANCE;
	}
	static function boreOnAxis(shape:Shape, index:Int, origin:Vector, axis:Vector, namedThread:Bool = false, containsAxis:Bool = false):Null<Bore> {
		var face = shape.subshape(CadKit.ShapeKind.Face, index);
		try {
			var kind = face.surfaceKind();
			if (kind != CadKit.SurfaceKind.Cylinder && !(namedThread && kind == CadKit.SurfaceKind.Cone)) { face.close(); return null; }
			var geometry = face.faceAxis();
			var point = Vector.fromNative(geometry.get_origin()), direction = Vector.fromNative(geometry.get_direction());
			var radius = geometry.get_radius();
			if (containsAxis) {
				var delta = point.subtract(origin);
				if (Math.abs(direction.dot(axis)) < 1 - 1e-6 || delta.subtract(axis.scale(delta.dot(axis))).length() > radius + TOLERANCE) { face.close(); return null; }
			} else if (!coaxial(point, direction, origin, axis)) { face.close(); return null; }
			if (kind == CadKit.SurfaceKind.Cone) {
				radius = 1e300;
				for (edgeIndex in 0...face.subshapeCount(CadKit.ShapeKind.Edge)) {
					var edge = face.subshape(CadKit.ShapeKind.Edge, edgeIndex);
					try {
						if (edge.curveKind() == CadKit.CurveKind.Circle) radius = Math.min(radius, edge.edgeAxis().get_radius());
						edge.close();
					} catch (error:Dynamic) { edge.close(); throw error; }
				}
				if (radius == 1e300 || radius <= 0) throw 'Named conical thread ${shape.elementName(CadKit.ShapeKind.Face, index)} has no circular bore boundary';
			}
			var extent = Threads.extent(face);
			var first = projection(point.add(direction.scale(extent.minimum)), origin, axis);
			var last = projection(point.add(direction.scale(extent.maximum)), origin, axis);
			face.close();
			return {index: index, name: shape.elementName(CadKit.ShapeKind.Face, index), radius: radius, minimum: Math.min(first, last), maximum: Math.max(first, last)};
		} catch (error:Dynamic) { face.close(); throw error; }
	}
	/** Normals only distinguish a bore from an outside cylinder; they never select a thread. */
	static function inward(shape:Shape, index:Int):Bool {
		var face = shape.subshape(CadKit.ShapeKind.Face, index);
		try {
			var axis = face.faceAxis(), mesh = face.tessellate();
			var point = new Vector(mesh.vertices.getDouble(0), mesh.vertices.getDouble(8), mesh.vertices.getDouble(16));
			var normal = new Vector(mesh.normals.getDouble(0), mesh.normals.getDouble(8), mesh.normals.getDouble(16));
			var delta = point.subtract(Vector.fromNative(axis.get_origin())), direction = Vector.fromNative(axis.get_direction());
			var result = normal.dot(delta.subtract(direction.scale(delta.dot(direction)))) < 0;
			face.close(); return result;
		} catch (error:Dynamic) { face.close(); throw error; }
	}
	/** Tip depth comes from the current solid in the screw-axis frame, not a nominal length cache. */
	static function axialEnd(part:Part, origin:Vector, x:Vector, axis:Vector):Float {
		var local = new Location(new Plane(origin, x, axis)).inverse().apply(part.shape);
		try { var end = local.bounds().get_max().get_z(); local.close(); return end; }
		catch (error:Dynamic) { local.close(); throw error; }
	}
	static function bottoms(part:Part, bore:Bore, origin:Vector, axis:Vector, tip:Float):Bool {
		return occupied(part, origin, axis, bore.maximum, tip, bore.radius);
	}
	/** Query only the solid: manufacturing attributes do not participate in a volume probe. */
	static function occupied(part:Part, origin:Vector, axis:Vector, start:Float, end:Float, radius:Float):Bool {
		if (end <= start + TOLERANCE) return false;
		var probe = Part.cylinder(radius - Math.min(TOLERANCE, radius * 1e-4), end - start - TOLERANCE);
		var placed = probe.placed(new Location(new Plane(origin.add(axis.scale(start + TOLERANCE)),
			Math.abs(axis.x) < 0.9 ? axis.cross(Vector.X()).normalized() : axis.cross(Vector.Y()).normalized(), axis)));
		probe.close();
		var solid = part.shape.cloneShape();
		solid.setThreadedFaces([]);
		try {
			var overlap = solid.common(placed.shape);
			var result = overlap.volume() > 1e-8;
			overlap.close(); solid.close(); placed.close(); return result;
		} catch (error:Dynamic) { solid.close(); placed.close(); throw error; }
	}
	static function posed(local:Part, pose:AssemblyFrame):Part {
		try {
			var x = AssemblyFrames.transformVector(pose, 1, 0, 0), z = AssemblyFrames.transformVector(pose, 0, 0, 1);
			var result = local.placed(new Location(new Plane(new Vector(pose.x, pose.y, pose.z),
				new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z))));
			local.close(); return result;
		} catch (error:Dynamic) { local.close(); throw error; }
	}
}
