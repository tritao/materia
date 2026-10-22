package machinekit.tooling;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import machinekit.component.Solids;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelBlockingRelation;

typedef RemovalMember = {
	var id:String;
	var part:Part;
	var direction:Vector;
}

typedef RemovalReport = {
	var blocking:Array<PanelBlockingRelation>;
	var removalOrder:Array<String>;
	var assemblyOrder:Array<String>;
	var travelMm:Float;
	var contactVolumeToleranceMm3:Float;
}

/** Exact straight-translation sweeps. Inputs remain borrowed; every temporary is closed.
 * Planar/cylindrical faces and axial or transverse conical withdrawals are supported. Other surfaces fail explicitly.
 */
class SweptRemoval {
	static function prism(face:Shape, delta:Vector, pieces:Array<Part>):Void {
		var shape = face.extrude(delta.native());
		var part = new Part(shape);
		pieces.push(part);
		if (!part.valid())
			throw "Invalid boundary-face sweep";
	}

	/** Exact half-space split; each resulting face projects without folds. */
	static function radialSweep(face:Shape,delta:Vector,pieces:Array<Part>):Void {
		var axis = face.faceAxis(),
			d = Vector.fromNative(axis.get_direction()).normalized();
		var projected = delta.subtract(d.scale(delta.dot(d)));
		if (projected.length() > 1e-8) {
			// Split at exact tangent generators of a cylinder, or a cone
			// under purely transverse translation, before extrusion.
			// This prevents a >180-degree face prism from folding over itself.
			var u = projected.normalized(),
				w = u.cross(d).normalized(),
				origin = Vector.fromNative(axis.get_origin());
			var bb = face.bounds(),
				lo = Vector.fromNative(bb.get_min()),
				hi = Vector.fromNative(bb.get_max());
			var length = 2 * (hi.subtract(lo).length() + lo.subtract(origin).length() + 1);
			for (sign in [-1.0, 1.0]) {
				var pose = F.fromRotationMatrix(origin.x, origin.y, origin.z, [u.x, d.x, w.x, u.y, d.y, w.y, u.z, d.z, w.z]);
				var box = Part.box(length, 2 * length, 2 * length);
				var local:Part;
				try
					local = box.translated(new Vector(sign * length / 2, 0, -length))
				catch (e:Dynamic) {
					box.close();
					throw e;
				}
				box.close();
				var clip:Part;
				try
					clip = PosedParts.moved(local, pose)
				catch (e:Dynamic) {
					local.close();
					throw e;
				}
				local.close();
				var half:Shape;
				try
					half = face.common(clip.shape)
				catch (e:Dynamic) {
					clip.close();
					throw e;
				}
				clip.close();
				try {
					for (j in 0...half.subshapeCount(CadKit.ShapeKind.Face)) {
						var sub = half.subshape(CadKit.ShapeKind.Face, j);
						try {
							prism(sub, delta, pieces);
							sub.close();
						} catch (e:Dynamic) {
							sub.close();
							throw e;
						}
					}
					half.close();
				} catch (e:Dynamic) {
					half.close();
					throw e;
				}
			}
		}
	}

	public static function sweep(part:Part, delta:Vector):Part {
		if (delta.length() < 1e-8)
			throw "Removal displacement must be nonzero";
		var pieces:Array<Part> = [];
		try {
			pieces.push(new Part(part.shape.cloneShape()));
			pieces.push(part.translated(delta));
			for (i in 0...part.shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var face = part.shape.subshape(CadKit.ShapeKind.Face, i);
				try {
					if (face.surfaceKind() == CadKit.SurfaceKind.Plane) {
						if (Math.abs(Vector.fromNative(face.faceNormal()).dot(delta.normalized())) > 1e-10)
							prism(face, delta, pieces);
					} else if (face.surfaceKind() == CadKit.SurfaceKind.Cone) {
						var axis=Vector.fromNative(face.faceAxis().get_direction()).normalized();
						if (axis.cross(delta.normalized()).length()<=1e-10)
							prism(face,delta,pieces);
						else if (Math.abs(axis.dot(delta.normalized()))<=1e-10)
							radialSweep(face,delta,pieces);
						else throw "Unsupported removal boundary surface";
					} else if (face.surfaceKind() == CadKit.SurfaceKind.Cylinder) {
						radialSweep(face,delta,pieces);
					} else
						throw "Unsupported removal boundary surface";
					face.close();
				} catch (e:Dynamic) {
					face.close();
					throw e;
				}
			}
			var result = Solids.union(pieces);
			pieces.push(result);
			if (!result.valid()) {
				result.close();
				throw "Invalid union of boundary sweeps";
			}
			return result;
		} catch (e:Dynamic) {
			for (p in pieces)
				if (!p.shape.isClosed())
					p.close();
			throw e;
		}
	}

	public static function intersectionVolume(a:Part, b:Part):Float {
		var common = a.intersect(b);
		try {
			var volume = common.volume();
			common.close();
			return volume;
		} catch (e:Dynamic) {
			common.close();
			throw e;
		}
	}

	/** Full-set diagonal clears every possible projected bound along a unit direction. */
	public static function analyze(members:Array<RemovalMember>, contactVolumeToleranceMm3:Float = 1e-5):RemovalReport {
		if (members == null
			|| members.length == 0
			|| members.length > 128
			|| !Math.isFinite(contactVolumeToleranceMm3)
			|| contactVolumeToleranceMm3 < 0)
			throw "Invalid removal analysis";
		var ids:Array<String> = [],
			low = new Vector(1e30, 1e30, 1e30),
			high = new Vector(-1e30, -1e30, -1e30);
		for (m in members) {
			if (ids.indexOf(m.id) >= 0 || m.id == null || m.id.length == 0)
				throw "Duplicate/invalid removal member";
			ids.push(m.id);
			if (Math.abs(m.direction.length() - 1) > 1e-8)
				throw "Removal direction must be unit length";
			var b = m.part.shape.bounds(),
				a = Vector.fromNative(b.get_min()),
				c = Vector.fromNative(b.get_max());
			low = new Vector(Math.min(low.x, a.x), Math.min(low.y, a.y), Math.min(low.z, a.z));
			high = new Vector(Math.max(high.x, c.x), Math.max(high.y, c.y), Math.max(high.z, c.z));
		}
		var travel = high.subtract(low).length() + 1,
			blocking:Array<PanelBlockingRelation> = [];
		// Exact broad phase: disjoint bounds cannot contain a positive-volume intersection.
		// The accepted pairs still use the complete boundary-face sweep and booleans.
		function meets(a:Part, b:Part):Bool {
			var x = a.shape.bounds(),
				y = b.shape.bounds(),
				al = Vector.fromNative(x.get_min()),
				ah = Vector.fromNative(x.get_max()),
				bl = Vector.fromNative(y.get_min()),
				bh = Vector.fromNative(y.get_max());
			return Math.min(ah.x, bh.x) > Math.max(al.x, bl.x)
				&& Math.min(ah.y, bh.y) > Math.max(al.y, bl.y)
				&& Math.min(ah.z, bh.z) > Math.max(al.z, bl.z);
		}
		for (i in 0...members.length)
			for (j in i + 1...members.length) {
				if (!meets(members[i].part, members[j].part))
					continue;
				var volume = intersectionVolume(members[i].part, members[j].part);
				if (volume > contactVolumeToleranceMm3)
					throw 'Initial overlap ${members[i].id}/${members[j].id}';
			}
		for (a in members) {
			var swept = sweep(a.part, a.direction.scale(travel));
			try {
				for (b in members)
					if (b.id != a.id && meets(swept, b.part)) {
						var volume = intersectionVolume(swept, b.part);
						if (volume > contactVolumeToleranceMm3)
							blocking.push({movingId: a.id, blockerId: b.id});
					}
				swept.close();
			} catch (e:Dynamic) {
				swept.close();
				throw e;
			}
		}
		blocking.sort((a, b) -> {
			var moving = Reflect.compare(a.movingId, b.movingId);
			return moving == 0 ? Reflect.compare(a.blockerId, b.blockerId) : moving;
		});
		var order = PanelAssemblyPlan.topological(ids, blocking),
			assembly = order.copy();
		assembly.reverse();
		return {
			blocking: blocking,
			removalOrder: order,
			assemblyOrder: assembly,
			travelMm: travel,
			contactVolumeToleranceMm3: contactVolumeToleranceMm3
		};
	}
}
