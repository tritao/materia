package machinekit.welding;

import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** How the two members of a seam meet, found from the faces' geometry (see `WeldSeams`). */
enum JointType {
	/** One member stands on the other's face, with its contact face the narrower (a T-joint). */
	Fillet;
	/** One member lies on the other's face, with its contact face the wider, so its edge is the corner. */
	Lap;
	/** Two end faces meet edge to edge and the outer faces run on flat. */
	Butt;
	/** Two end faces meet edge to edge and the outer faces turn (a mitred or square corner). */
	Corner;
}

/** One face of a seam: a member occurrence and the face's topological name in that member's geometry. */
typedef SeamFace = {
	var member:String;
	var face:String;
}

/**
 * The pose a torch holds at one place on a seam. All vectors are unit and expressed in the frame the
 * seam is in (the workpiece's, until `transformed` moves it).
 */
class SeamFrame {
	public final position:Vector;
	/** The direction of travel along the seam. */
	public final tangent:Vector;
	/** Bisector of the two faces' outward normals, pointing out of the corner: the torch axis at 0 work and travel angle. */
	public final bisector:Vector;
	/** The torch's axis after the work and travel angles, from the wire tip back toward the torch body. */
	public final torchAxis:Vector;

	public function new(position:Vector, tangent:Vector, bisector:Vector, torchAxis:Vector) {
		this.position = position;
		this.tangent = tangent;
		this.bisector = bisector;
		this.torchAxis = torchAxis;
	}

	/** The direction the wire points, from the torch toward the work. */
	public function wire():Vector return torchAxis.scale(-1);

	/**
	 * The wire tip's pose for a robot: +Z along the wire (the torch connector's convention), +X along the direction of
	 * travel as far as it is square to the wire, and Y completing the right-handed frame.
	 */
	public function toAssemblyFrame():AssemblyFrame {
		var z = wire();
		var x = tangent.subtract(z.scale(tangent.dot(z))).normalized();
		var y = z.cross(x);
		return AssemblyFrames.fromRotationMatrix(position.x, position.y, position.z, [x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z]);
	}

	/** This frame in the parent of the frame it is in now, with `frame` its pose there. */
	public function transformed(frame:AssemblyFrame):SeamFrame {
		function point(v:Vector):Vector {
			var p = AssemblyFrames.transformPoint(frame, v.x, v.y, v.z);
			return new Vector(p.x, p.y, p.z);
		}
		function direction(v:Vector):Vector {
			var d = AssemblyFrames.transformVector(frame, v.x, v.y, v.z);
			return new Vector(d.x, d.y, d.z);
		}
		return new SeamFrame(point(position), direction(tangent), direction(bisector), direction(torchAxis));
	}
}

/**
 * A weld seam found where two members meet: a straight edge between a face of each, in the frame of the
 * workpiece. It is never authored. The faces are named by their topological names, so the seam keeps
 * its name, and follows its edge, as the members are edited.
 *
 * The torch frame comes from the geometry. The tangent runs along the edge; the torch axis is the
 * bisector of the two faces' outward normals (the open side of the corner), then tilted by the work angle
 * about the tangent and by the travel angle about the work-angled axis. A positive travel angle pushes
 * (the wire points ahead of the torch, in the direction of travel); a negative one drags. A positive work
 * angle leans the axis toward the first face's side.
 */
class WeldSeam {
	/** Default push angle for MIG welding, in radians. */
	public static inline var PUSH_ANGLE:Float = 0.17453292519943295;

	/** `a` and `b` are ordered by member then face name, so the same pair always gives the same seam name. */
	public final a:SeamFace;
	public final b:SeamFace;
	public final joint:JointType;
	public final start:Vector;
	public final stop:Vector;
	/** The faces' outward normals, along the seam. */
	public final normalA:Vector;
	public final normalB:Vector;
	/** Intended leg size in millimetres: how far the weld reaches along each face. */
	public final legSize:Float;
	public final passes:Int;
	public final travelAngle:Float;
	public final workAngle:Float;
	/** Which of several seams between the same two faces this is, in order along the faces; 0 when there is one. */
	public final piece:Int;

	public function new(a:SeamFace, b:SeamFace, joint:JointType, start:Vector, stop:Vector, normalA:Vector, normalB:Vector,
			legSize:Float, passes:Int, travelAngle:Float, workAngle:Float, piece:Int = 0) {
		if (!(stop.subtract(start).length() > 1e-6)) throw "A weld seam needs a start and a stop apart";
		if (!(legSize >= 0) || passes < 1) throw "A weld seam needs a leg size and at least one pass";
		this.a = a;
		this.b = b;
		this.joint = joint;
		this.start = start;
		this.stop = stop;
		this.normalA = normalA.normalized();
		this.normalB = normalB.normalized();
		this.legSize = legSize;
		this.passes = passes;
		this.travelAngle = travelAngle;
		this.workAngle = workAngle;
		this.piece = piece;
	}

	/** The seam's name: its two faces, `member:face|member:face`, then `~n` when the faces meet in several seams. */
	public function name():String return '${a.member}:${a.face}|${b.member}:${b.face}' + (piece > 0 ? '~$piece' : "");

	public function length():Float return stop.subtract(start).length();

	public function tangent():Vector return stop.subtract(start).normalized();

	/** Midpoint of the seam. */
	public function centre():Vector return start.add(stop).scale(0.5);

	/** Angle between the two faces' outward normals, in radians: 90 degrees for a square corner, 0 for a butt. */
	public function normalAngle():Float return Math.atan2(normalA.cross(normalB).length(), normalA.dot(normalB));

	/** The torch axis with no work and travel angle: out of the corner, between the two faces. */
	public function bisector():Vector {
		var sum = normalA.add(normalB);
		// Faces whose outward normals oppose have no open side to weld.
		if (!(sum.length() > 1e-6)) throw 'Seam ${name()} has no open side between its faces';
		return sum.normalized();
	}

	/** The seam welded the other way: from its stop to its start. The name is the same. */
	public function reversed():WeldSeam
		return new WeldSeam(a, b, joint, stop, start, normalA, normalB, legSize, passes, travelAngle, workAngle, piece);

	/** The same seam with other process settings; an argument left out keeps its value. */
	public function configured(?leg:Float, ?passCount:Int, ?travel:Float, ?work:Float):WeldSeam
		return new WeldSeam(a, b, joint, start, stop, normalA, normalB, leg == null ? legSize : leg,
			passCount == null ? passes : passCount, travel == null ? travelAngle : travel, work == null ? workAngle : work, piece);

	/** The torch frame `distance` along the seam from its start, clamped to the seam. */
	public function frameAt(distance:Float):SeamFrame
		return frameAtParameter(Math.max(0, Math.min(1, distance / length())));

	/** The torch frame at `fraction` of the way along the seam, 0 at the start and 1 at the stop. */
	public function frameAtParameter(fraction:Float):SeamFrame {
		var t = tangent();
		var position = start.add(stop.subtract(start).scale(Math.max(0, Math.min(1, fraction))));
		var open = bisector();
		// The sideways axis in the cross-section, toward the first face's side.
		var side = t.cross(open);
		if (side.length() > 1e-9) {
			side = side.normalized();
			if (side.dot(normalA) < 0) side = side.scale(-1);
		}
		var worked = open.scale(Math.cos(workAngle)).add(side.scale(Math.sin(workAngle)));
		// Pushing swings the wire (the opposite of the axis) toward the travel direction.
		var wire = worked.scale(-Math.cos(travelAngle)).add(t.scale(Math.sin(travelAngle)));
		return new SeamFrame(position, t, open, wire.scale(-1).normalized());
	}

	/** This seam in the parent of the frame it is in now, with `frame` its pose there. */
	public function transformed(frame:AssemblyFrame):WeldSeam {
		function point(v:Vector):Vector {
			var p = AssemblyFrames.transformPoint(frame, v.x, v.y, v.z);
			return new Vector(p.x, p.y, p.z);
		}
		function direction(v:Vector):Vector {
			var d = AssemblyFrames.transformVector(frame, v.x, v.y, v.z);
			return new Vector(d.x, d.y, d.z);
		}
		return new WeldSeam(a, b, joint, point(start), point(stop), direction(normalA), direction(normalB), legSize, passes,
			travelAngle, workAngle, piece);
	}
}
