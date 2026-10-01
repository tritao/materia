package cadkit.parametric;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.ParametricError;

/** Small geometric fallback key used when an operation has no direct mapping. */
class TopologyFingerprint {
	private static inline var AbsolutePositionTolerance:Float = 0.000001;
	private static inline var RelativePositionTolerance:Float = 0.00000001;
	private static inline var RelativeSizeTolerance:Float = 0.000001;
	private static inline var MinimumDirectionAgreement:Float = 0.999999;

	public final kind:CadKit.ShapeKind;
	public final surfaceKind:CadKit.SurfaceKind;
	public final curveKind:CadKit.CurveKind;
	public final x:Float;
	public final y:Float;
	public final z:Float;
	public final dx:Float;
	public final dy:Float;
	public final dz:Float;
	public final measure:Float;
	/**
		Edges only: (x, y, z) is the edge's midpoint (true, captured since document version 10) rather than its
		first vertex (false, older documents). The midpoint does not depend on the edge's orientation or, for a
		closed edge, on where its seam vertex lies.
	*/
	public final midpoint:Bool;
	/**
		The element's topological name (plans/TOPOLOGICAL_NAMING.md), captured since document version 11; null for
		older records and unnamed shapes. Resolution tries it first (`TopologyResolver`); the geometry above is then
		the tie-breaker and the fallback.
	*/
	public final name:Null<String>;

	private function new(
		kind:CadKit.ShapeKind,
		surfaceKind:CadKit.SurfaceKind,
		curveKind:CadKit.CurveKind,
		x:Float,
		y:Float,
		z:Float,
		dx:Float,
		dy:Float,
		dz:Float,
		measure:Float,
		midpoint:Bool,
		?name:String) {
		this.kind = kind;
		this.surfaceKind = surfaceKind;
		this.curveKind = curveKind;
		this.x = x;
		this.y = y;
		this.z = z;
		this.dx = dx;
		this.dy = dy;
		this.dz = dz;
		this.measure = measure;
		this.midpoint = midpoint;
		this.name = name;
	}

	public static function fromData(
		kind:CadKit.ShapeKind,
		surfaceKind:CadKit.SurfaceKind,
		curveKind:CadKit.CurveKind,
		x:Float,
		y:Float,
		z:Float,
		dx:Float,
		dy:Float,
		dz:Float,
		measure:Float,
		midpoint:Bool = true,
		?name:String):TopologyFingerprint {
		return new TopologyFingerprint(
			kind, surfaceKind, curveKind, x, y, z, dx, dy, dz, measure, midpoint, name);
	}

	/**
		`shape`'s fingerprint and name; an edge's position is its midpoint, or its first vertex when `midpoint` is false
		(older documents). The name is the element's own: a face or edge taken from a named shape keeps its name.
	*/
	public static function capture(shape:Shape, midpoint:Bool = true):TopologyFingerprint {
		var measured = geometryOf(shape, midpoint);
		var names = shape.elementNames(measured.kind);
		if (names.length != 1)
			return measured;
		return new TopologyFingerprint(measured.kind, measured.surfaceKind, measured.curveKind, measured.x, measured.y, measured.z,
			measured.dx, measured.dy, measured.dz, measured.measure, measured.midpoint, names[0]);
	}

	/** A short description for people choosing between elements: "planar face at (30, 20, 10), 1100 mm²". */
	public function describe():String {
		var at = 'at (${round(x)}, ${round(y)}, ${round(z)})';
		if (kind == CadKit.ShapeKind.Face)
			return '${surfaceWord()} face $at, ${round(measure)} mm²';
		if (kind == CadKit.ShapeKind.Edge)
			return '${curveWord()} edge $at, ${round(measure)} mm long';
		return 'vertex $at';
	}

	function surfaceWord():String {
		if (surfaceKind == CadKit.SurfaceKind.Plane) return "planar";
		if (surfaceKind == CadKit.SurfaceKind.Cylinder) return "cylindrical";
		if (surfaceKind == CadKit.SurfaceKind.Cone) return "conical";
		if (surfaceKind == CadKit.SurfaceKind.Sphere) return "spherical";
		if (surfaceKind == CadKit.SurfaceKind.Torus) return "toroidal";
		return "curved";
	}

	function curveWord():String {
		if (curveKind == CadKit.CurveKind.Line) return "straight";
		if (curveKind == CadKit.CurveKind.Circle) return "circular";
		if (curveKind == CadKit.CurveKind.Ellipse) return "elliptical";
		return "curved";
	}

	static function round(value:Float):Float
		return Math.round(value * 100) / 100;

	/** `fingerprint`'s geometry alone, for resolution that must not consult names. */
	public static function withoutName(fingerprint:TopologyFingerprint):TopologyFingerprint {
		return new TopologyFingerprint(fingerprint.kind, fingerprint.surfaceKind, fingerprint.curveKind, fingerprint.x, fingerprint.y,
			fingerprint.z, fingerprint.dx, fingerprint.dy, fingerprint.dz, fingerprint.measure, fingerprint.midpoint);
	}

	/** `shape`'s geometry alone. */
	static function geometryOf(shape:Shape, midpoint:Bool):TopologyFingerprint {
		var kind = shape.kind();
		if (kind == CadKit.ShapeKind.Face) {
			var surface = shape.surfaceKind();
			var center = shape.center();
			var normal = shape.faceNormal();
			return new TopologyFingerprint(
				kind,
				surface,
				CadKit.CurveKind.Unknown,
				center.get_x(),
				center.get_y(),
				center.get_z(),
				normal.get_x(),
				normal.get_y(),
				normal.get_z(),
				shape.faceArea(),
				false);
		} else if (kind == CadKit.ShapeKind.Edge) {
			var tangent = shape.tangentAt();
			var edgeCenter:CadKit.Vec3;
			if (midpoint) {
				edgeCenter = shape.positionAt(0.5);
			} else {
				var endpoint = shape.subshape(CadKit.ShapeKind.Vertex, 0);
				edgeCenter = endpoint.position();
				endpoint.close();
			}
			return new TopologyFingerprint(
				kind,
				CadKit.SurfaceKind.Unknown,
				shape.curveKind(),
				edgeCenter.get_x(),
				edgeCenter.get_y(),
				edgeCenter.get_z(),
				tangent.get_x(),
				tangent.get_y(),
				tangent.get_z(),
				shape.edgeLength(),
				midpoint);
		} else if (kind == CadKit.ShapeKind.Vertex) {
			var position = shape.position();
			return new TopologyFingerprint(
				kind,
				CadKit.SurfaceKind.Unknown,
				CadKit.CurveKind.Unknown,
				position.get_x(),
				position.get_y(),
				position.get_z(),
				0.0,
				0.0,
				0.0,
				0.0,
				false);
		} else {
			throw new ParametricError("topology references require faces, edges, or vertices");
		}
	}

	/** How well `candidate` matches (see `scoreAgainst`); its kind and surface or curve kind are checked before it is measured. */
	public function score(candidate:Shape):Float {
		if (candidate.kind() != kind)
			return -1.0e30;
		if (kind == CadKit.ShapeKind.Face && candidate.surfaceKind() != surfaceKind)
			return -1.0e30;
		if (kind == CadKit.ShapeKind.Edge && candidate.curveKind() != curveKind)
			return -1.0e30;
		return scoreAgainst(geometryOf(candidate, midpoint));
	}

	/**
		How well a candidate's fingerprint matches this one: about 1 for the same face, edge or vertex up to
		numerical noise, -1e30 for anything else (another kind, surface or curve, direction, size or place).
		Fingerprints alone suffice, so a matcher without the geometry (an editor holding descriptors) agrees
		with one that has it. Edge positions are compared only between fingerprints with the same anchor.
	*/
	public function scoreAgainst(candidate:TopologyFingerprint):Float {
		if (candidate.kind != kind)
			return -1.0e30;
		var offsetX = candidate.x - x, offsetY = candidate.y - y, offsetZ = candidate.z - z;
		var distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY + offsetZ * offsetZ);
		var positionScale:Float;
		var normalizedSizeChange = 0.0;
		var directionAgreement = 1.0;
		if (kind == CadKit.ShapeKind.Face) {
			if (candidate.surfaceKind != surfaceKind)
				return -1.0e30;
			var referenceSize = Math.pow(measure, 0.5);
			var candidateSize = Math.pow(candidate.measure, 0.5);
			positionScale = Math.max(referenceSize, candidateSize);
			normalizedSizeChange = relativeSizeChange(referenceSize, candidateSize);
			var dot = dx * candidate.dx + dy * candidate.dy + dz * candidate.dz;
			if (dot < MinimumDirectionAgreement)
				return -1.0e30;
			directionAgreement = dot;
		} else if (kind == CadKit.ShapeKind.Edge) {
			if (candidate.curveKind != curveKind || candidate.midpoint != midpoint)
				return -1.0e30;
			var referenceSize = measure;
			var candidateSize = candidate.measure;
			positionScale = Math.max(referenceSize, candidateSize);
			normalizedSizeChange = relativeSizeChange(referenceSize, candidateSize);
			var tangentDot = Math.abs(dx * candidate.dx + dy * candidate.dy + dz * candidate.dz);
			if (tangentDot < MinimumDirectionAgreement)
				return -1.0e30;
			directionAgreement = tangentDot;
		} else if (kind == CadKit.ShapeKind.Vertex) {
			positionScale = 1.0;
		} else {
			return -1.0e30;
		}

		if (positionScale <= 1.0e-12 || normalizedSizeChange > RelativeSizeTolerance)
			return -1.0e30;
		var normalizedDistance = distance / positionScale;
		// Coordinates and measurements use CadKit's canonical millimetre units. Keep
		// fingerprint matching close enough to absorb numerical noise, not nearby topology.
		var maximumDistance = Math.max(AbsolutePositionTolerance, positionScale * RelativePositionTolerance);
		if (distance > maximumDistance)
			return -1.0e30;
		return directionAgreement - normalizedDistance - normalizedSizeChange;
	}

	private function relativeSizeChange(oldValue:Float, newValue:Float):Float {
		var difference = Math.abs(oldValue - newValue);
		var magnitude = Math.max(Math.abs(oldValue), Math.abs(newValue));
		return magnitude <= 1.0e-12 ? 0.0 : difference / magnitude;
	}
}
