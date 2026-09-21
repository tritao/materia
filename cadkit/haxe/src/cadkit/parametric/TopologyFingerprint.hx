package cadkit.parametric;

import CadKit;
import cadkit.Shape;
import cadkit.parametric.ParametricError;

/** Small geometric fallback key used when an operation has no direct mapping. */
class TopologyFingerprint {
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
		measure:Float) {
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
		measure:Float):TopologyFingerprint {
		return new TopologyFingerprint(
			kind, surfaceKind, curveKind, x, y, z, dx, dy, dz, measure);
	}

	public static function capture(shape:Shape):TopologyFingerprint {
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
				shape.faceArea());
		} else if (kind == CadKit.ShapeKind.Edge) {
			var tangent = shape.tangentAt();
			var endpoint = shape.subshape(CadKit.ShapeKind.Vertex, 0);
			var edgeCenter = endpoint.position();
			endpoint.close();
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
				shape.edgeLength());
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
				0.0);
		} else {
			throw new ParametricError("topology references require faces, edges, or vertices");
		}
	}

	public function score(candidate:Shape):Float {
		if (candidate.kind() != kind)
			return -1.0e30;

		var distance = distanceTo(candidate);
		if (kind == CadKit.ShapeKind.Face) {
			if (candidate.surfaceKind() != surfaceKind)
				return -1.0e30;
			var normal = candidate.faceNormal();
			var dot = dx * normal.get_x() + dy * normal.get_y() + dz * normal.get_z();
			if (dot < 0.98)
				return -1.0e30;
			return dot * 1000.0 - distance - relativeDifference(measure, candidate.faceArea());
		} else if (kind == CadKit.ShapeKind.Edge) {
			if (candidate.curveKind() != curveKind)
				return -1.0e30;
			var tangent = candidate.tangentAt();
			var tangentDot = dx * tangent.get_x() + dy * tangent.get_y() + dz * tangent.get_z();
			if (tangentDot < 0.0)
				tangentDot = -tangentDot;
			if (tangentDot < 0.98)
				return -1.0e30;
			return tangentDot * 1000.0 - distance - relativeDifference(measure, candidate.edgeLength());
		} else if (kind == CadKit.ShapeKind.Vertex) {
			return -distance;
		}
		return -1.0e30;
	}

	private function distanceTo(candidate:Shape):Float {
		var point:CadKit.Vec3;
		if (kind == CadKit.ShapeKind.Face) {
			point = candidate.center();
		} else if (kind == CadKit.ShapeKind.Edge) {
			var endpoint = candidate.subshape(CadKit.ShapeKind.Vertex, 0);
			point = endpoint.position();
			endpoint.close();
		} else if (kind == CadKit.ShapeKind.Vertex) {
			point = candidate.position();
		} else {
				return 1.0e30;
		}
		var offsetX = point.get_x() - x;
		var offsetY = point.get_y() - y;
		var offsetZ = point.get_z() - z;
		return offsetX * offsetX + offsetY * offsetY + offsetZ * offsetZ;
	}

	private function relativeDifference(oldValue:Float, newValue:Float):Float {
		var difference = oldValue - newValue;
		if (difference < 0.0)
			difference = -difference;
		var magnitude = oldValue;
		if (magnitude < 0.0)
			magnitude = -magnitude;
		if (magnitude < 1.0)
			magnitude = 1.0;
		return difference / magnitude;
	}
}
