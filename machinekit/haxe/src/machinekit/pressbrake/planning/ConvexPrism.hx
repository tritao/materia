package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.SheetDiagnosis;

/** Immutable convex polygon extrusion, in press-frame mm. Curves must be decomposed before construction. */
class ConvexPrism {
	public final id:String;
	public final extrusion:Vector;
	final base:Array<Vector>;
	final corners:Array<Vector>;
	final normals:Array<Vector>;
	final directions:Array<Vector>;

	public function new(id:String, polygon:Array<Vector>, extrusion:Vector) {
		if (id == null || id.length == 0 || polygon == null || polygon.length < 3 || extrusion == null)
			throw new SheetDiagnosis("press.convex-prism", "A prism needs an identity, polygon and extrusion");
		this.id = id; this.extrusion = extrusion; base = polygon.copy();
		for (point in base) if (point == null) fail("Polygon has a null vertex");
		var origin = base[0], area = new Vector();
		for (i in 0...base.length)
			area = area.add(base[i].subtract(origin).cross(base[(i + 1) % base.length].subtract(origin)));
		if (area.length() <= 1e-8) fail("Polygon has zero area");
		var normal = area.normalized();
		if (Math.abs(extrusion.dot(normal)) <= 1e-8) fail("Extrusion has no thickness perpendicular to the polygon");
		for (point in base) if (Math.abs(point.subtract(origin).dot(normal)) > 1e-8) fail("Polygon is not planar");
		normals = [normal]; directions = [extrusion.normalized()]; corners = base.copy();
		for (point in base) corners.push(point.add(extrusion));
		for (i in 0...base.length) {
			var edge = base[(i + 1) % base.length].subtract(base[i]);
			if (edge.length() <= 1e-8) fail("Polygon has a zero-length edge");
			var inward = normal.cross(edge).normalized();
			for (point in base) if (point.subtract(base[i]).dot(inward) < -1e-8) fail("Polygon is not convex");
			directions.push(edge.normalized()); normals.push(edge.cross(extrusion).normalized());
		}
	}
	function fail(reason:String):Void throw new SheetDiagnosis("press.convex-prism", reason, [id]);
	public function vertices():Array<Vector> return corners.copy();
	public function faceNormals():Array<Vector> return normals.copy();
	public function edgeDirections():Array<Vector> return directions.copy();
	/** Euclidean inset of every physical face plane; null means no resolvable inset material remains. */
	public function inset(distance:Float):Null<ConvexPrism> {
		if (!Math.isFinite(distance) || distance < 0) fail("Inset distance must be finite nonnegative mm");
		if (distance == 0) return this;
		var thickness = Math.abs(extrusion.dot(normals[0]));
		if (thickness - 2 * distance <= 1e-8) return null;
		var center = new Vector();
		for (point in base) center = center.add(point.scale(1.0 / base.length));
		var polygon = base.copy();
		for (i in 0...base.length) {
			var origin = base[i], edge = base[(i + 1) % base.length].subtract(origin);
			var inward = edge.cross(extrusion).normalized();
			if (center.subtract(origin).dot(inward) < 0) inward = inward.scale(-1);
			var clipped:Array<Vector> = [];
			for (j in 0...polygon.length) {
				var a = polygon[j], b = polygon[(j + 1) % polygon.length];
				var da = a.subtract(origin).dot(inward) - distance, db = b.subtract(origin).dot(inward) - distance;
				if (da >= 0) clipped.push(a);
				if ((da >= 0) != (db >= 0)) clipped.push(a.add(b.subtract(a).scale(da / (da - db))));
			}
			polygon = [];
			for (point in clipped)
				if (polygon.length == 0 || point.subtract(polygon[polygon.length - 1]).length() > 1e-8) polygon.push(point);
			if (polygon.length > 1 && polygon[0].subtract(polygon[polygon.length - 1]).length() <= 1e-8) { var removed = polygon.pop(); }
			if (polygon.length < 3) return null;
		}
		var area = new Vector(), origin = polygon[0];
		for (i in 0...polygon.length)
			area = area.add(polygon[i].subtract(origin).cross(polygon[(i + 1) % polygon.length].subtract(origin)));
		if (area.length() <= 1e-8) return null;
		var capShift = extrusion.scale(distance / thickness);
		return new ConvexPrism(id, [for (point in polygon) point.add(capShift)], extrusion.scale(1 - 2 * distance / thickness));
	}
	public function placed(location:Location):ConvexPrism
		return new ConvexPrism(id, [for (point in base) location.plane.toWorld(point)], location.direction(extrusion));
}
