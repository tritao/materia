package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.OutlineGeometry;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;

/** One common nominal X/R setting derived from a placed sheet boundary and real finger datums. */
class BendRegistration {
	public final x:Float;
	public final r:Float;
	final contactPoints:Array<Vector>;
	function new(x:Float, r:Float, contacts:Array<Vector>) {
		this.x = x; this.r = r; contactPoints = contacts.copy();
	}
	public function contacts():Array<Vector> return contactPoints.copy();

	public static function plan(sheet:Sheet, pose:PressSheetPose, gauge:GaugeEdge,
			orientation:BendOrientation, press:PressGeometry):BendRegistration {
		var boundary = sheet.wall(gauge.wall).outline.segment(gauge.edge.loop, gauge.edge.segment);
		var plane = pose.wall(gauge.wall).plane;
		var depth = orientation.faceAUp ? sheet.thickness() : 0.0;
		var fingers = press.fingerContacts();
		if (fingers.length < 2)
			throw new SheetDiagnosis("press.gauge-fingers", "Registration needs at least two physical finger datums");
		var candidates:Array<Array<Vector>> = [];
		var a = plane.xDirection.x, b = plane.yDirection.x, norm = a * a + b * b;
		if (norm < 1e-16)
			throw new SheetDiagnosis("press.gauge-edge-direction", "The selected wall cannot span the lateral finger positions", ["wall:" + gauge.wall]);
		for (finger in fingers) {
			var c = finger.x - plane.origin.x + plane.normal.x * depth;
			var center = new Point2(a * c / norm, b * c / norm);
			var reachX = 0.0, reachY = 0.0;
			for (point in boundary.extrema()) {
				reachX = Math.max(reachX, Math.abs(point.x - center.x));
				reachY = Math.max(reachY, Math.abs(point.y - center.y));
			}
			var scale = (reachX + reachY + 1) / Math.sqrt(norm);
			var crossing = Segment.line(new Point2(center.x - b * scale, center.y + a * scale),
				new Point2(center.x + b * scale, center.y - a * scale));
			var intersections = OutlineGeometry.intersections(boundary, crossing);
			if (intersections.length == 0)
				throw new SheetDiagnosis("press.gauge-edge-miss", "Selected boundary does not reach finger " + finger.id,
					["wall:" + gauge.wall, finger.id]);
			var points:Array<Vector> = [];
			for (point in intersections) {
				var world = plane.toWorld(new Vector(point.x, point.y, -depth));
				var x = world.y - finger.y, r = world.z - finger.z;
				if (x >= press.backgaugeX.lower - 1e-8 && x <= press.backgaugeX.upper + 1e-8
					&& r >= press.backgaugeR.lower - 1e-8 && r <= press.backgaugeR.upper + 1e-8) points.push(world);
			}
			if (points.length == 0)
				throw new SheetDiagnosis("press.gauge-travel", "All boundary contacts at finger " + finger.id + " exceed nominal X/R travel",
					["wall:" + gauge.wall, finger.id]);
			points.sort(function(left:Vector, right:Vector):Int {
				return left.y < right.y ? -1 : left.y > right.y ? 1 : left.z < right.z ? -1 : left.z > right.z ? 1 : 0;
			});
			candidates.push(points);
		}
		var result = common(candidates, fingers, 0, 0, 0, []);
		if (result == null)
			throw new SheetDiagnosis("press.gauge-incompatible", "The selected boundary has no common X/R setting for all fingers", ["wall:" + gauge.wall]);
		return result;
	}
	static function common(candidates:Array<Array<Vector>>, fingers:Array<PressFingerDatum>, index:Int,
			x:Float, r:Float, selected:Array<Vector>):Null<BendRegistration> {
		if (index == candidates.length) return new BendRegistration(x, r, selected);
		for (point in candidates[index]) {
			var nextX = point.y - fingers[index].y, nextR = point.z - fingers[index].z;
			if (index != 0 && (Math.abs(nextX - x) > 1e-8 || Math.abs(nextR - r) > 1e-8)) continue;
			var next = selected.copy(); next.push(point);
			var result = common(candidates, fingers, index + 1, nextX, nextR, next);
			if (result != null) return result;
		}
		return null;
	}
}
