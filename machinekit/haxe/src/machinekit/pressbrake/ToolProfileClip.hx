package machinekit.pressbrake;

/** Half-plane clipping for the convex triangles used by tooling collision geometry. */
class ToolProfileClip {
	public static function below(points:Array<ToolProfilePoint>, height:Float):Array<ToolProfilePoint> {
		var result:Array<ToolProfilePoint> = [];
		for (i in 0...points.length) {
			var a = points[i], b = points[(i + 1) % points.length];
			var aInside = a.z <= height, bInside = b.z <= height;
			if (aInside) append(result, a);
			if (aInside != bInside) {
				var fraction = (height - a.z) / (b.z - a.z);
				append(result, new ToolProfilePoint(a.y + fraction * (b.y - a.y), height));
			}
		}
		if (result.length > 1 && same(result[0], result[result.length - 1])) result.pop();
		return result;
	}

	static function append(points:Array<ToolProfilePoint>, point:ToolProfilePoint):Void {
		if (points.length == 0 || !same(points[points.length - 1], point)) points.push(point);
	}

	static function same(a:ToolProfilePoint, b:ToolProfilePoint):Bool
		return Math.abs(a.y - b.y) <= 1e-9 && Math.abs(a.z - b.z) <= 1e-9;
}
