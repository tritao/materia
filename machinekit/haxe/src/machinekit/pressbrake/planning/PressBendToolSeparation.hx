package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendZone;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.WallOutline;
import machinekit.pressbrake.ToolContactPrism;

/** Exact support intervals certify clearance; failure to separate is not a collision verdict. */
class PressBendToolSeparation {
	public static function clear(zone:BendZone,placement:Location,tool:ToolContactPrism):Bool {
		if (PressBendToolBounds.separated(zone,placement,tool)) return true;
		var frame = placement.compose(zone.frame), end = placement.compose(zone.endFrame());
		var axes = [new Vector(1,0,0),new Vector(0,1,0),new Vector(0,0,1),
			frame.direction(new Vector(1,0,0)),frame.direction(new Vector(0,0,1)),
			end.direction(new Vector(1,0,0)),end.direction(new Vector(0,0,1))];
		var lowY = Math.POSITIVE_INFINITY, highY = Math.NEGATIVE_INFINITY;
		var lowZ = Math.POSITIVE_INFINITY, highZ = Math.NEGATIVE_INFINITY;
		for (loop in tool.section.loops()) for (segment in loop) for (p in segment.extrema()) {
			lowY = Math.min(lowY,p.x); highY = Math.max(highY,p.x);
			lowZ = Math.min(lowZ,p.y); highZ = Math.max(highZ,p.y);
		}
		var middle = (lowY+highY)/2;
		for (interval in [[lowY,middle],[middle,highY]]) {
			var clip = WallOutline.polygon([new Point2(interval[0],lowZ-1),new Point2(interval[1],lowZ-1),
				new Point2(interval[1],highZ+1),new Point2(interval[0],highZ+1)]);
			for (section in OutlineBoolean.intersection(tool.section,clip,"tool-support-interval")) {
				var separated = false;
				for (axis in axes) {
					var stock = PressBendToolBounds.projectionAlong(zone,placement,axis), cap = project(section,axis);
					var first = axis.x*tool.start, last = axis.x*tool.end;
					var low = cap[0]+Math.min(first,last), high = cap[1]+Math.max(first,last);
					if (stock[1] <= low+1e-8 || high <= stock[0]+1e-8) { separated = true; break; }
				}
				if (!separated) return false;
			}
		}
		return true;
	}
	static function project(section:WallOutline,axis:Vector):Array<Float> {
		var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
		for (loop in section.loops()) for (segment in loop) {
			var points = [segment.start,segment.end];
			if (!segment.isLine()) {
				var center = segment.arcCenter(), angle = Math.atan2(axis.z,axis.y);
				for (a in [angle,angle+Math.PI]) {
					var point = new Point2(center.x+segment.radius*Math.cos(a),center.y+segment.radius*Math.sin(a));
					if (segment.fractionAt(point) <= 1+1e-12) points.push(point);
				}
			}
			for (point in points) {
				var value = axis.y*point.x+axis.z*point.y;
				low = Math.min(low,value); high = Math.max(high,value);
			}
		}
		return [low,high];
	}
}
